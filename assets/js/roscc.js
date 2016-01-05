function RosccConfig($routeProvider, localStorageServiceProvider) {
  $routeProvider
    .when('/', {
      templateUrl: 'app/control/control.html',
      controller: 'ControlController',
      controllerAs: 'vm',
    })
    .when('/settings', {
      templateUrl: 'app/settings/settings.html',
      controller: 'SettingsController',
      controllerAs: 'vm',
    })
    .otherwise({ redirectTo: '/' });

  localStorageServiceProvider
    .setPrefix('roscc');
}

angular.module('roscc', ['ngRoute', 'ui.bootstrap', 'LocalStorageModule']).config(RosccConfig);

function DomainsFactory() {
  return {
    filterAdvanced: function (entry, advanced) {
      var entryArray = entry.split('/');

      if (advanced) {
        return true;
      }

      if (!entry || _.isEmpty(entryArray)) {
        return false;
      }

      return (_.last(entryArray)[0] === _.last(entryArray)[0].toUpperCase());
    },
    getDomains: function (array) {
      var result = [];
      angular.forEach(array, function (entry) {
        var nameArray = entry.name.split('/');
        if (nameArray.length > 1) {
          result.push(nameArray[1]);
        }
      });
      return _.uniq(result).sort();
    },
    getGlobalParameters: function (array) {
      var result = [];
      angular.forEach(array, function (entry) {
        var nameArray = entry.name.split('/');
        if (nameArray.length === 2) {
          entry.abbr = _.last(nameArray);
          result.push(entry);
        }
      });
      return result;
    },
    getDataForDomain: function (array, domainName) {
      var result = [];
      angular.forEach(array, function (entry) {
        var nameArray = entry.name.split('/');
        if (nameArray.length > 1 && nameArray[1] === domainName) {
          entry.abbr = nameArray.slice(2).join(' ');
          result.push(entry);
        }
      });
      return result;
    },
  };
}

// Filter advanced topics, services, parameters by checking the beginning capital letter
angular.module('roscc').factory('Domains', DomainsFactory);

function QuaternionsFactory() {
  return {
    getRoll: function (q) {
      if (!q) {
        return '';
      }
      var rad = Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
      return 180 / Math.PI * rad;
    },
    getPitch: function (q) {
      if (!q) {
        return '';
      }
      var rad = Math.asin(2 * (q.w * q.y - q.z * q.x));
      return 180 / Math.PI * rad;
    },
    getYaw: function (q) {
      if (!q) {
        return '';
      }
      var rad = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      return 180 / Math.PI * rad;
    },
    getInit: function () {
      return { w: 1, x: 0, y: 0, z: 0 };
    },
  };
}

// Filter advanced topics, services, parameters by checking the beginning capital letter
angular.module('roscc').factory('Quaternions', QuaternionsFactory);

var ros;
var isConnected = false;

function ControlController($timeout, $interval, Settings, Domains) {
  var vm = this;
  var maxConsoleEntries = 200;

  vm.Domains = Domains;
  vm.setActiveDomain = setActiveDomain;
  vm.getDomains = getDomains;
  vm.getGlobalParameters = getGlobalParameters;
  vm.onConnected = onConnected;
  vm.newRosConnection = newRosConnection;
  vm.isConnected = isConnected;
  vm.setting = Settings.get();


  // The active domain shows further information in the center view
  function setActiveDomain(domain) {
    vm.activeDomain = domain;
  }

  function getDomains() {
    var allData = vm.data.topics.concat(vm.data.services, vm.data.nodes);
    var domains = Domains.getDomains(allData);

    if (!vm.activeDomain) {
      setActiveDomain(domains[0]);
    }
    return domains;
  }

  function getGlobalParameters() {
    return Domains.getGlobalParameters(vm.data.parameters);
  }

  function resetData() {
    vm.data = {
      rosout: [],
      topics: [],
      nodes: [],
      parameters: [],
      services: [],
    };
  }

  function newRosConnection() {
    if (isConnected || !vm.setting) {
      return;
    }

    if (ros) {
      ros.close(); // Close old connection
      ros = false;
      return;
    }

    ros = new ROSLIB.Ros({ url: 'ws://' + vm.setting.address + ':' + vm.setting.port });

    ros.on('connection', function () {
      vm.onConnected();
      isConnected = true;
      vm.isConnected = isConnected;
    });

    ros.on('error', function () {
      isConnected = false;
      vm.isConnected = isConnected;
    });

    ros.on('close', function () {
      isConnected = false;
      vm.isConnected = isConnected;
    });
  }

  function onConnected() {
    $timeout(function () {
      loadData();

      setConsole();
      if (vm.setting.battery) {
        setBattery();
      }
    }, 500);
  }

  // Load ROS connection and keep trying if it fails
  newRosConnection();
  $interval(function () {
    newRosConnection();
  }, 1000); // [ms]

  resetData();
  if (isConnected) {
    vm.onConnected();
  }


  // Setup of console (in the right sidebar)
  function setConsole() {
    var consoleTopic = new ROSLIB.Topic({
      ros: ros,
      name: vm.setting.log,
      messageType: 'rosgraph_msgs/Log',
    });
    consoleTopic.subscribe(function (message) {
      var nameArray = message.name.split('/');
      var d = new Date(message.header.stamp.secs * 1E3 + message.header.stamp.nsecs * 1E-6);

      message.abbr = (nameArray.length > 1) ? nameArray[1] : message.name;

      // String formatting of message time and date
      function addZero(i) { return i < 10 ? '0' + i : i; }
      message.dateString = addZero(d.getHours()) + ':' +
        addZero(d.getMinutes()) + ':' +
        addZero(d.getSeconds()) + '.' +
        addZero(d.getMilliseconds());
      vm.data.rosout.unshift(message);

      if (vm.data.rosout.length > maxConsoleEntries) {
        vm.data.rosout.pop();
      }
    });
  }

  // Setup battery status
  function setBattery() {
    var batteryTopic = new ROSLIB.Topic({
      ros: ros,
      name: vm.setting.batteryTopic,
      messageType: 'std_msgs/Float32',
    });
    batteryTopic.subscribe(function (message) {
      vm.batteryStatus = message.data;
    });
  }

  // Load structure, all data, parameters, topics, services, nodes...
  function loadData() {
    resetData();

    ros.getTopics(function (topics) {
      angular.forEach(topics, function (topic) {
        vm.data.topics.push({ name: topic });

        ros.getTopicType(topic, function (type) {
          _.findWhere(vm.data.topics, { name: topic }).type = type;
        });
      });
    });

    ros.getServices(function (services) {
      angular.forEach(services, function (service) {
        vm.data.services.push({ name: service });

        ros.getServiceType(service, function (type) {
          _.findWhere(vm.data.services, { name: service }).type = type;
        });
      });
    });

    ros.getParams(function (params) {
      angular.forEach(params, function (param) {
        var p = new ROSLIB.Param({ ros: ros, name: param });

        vm.data.parameters.push({ name: param });

        p.get(function (value) {
          _.findWhere(vm.data.parameters, { name: param }).value = value;
        });
      });
    });

    ros.getNodes(function (nodes) {
      angular.forEach(nodes, function (entry) {
        vm.data.nodes.push({ name: entry });
      });
    });
  }
}

angular.module('roscc').controller('ControlController', ControlController);

function ParamaterDirective() {
  return {
    scope: { parameter: '=' },
    templateUrl: 'app/parameters/parameters.html',
    controllerAs: 'vm',
    controller: function ($scope) {
      var vm = this;
      var p = new ROSLIB.Param({ ros: ros, name: $scope.parameter.name });

      vm.parameter = $scope.parameter;
      vm.setValue = setValue;

      function setValue(value) {
        p.set(value);
      }
    },
  };
}

angular.module('roscc').directive('ccParameter', ParamaterDirective);

function NavbarDirective($location) {
  return {
    templateUrl: 'app/navbar/navbar.html',
    controllerAs: 'vm',
    controller: function () {
      var vm = this;
      vm.isPath = isPath;

      function isPath(path) {
        return $location.path() === path;
      }
    },
  };
}

angular.module('roscc').directive('ccNavbar', NavbarDirective);

function serviceDirective(fileName) {
  return function () {
    return {
      scope: { service: '=' },
      templateUrl: 'app/services/' + fileName + '.html',
      controllerAs: 'vm',
      controller: function ($scope, $timeout) {
        var vm = this;
        vm.service = $scope.service;
        vm.callService = callService;
        vm.callServiceJSON = callServiceJSON;

        function callService(data) {
          var service = new ROSLIB.Service({
            ros: ros,
            name: vm.service.name,
            serviceType: vm.service.type,
          });
          var request = new ROSLIB.ServiceRequest(data);

          service.callService(request, function (result) {
            $timeout(function () {
              vm.result = result;
            });
          });
        }

        function callServiceJSON(data) {
          callService(angular.fromJSON(data));
        }
      },
    };
  };
}


angular.module('roscc')
  .directive('ccServiceDefault', serviceDirective('default'))

  .directive('ccServiceEmpty', serviceDirective('std_srvs/empty'))
  .directive('ccServiceTrigger', serviceDirective('std_srvs/trigger'))
  .directive('ccServiceMovingpiBool', serviceDirective('movingpi/bool'));

function SettingsController(localStorageService, Settings) {
  var vm = this;
  vm.add = add;
  vm.remove = remove;
  vm.save = save;
  vm.settings = Settings.getSettings() || [Settings.getDefaultSetting()];
  vm.index = Settings.getIndex();


  if (!vm.index || vm.index > vm.settings.length) {
    vm.index = '0';
  }

  function save() {
    Settings.save(vm.settings, vm.index);
  }

  function add() {
    vm.settings.push(Settings.getDefaultSetting()); // Clone object
    vm.index = String(vm.settings.length - 1);
    save();
  }

  function remove() {
    vm.settings.splice(vm.index, 1);
    vm.index = '0';

    if (!vm.settings.length) {
      vm.add();
    }
    save();
  }
}

angular.module('roscc').controller('SettingsController', SettingsController);

function SettingsFactory($location, localStorageService) {
  var setting;
  var settings;
  var index;

  return {
    load: function () {
      index = localStorageService.get('selectedSettingIndex');
      settings = localStorageService.get('settings');
      if (settings && index) {
        setting = settings[index];
      }

      // If there are no saved settings, redirect to /settings for first setting input
      if (!setting) {
        $location.path('/settings').replace();
      }
    },
    save: function (newSettings, newIndex) {
      settings = newSettings;
      index = newIndex;
      localStorageService.set('selectedSettingIndex', newIndex);
      localStorageService.set('settings', newSettings);
    },
    get: function () {
      if (!setting) {
        this.load();
      }

      return setting;
    },
    getIndex: function () {
      if (!setting) {
        this.load();
      }

      return index;
    },
    getSettings: function () {
      if (!setting) {
        this.load();
      }

      return settings;
    },
    getDefaultSetting: function () {
      return {
        name: 'New Setting',
        address: location.hostname,
        port: 9090,
        log: '/rosout',
        imagePreview: { port: 0, quality: 70, width: 640, height: 480 },
        battery: true,
        batteryTopic: '',
        advanced: false,
      };
    },
  };
}

angular.module('roscc').factory('Settings', SettingsFactory);

function topicDirective(fileName) {
  return function () {
    return {
      scope: { topic: '=' },
      templateUrl: 'app/topics/' + fileName + '.html',
      controllerAs: 'vm',
      controller: function ($scope, $timeout, Settings, Quaternions) {
        var vm = this;
        var roslibTopic = new ROSLIB.Topic({
          ros: ros,
          name: $scope.topic.name,
          messageType: $scope.topic.type,
        });

        vm.topic = $scope.topic;
        vm.toggleSubscription = toggleSubscription;
        vm.publishMessage = publishMessage;
        vm.publishMessageJSON = publishMessageJSON;
        vm.isSubscribing = false;
        vm.setting = Settings.get();
        vm.Quaternions = Quaternions;


        function toggleSubscription(data) {
          if (!data) {
            roslibTopic.subscribe(function (message) {
              $timeout(function () {
                vm.message = message;
              });
            });
          } else {
            roslibTopic.unsubscribe();
          }

          vm.isSubscribing = !data;
        }

        function publishMessage(data) {
          var message = new ROSLIB.Message(data);
          roslibTopic.publish(message);
        }

        function publishMessageJSON(data) {
          publishMessage(angular.fromJSON(data));
        }
      },
    };
  };
}


angular.module('roscc')
  .directive('ccTopicDefault', topicDirective('default'))

  .directive('ccTopicNumber', topicDirective('std_msgs/number'))

  .directive('ccTopicFluidPressure', topicDirective('sensor_msgs/fluid-pressure'))
  .directive('ccTopicIlluminance', topicDirective('sensor_msgs/illuminance'))
  .directive('ccTopicImage', topicDirective('sensor_msgs/image'))
  .directive('ccTopicImu', topicDirective('sensor_msgs/imu'))
  .directive('ccTopicJoy', topicDirective('sensor_msgs/joy'))
  .directive('ccTopicMagneticField', topicDirective('sensor_msgs/magnetic-field'))
  .directive('ccTopicRange', topicDirective('sensor_msgs/range'))
  .directive('ccTopicRelativeHumidity', topicDirective('sensor_msgs/relative-humidity'))
  .directive('ccTopicTemperature', topicDirective('sensor_msgs/temperature'))

  .directive('ccTopicPose', topicDirective('geometry_msgs/pose'))
  .directive('ccTopicPoseStamped', topicDirective('geometry_msgs/pose-stamped'))
  .directive('ccTopicPose2d', topicDirective('geometry_msgs/pose2D'))

  .directive('ccTopicFlypiMotorsThrust', topicDirective('flypi/motors-thrust'))
  .directive('ccTopicFlypiSteering', topicDirective('flypi/steering'))

  .directive('ccTopicVisionodometryMotion', topicDirective('visionodometry/motion'));
