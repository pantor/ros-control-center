function ROSCCConfig($routeProvider, localStorageServiceProvider) {
  $routeProvider.when('/', {
    templateUrl: 'app/control/control.html',
    controller: 'ControlController',
    controllerAs: 'vm'
  }).when('/settings', {
    templateUrl: 'app/settings/settings.html',
    controller: 'SettingsController',
    controllerAs: 'vm'
  }).otherwise({ redirectTo: '/' });

  localStorageServiceProvider.setPrefix('roscc');
}

angular.module('roscc', ['ngRoute', 'ui.bootstrap', 'LocalStorageModule']).config(ROSCCConfig);
let ros;
let isConnected = false;

class ControlController {
  constructor($timeout, $interval, Settings, Domains) {
    this.$timeout = $timeout;
    this.Domains = Domains;

    this.isConnected = isConnected;
    this.setting = Settings.get();
    this.maxConsoleEntries = 200;

    // Load ROS connection and keep trying if it fails
    this.newRosConnection();
    $interval(() => {
      this.newRosConnection();
    }, 1000); // [ms]

    this.resetData();
    if (isConnected) {
      this.onConnected();
    }
  }

  // The active domain shows further information in the center view
  setActiveDomain(domain) {
    this.activeDomain = domain;
  }

  getDomains() {
    const allData = this.data.topics.concat(this.data.services, this.data.nodes);
    const domains = this.Domains.getDomains(allData);

    if (!this.activeDomain) {
      this.setActiveDomain(domains[0]);
    }
    return domains;
  }

  getGlobalParameters() {
    return this.Domains.getGlobalParameters(this.data.parameters);
  }

  resetData() {
    this.data = {
      rosout: [],
      topics: [],
      nodes: [],
      parameters: [],
      services: []
    };
  }

  newRosConnection() {
    if (isConnected || !this.setting) {
      return;
    }

    if (ros) {
      ros.close(); // Close old connection
      ros = false;
      return;
    }

    ros = new ROSLIB.Ros({ url: `ws://${ this.settings.address }:${ this.settings.port }` });

    ros.on('connection', () => {
      this.onConnected();
      isConnected = true;
      this.isConnected = isConnected;
    });

    ros.on('error', () => {
      isConnected = false;
      this.isConnected = isConnected;
    });

    ros.on('close', () => {
      isConnected = false;
      this.isConnected = isConnected;
    });
  }

  onConnected() {
    // wait a moment until ROS is loaded and initialized
    this.$timeout(() => {
      this.loadData();

      this.setConsole();
      if (this.setting.battery) {
        this.setBattery();
      }
    }, 1000); // [ms]
  }

  // Setup of console (in the right sidebar)
  setConsole() {
    const consoleTopic = new ROSLIB.Topic({
      ros,
      name: this.setting.log,
      messageType: 'rosgraph_msgs/Log'
    });
    consoleTopic.subscribe(message => {
      const nameArray = message.name.split('/');
      const d = new Date(message.header.stamp.secs * 1E3 + message.header.stamp.nsecs * 1E-6);

      message.abbr = nameArray.length > 1 ? nameArray[1] : message.name;

      // String formatting of message time and date
      function addZero(i) {
        return i < 10 ? `0${ i }` : `${ i }`;
      }
      message.dateString = `${ addZero(d.getHours()) }:
      ${ addZero(d.getMinutes()) }:
      ${ addZero(d.getSeconds()) }.
      ${ addZero(d.getMilliseconds()) }`;
      this.data.rosout.unshift(message);

      if (this.data.rosout.length > this.maxConsoleEntries) {
        this.data.rosout.pop();
      }
    });
  }

  // Setup battery status
  setBattery() {
    const batteryTopic = new ROSLIB.Topic({
      ros,
      name: this.setting.batteryTopic,
      messageType: 'std_msgs/Float32'
    });
    batteryTopic.subscribe(message => {
      this.batteryStatus = message.data;
    });
  }

  // Load structure, all data, parameters, topics, services, nodes...
  loadData() {
    this.resetData();

    ros.getTopics(topics => {
      angular.forEach(topics, name => {
        this.data.topics.push({ name });

        ros.getTopicType(name, type => {
          _.findWhere(this.data.topics, { name }).type = type;
        });
      });
    });

    ros.getServices(services => {
      angular.forEach(services, name => {
        this.data.services.push({ name });

        ros.getServiceType(name, type => {
          _.findWhere(this.data.services, { name }).type = type;
        });
      });
    });

    ros.getParams(params => {
      angular.forEach(params, name => {
        const param = new ROSLIB.Param({ ros, name });
        this.data.parameters.push({ name });

        param.get(value => {
          _.findWhere(this.data.parameters, { name }).value = value;
        });
      });
    });

    ros.getNodes(nodes => {
      angular.forEach(nodes, name => {
        this.data.nodes.push({ name });
      });
    });
  }
}

angular.module('roscc').controller('ControlController', ControlController);
class DomainsService {
  static filterAdvanced(entry, advanced) {
    const entryArray = entry.split('/');
    if (advanced) {
      return true;
    }
    if (!entry || _.isEmpty(entryArray)) {
      return false;
    }
    return _.last(entryArray)[0] === _.last(entryArray)[0].toUpperCase();
  }

  static getDomains(array) {
    const result = [];
    angular.forEach(array, entry => {
      const nameArray = entry.name.split('/');
      if (nameArray.length > 1) {
        result.push(nameArray[1]);
      }
    });
    return _.uniq(result).sort();
  }

  static getGlobalParameters(array) {
    const result = [];
    angular.forEach(array, entry => {
      const nameArray = entry.name.split('/');
      if (nameArray.length === 2) {
        entry.abbr = _.last(nameArray);
        result.push(entry);
      }
    });
    return result;
  }

  static getDataForDomain(array, domainName, advanced) {
    const result = [];
    angular.forEach(array, entry => {
      const nameArray = entry.name.split('/');
      if (nameArray.length > 1 && nameArray[1] === domainName && this.filterAdvanced(entry.name, advanced)) {
        entry.abbr = nameArray.slice(2).join(' ');
        result.push(entry);
      }
    });
    return result;
  }
}

// Filter advanced topics, services, parameters by checking the beginning capital letter
angular.module('roscc').service('Domains', DomainsService);
class QuaternionsService {
  static getRoll(q) {
    if (!q) {
      return '';
    }
    const rad = Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    return 180 / Math.PI * rad;
  }

  static getPitch(q) {
    if (!q) {
      return '';
    }
    const rad = Math.asin(2 * (q.w * q.y - q.z * q.x));
    return 180 / Math.PI * rad;
  }

  static getYaw(q) {
    if (!q) {
      return '';
    }
    const rad = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    return 180 / Math.PI * rad;
  }

  static getInit() {
    return { w: 1, x: 0, y: 0, z: 0 };
  }
}

// Quaternions to Euler angles converter
angular.module('roscc').service('Quaternions', QuaternionsService);
function ParamaterDirective() {
  return {
    scope: { parameter: '=' },
    templateUrl: 'app/parameters/parameters.html',
    controllerAs: 'vm',
    controller($scope) {
      const param = new ROSLIB.Param({ ros, name: $scope.parameter.name });

      this.parameter = $scope.parameter;
      this.setValue = setValue;

      function setValue(value) {
        param.set(value);
      }
    }
  };
}

angular.module('roscc').directive('ccParameter', ParamaterDirective);
function NavbarDirective($location) {
  return {
    templateUrl: 'app/navbar/navbar.html',
    controllerAs: 'vm',
    controller() {
      this.isPath = isPath;

      function isPath(path) {
        return $location.path() === path;
      }
    }
  };
}

angular.module('roscc').directive('ccNavbar', NavbarDirective);
function serviceDirective() {
  return {
    scope: { service: '=' },
    template: '<ng-include src="vm.fileName"></ng-include>',
    controllerAs: 'vm',
    controller($scope, $timeout, $http) {
      const path = 'app/services/';

      this.service = $scope.service;
      this.callService = callService;
      this.fileName = `${ path }default.html`;

      // Check if file exists
      $scope.$watch('service.type', () => {
        if (!$scope.service.type) {
          return;
        }
        const fileName = `${ path }${ $scope.service.type }.html`;

        this.service = $scope.service;
        $http.get(fileName).then(result => {
          if (result.data) {
            this.fileName = fileName;
          }
        });
      });

      function callService(input, isJSON) {
        const data = isJSON ? angular.fromJSON(input) : input;
        const service = new ROSLIB.Service({
          ros,
          name: this.service.name,
          serviceType: this.service.type
        });
        const request = new ROSLIB.ServiceRequest(data);

        service.callService(request, result => {
          $timeout(() => {
            this.result = result;
          });
        });
      }
    }
  };
}

angular.module('roscc').directive('ccService', serviceDirective);
class SettingsController {
  constructor(localStorageService, Settings) {
    this.Settings = Settings;

    this.settings = Settings.getSettings() || [Settings.getDefaultSetting()];
    this.index = Settings.getIndex();

    if (!this.index || this.index > this.settings.length) {
      this.index = '0';
    }
  }

  save() {
    this.Settings.save(this.settings, this.index);
  }

  add() {
    this.settings.push(this.Settings.getDefaultSetting()); // Clone object
    this.index = String(this.settings.length - 1);
    this.save();
  }

  remove() {
    this.settings.splice(this.index, 1);
    this.index = '0';

    if (!this.settings.length) {
      this.add();
    }
    this.save();
  }
}

angular.module('roscc').controller('SettingsController', SettingsController);
class SettingsService {
  constructor($location, localStorageService) {
    this.$location = $location;
    this.localStorageService = localStorageService;
  }

  load() {
    this.index = this.localStorageService.get('selectedSettingIndex');
    this.settings = this.localStorageService.get('settings');
    if (this.settings && this.index) {
      this.setting = this.settings[this.index];
    }

    // If there are no saved settings, redirect to /settings for first setting input
    if (!this.setting) {
      this.$location.path('/settings').replace();
    }
  }

  save(newSettings, newIndex) {
    this.settings = newSettings;
    this.index = newIndex;
    this.localStorageService.set('selectedSettingIndex', newIndex);
    this.localStorageService.set('settings', newSettings);
  }

  get() {
    if (!this.setting) {
      this.load();
    }

    return this.setting;
  }

  getIndex() {
    if (!this.setting) {
      this.load();
    }

    return this.index;
  }

  getSettings() {
    if (!this.setting) {
      this.load();
    }

    return this.settings;
  }

  static getDefaultSetting() {
    return {
      name: 'Robot Name',
      address: '127.0.0.1', // use localhost
      port: 9090, // default port of rosbridge_server
      log: '/rosout',
      imagePreview: { port: 0, quality: 70, width: 640, height: 480 },
      battery: false,
      batteryTopic: '',
      advanced: false
    };
  }
}

angular.module('roscc').service('Settings', SettingsService);
function topicDirective() {
  return {
    scope: { topic: '=' },
    template: '<ng-include src="vm.fileName"></ng-include>',
    controllerAs: 'vm',
    controller($scope, $timeout, $http, Settings, Quaternions) {
      const roslibTopic = new ROSLIB.Topic({
        ros,
        name: $scope.topic.name,
        messageType: $scope.topic.type
      });
      const path = 'app/topics/';

      this.topic = $scope.topic;
      this.toggleSubscription = toggleSubscription;
      this.publishMessage = publishMessage;
      this.isSubscribing = false;
      this.setting = Settings.get();
      this.Quaternions = Quaternions;
      this.fileName = `${ path }default.html`;

      // Check if file exists
      $scope.$watch('topic.type', () => {
        if (!$scope.topic.type) {
          return;
        }
        const fileName = `${ path }${ $scope.topic.type }.html`;

        this.topic = $scope.topic;
        $http.get(fileName).then(result => {
          if (result.data) {
            this.fileName = fileName;
          }
        });
      });

      function toggleSubscription(data) {
        if (!data) {
          roslibTopic.subscribe(message => {
            $timeout(() => {
              this.message = message;
            });
          });
        } else {
          roslibTopic.unsubscribe();
        }
        this.isSubscribing = !data;
      }

      function publishMessage(input, isJSON) {
        const data = isJSON ? angular.fromJSON(input) : input;
        const message = new ROSLIB.Message(data);
        roslibTopic.publish(message);
      }
    }
  };
}

angular.module('roscc').directive('ccTopic', topicDirective);