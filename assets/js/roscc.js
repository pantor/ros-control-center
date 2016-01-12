'use strict';

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
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var ros = undefined;
var isConnected = false;

var ControlController = function () {
  function ControlController($timeout, $interval, Settings, Domains) {
    var _this = this;

    _classCallCheck(this, ControlController);

    this.$timeout = $timeout;
    this.Domains = Domains;

    this.isConnected = isConnected;
    this.setting = Settings.get();
    this.maxConsoleEntries = 200;

    // Load ROS connection and keep trying if it fails
    this.newRosConnection();
    $interval(function () {
      _this.newRosConnection();
    }, 1000); // [ms]

    this.resetData();
    if (isConnected) {
      this.onConnected();
    }
  }

  // The active domain shows further information in the center view

  _createClass(ControlController, [{
    key: 'setActiveDomain',
    value: function setActiveDomain(domain) {
      this.activeDomain = domain;
    }
  }, {
    key: 'getDomains',
    value: function getDomains() {
      var allData = this.data.topics.concat(this.data.services, this.data.nodes);
      var domains = this.Domains.getDomains(allData);

      if (!this.activeDomain) {
        this.setActiveDomain(domains[0]);
      }
      return domains;
    }
  }, {
    key: 'getGlobalParameters',
    value: function getGlobalParameters() {
      return this.Domains.getGlobalParameters(this.data.parameters);
    }
  }, {
    key: 'resetData',
    value: function resetData() {
      this.data = {
        rosout: [],
        topics: [],
        nodes: [],
        parameters: [],
        services: []
      };
    }
  }, {
    key: 'newRosConnection',
    value: function newRosConnection() {
      var _this2 = this;

      if (isConnected || !this.setting) {
        return;
      }

      if (ros) {
        ros.close(); // Close old connection
        ros = false;
        return;
      }

      ros = new ROSLIB.Ros({ url: 'ws://' + this.setting.address + ':' + this.setting.port });

      ros.on('connection', function () {
        _this2.onConnected();
        isConnected = true;
        _this2.isConnected = isConnected;
      });

      ros.on('error', function () {
        isConnected = false;
        _this2.isConnected = isConnected;
      });

      ros.on('close', function () {
        isConnected = false;
        _this2.isConnected = isConnected;
      });
    }
  }, {
    key: 'onConnected',
    value: function onConnected() {
      var _this3 = this;

      // wait a moment until ROS is loaded and initialized
      this.$timeout(function () {
        _this3.loadData();

        _this3.setConsole();
        if (_this3.setting.battery) {
          _this3.setBattery();
        }
      }, 1000); // [ms]
    }

    // Setup of console (in the right sidebar)

  }, {
    key: 'setConsole',
    value: function setConsole() {
      var _this4 = this;

      var consoleTopic = new ROSLIB.Topic({
        ros: ros,
        name: this.setting.log,
        messageType: 'rosgraph_msgs/Log'
      });
      consoleTopic.subscribe(function (message) {
        var nameArray = message.name.split('/');
        var d = new Date(message.header.stamp.secs * 1E3 + message.header.stamp.nsecs * 1E-6);

        message.abbr = nameArray.length > 1 ? nameArray[1] : message.name;

        // String formatting of message time and date
        function addZero(i) {
          return i < 10 ? '0' + i : i;
        }
        message.dateString = addZero(d.getHours()) + ':' + addZero(d.getMinutes()) + ':' + addZero(d.getSeconds()) + '.' + addZero(d.getMilliseconds());
        _this4.data.rosout.unshift(message);

        if (_this4.data.rosout.length > _this4.maxConsoleEntries) {
          _this4.data.rosout.pop();
        }
      });
    }

    // Setup battery status

  }, {
    key: 'setBattery',
    value: function setBattery() {
      var _this5 = this;

      var batteryTopic = new ROSLIB.Topic({
        ros: ros,
        name: this.setting.batteryTopic,
        messageType: 'std_msgs/Float32'
      });
      batteryTopic.subscribe(function (message) {
        _this5.batteryStatus = message.data;
      });
    }

    // Load structure, all data, parameters, topics, services, nodes...

  }, {
    key: 'loadData',
    value: function loadData() {
      var _this6 = this;

      this.resetData();

      ros.getTopics(function (topics) {
        angular.forEach(topics, function (name) {
          _this6.data.topics.push({ name: name });

          ros.getTopicType(name, function (type) {
            _.findWhere(_this6.data.topics, { name: name }).type = type;
          });
        });
      });

      ros.getServices(function (services) {
        angular.forEach(services, function (name) {
          _this6.data.services.push({ name: name });

          ros.getServiceType(name, function (type) {
            _.findWhere(_this6.data.services, { name: name }).type = type;
          });
        });
      });

      ros.getParams(function (params) {
        angular.forEach(params, function (name) {
          var param = new ROSLIB.Param({ ros: ros, name: name });
          _this6.data.parameters.push({ name: name });

          param.get(function (value) {
            _.findWhere(_this6.data.parameters, { name: name }).value = value;
          });
        });
      });

      ros.getNodes(function (nodes) {
        angular.forEach(nodes, function (name) {
          _this6.data.nodes.push({ name: name });
        });
      });
    }
  }]);

  return ControlController;
}();

angular.module('roscc').controller('ControlController', ControlController);
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var DomainsService = function () {
  function DomainsService() {
    _classCallCheck(this, DomainsService);
  }

  _createClass(DomainsService, [{
    key: 'filterAdvanced',
    value: function filterAdvanced(entry, advanced) {
      var entryArray = entry.split('/');
      if (advanced) {
        return true;
      }
      if (!entry || _.isEmpty(entryArray)) {
        return false;
      }
      return _.last(entryArray)[0] === _.last(entryArray)[0].toUpperCase();
    }
  }, {
    key: 'getDomains',
    value: function getDomains(array) {
      var result = [];
      angular.forEach(array, function (entry) {
        var nameArray = entry.name.split('/');
        if (nameArray.length > 1) {
          result.push(nameArray[1]);
        }
      });
      return _.uniq(result).sort();
    }
  }, {
    key: 'getGlobalParameters',
    value: function getGlobalParameters(array) {
      var result = [];
      angular.forEach(array, function (entry) {
        var nameArray = entry.name.split('/');
        if (nameArray.length === 2) {
          entry.abbr = _.last(nameArray);
          result.push(entry);
        }
      });
      return result;
    }
  }, {
    key: 'getDataForDomain',
    value: function getDataForDomain(array, domainName, advanced) {
      var _this = this;

      var result = [];
      angular.forEach(array, function (entry) {
        var nameArray = entry.name.split('/');
        if (nameArray.length > 1 && nameArray[1] === domainName && _this.filterAdvanced(entry.name, advanced)) {
          entry.abbr = nameArray.slice(2).join(' ');
          result.push(entry);
        }
      });
      return result;
    }
  }]);

  return DomainsService;
}();

// Filter advanced topics, services, parameters by checking the beginning capital letter

angular.module('roscc').service('Domains', DomainsService);
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var QuaternionsService = function () {
  function QuaternionsService() {
    _classCallCheck(this, QuaternionsService);
  }

  _createClass(QuaternionsService, [{
    key: 'getRoll',
    value: function getRoll(q) {
      if (!q) {
        return '';
      }
      var rad = Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
      return 180 / Math.PI * rad;
    }
  }, {
    key: 'getPitch',
    value: function getPitch(q) {
      if (!q) {
        return '';
      }
      var rad = Math.asin(2 * (q.w * q.y - q.z * q.x));
      return 180 / Math.PI * rad;
    }
  }, {
    key: 'getYaw',
    value: function getYaw(q) {
      if (!q) {
        return '';
      }
      var rad = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      return 180 / Math.PI * rad;
    }
  }, {
    key: 'getInit',
    value: function getInit() {
      return { w: 1, x: 0, y: 0, z: 0 };
    }
  }]);

  return QuaternionsService;
}();

// Quaternions to Euler angles converter

angular.module('roscc').service('Quaternions', QuaternionsService);
'use strict';

function NavbarDirective($location) {
  return {
    templateUrl: 'app/navbar/navbar.html',
    controllerAs: 'vm',
    controller: function controller() {
      this.isPath = isPath;

      function isPath(path) {
        return $location.path() === path;
      }
    }
  };
}

angular.module('roscc').directive('ccNavbar', NavbarDirective);
'use strict';

function ParamaterDirective() {
  return {
    scope: { parameter: '=' },
    templateUrl: 'app/parameters/parameters.html',
    controllerAs: 'vm',
    controller: function controller($scope) {
      var param = new ROSLIB.Param({ ros: ros, name: $scope.parameter.name });

      this.parameter = $scope.parameter;
      this.setValue = setValue;

      function setValue(value) {
        param.set(value);
      }
    }
  };
}

angular.module('roscc').directive('ccParameter', ParamaterDirective);
'use strict';

function serviceDirective() {
  return {
    scope: { service: '=' },
    template: '<ng-include src=\"vm.fileName\"></ng-include>',
    controllerAs: 'vm',
    controller: function controller($scope, $timeout, $http) {
      var _this = this;

      var path = 'app/services/';

      this.service = $scope.service;
      this.callService = callService;
      this.fileName = path + 'default.html';

      // Check if file exists
      $scope.$watch('service.type', function () {
        if (!$scope.service.type) {
          return;
        }
        var fileName = path + $scope.service.type + '.html';

        _this.service = $scope.service;
        $http.get(fileName).then(function (result) {
          if (result.data) {
            _this.fileName = fileName;
          }
        });
      });

      function callService(input, isJSON) {
        var _this2 = this;

        var data = isJSON ? angular.fromJSON(input) : input;
        var service = new ROSLIB.Service({
          ros: ros,
          name: this.service.name,
          serviceType: this.service.type
        });
        var request = new ROSLIB.ServiceRequest(data);

        service.callService(request, function (result) {
          $timeout(function () {
            _this2.result = result;
          });
        });
      }
    }
  };
}

angular.module('roscc').directive('ccService', serviceDirective);
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var SettingsController = function () {
  function SettingsController(localStorageService, Settings) {
    _classCallCheck(this, SettingsController);

    this.Settings = Settings;

    this.settings = Settings.getSettings() || [Settings.getDefaultSetting()];
    this.index = Settings.getIndex();

    if (!this.index || this.index > this.settings.length) {
      this.index = '0';
    }
  }

  _createClass(SettingsController, [{
    key: 'save',
    value: function save() {
      this.Settings.save(this.settings, this.index);
    }
  }, {
    key: 'add',
    value: function add() {
      this.settings.push(this.Settings.getDefaultSetting()); // Clone object
      this.index = String(this.settings.length - 1);
      this.save();
    }
  }, {
    key: 'remove',
    value: function remove() {
      this.settings.splice(this.index, 1);
      this.index = '0';

      if (!this.settings.length) {
        this.add();
      }
      this.save();
    }
  }]);

  return SettingsController;
}();

angular.module('roscc').controller('SettingsController', SettingsController);
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var SettingsService = function () {
  function SettingsService($location, localStorageService) {
    _classCallCheck(this, SettingsService);

    this.$location = $location;
    this.localStorageService = localStorageService;
  }

  _createClass(SettingsService, [{
    key: 'load',
    value: function load() {
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
  }, {
    key: 'save',
    value: function save(newSettings, newIndex) {
      this.settings = newSettings;
      this.index = newIndex;
      this.localStorageService.set('selectedSettingIndex', newIndex);
      this.localStorageService.set('settings', newSettings);
    }
  }, {
    key: 'get',
    value: function get() {
      if (!this.setting) {
        this.load();
      }

      return this.setting;
    }
  }, {
    key: 'getIndex',
    value: function getIndex() {
      if (!this.setting) {
        this.load();
      }

      return this.index;
    }
  }, {
    key: 'getSettings',
    value: function getSettings() {
      if (!this.setting) {
        this.load();
      }

      return this.settings;
    }
  }, {
    key: 'getDefaultSetting',
    value: function getDefaultSetting() {
      return {
        name: 'New Setting',
        address: location.hostname,
        port: 9090,
        log: '/rosout',
        imagePreview: { port: 0, quality: 70, width: 640, height: 480 },
        battery: true,
        batteryTopic: '',
        advanced: false
      };
    }
  }]);

  return SettingsService;
}();

angular.module('roscc').service('Settings', SettingsService);
'use strict';

function topicDirective() {
  return {
    scope: { topic: '=' },
    template: '<ng-include src=\"vm.fileName\"></ng-include>',
    controllerAs: 'vm',
    controller: function controller($scope, $timeout, $http, Settings, Quaternions) {
      var _this = this;

      var roslibTopic = new ROSLIB.Topic({
        ros: ros,
        name: $scope.topic.name,
        messageType: $scope.topic.type
      });
      var path = 'app/topics/';

      this.topic = $scope.topic;
      this.toggleSubscription = toggleSubscription;
      this.publishMessage = publishMessage;
      this.isSubscribing = false;
      this.setting = Settings.get();
      this.Quaternions = Quaternions;
      this.fileName = path + 'default.html';

      // Check if file exists
      $scope.$watch('topic.type', function () {
        if (!$scope.topic.type) {
          return;
        }
        var fileName = path + $scope.topic.type + '.html';

        _this.topic = $scope.topic;
        $http.get(fileName).then(function (result) {
          if (result.data) {
            _this.fileName = fileName;
          }
        });
      });

      function toggleSubscription(data) {
        var _this2 = this;

        if (!data) {
          roslibTopic.subscribe(function (message) {
            $timeout(function () {
              _this2.message = message;
            });
          });
        } else {
          roslibTopic.unsubscribe();
        }
        this.isSubscribing = !data;
      }

      function publishMessage(input, isJSON) {
        var data = isJSON ? angular.fromJSON(input) : input;
        var message = new ROSLIB.Message(data);
        roslibTopic.publish(message);
      }
    }
  };
}

angular.module('roscc').directive('ccTopic', topicDirective);