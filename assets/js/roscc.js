'use strict';

function ROSCCConfig($routeProvider, localStorageServiceProvider) {
  $routeProvider.when('/', { template: '<cc-control></cc-control>' }).when('/settings', { template: '<cc-settings></cc-settings>' }).otherwise({ redirectTo: '/' });

  localStorageServiceProvider.setPrefix('roscc');
}

angular.module('roscc', ['ngRoute', 'ui.bootstrap', 'LocalStorageModule']).config(ROSCCConfig);
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var ros = void 0;
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
    key: 'hasFilteredDomains',
    value: function hasFilteredDomains(advanced) {
      var _this2 = this;

      return _.some(_.map(this.getDomains(), function (dom) {
        return _this2.Domains.filterAdvanced(dom, advanced);
      }));
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
      var _this3 = this;

      if (isConnected || this.setting === angular.isUndefined) {
        return;
      }

      if (ros) {
        ros.close(); // Close old connection
        ros = false;
        return;
      }

      ros = new ROSLIB.Ros({ url: 'ws://' + this.setting.address + ':' + this.setting.port });

      ros.on('connection', function () {
        _this3.onConnected();
        isConnected = true;
        _this3.isConnected = isConnected;
      });

      ros.on('error', function () {
        isConnected = false;
        _this3.isConnected = isConnected;
      });

      ros.on('close', function () {
        isConnected = false;
        _this3.isConnected = isConnected;
      });
    }
  }, {
    key: 'onConnected',
    value: function onConnected() {
      var _this4 = this;

      // wait a moment until ROS is loaded and initialized
      this.$timeout(function () {
        _this4.loadData();

        _this4.setConsole();
        if (_this4.setting.battery) {
          _this4.setBattery();
        }
      }, 1000); // [ms]
    }

    // Setup of console (in the right sidebar)

  }, {
    key: 'setConsole',
    value: function setConsole() {
      var _this5 = this;

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
          return i < 10 ? '0' + i : '' + i;
        }
        message.dateString = addZero(d.getHours()) + ':\n      ' + addZero(d.getMinutes()) + ':\n      ' + addZero(d.getSeconds()) + '.\n      ' + addZero(d.getMilliseconds());
        _this5.data.rosout.unshift(message);

        if (_this5.data.rosout.length > _this5.maxConsoleEntries) {
          _this5.data.rosout.pop();
        }
      });
    }

    // Setup battery status

  }, {
    key: 'setBattery',
    value: function setBattery() {
      var _this6 = this;

      var batteryTopic = new ROSLIB.Topic({
        ros: ros,
        name: this.setting.batteryTopic,
        messageType: 'std_msgs/Float32'
      });
      batteryTopic.subscribe(function (message) {
        _this6.batteryStatus = message.data;
      });
    }

    // Load structure, all data, parameters, topics, services, nodes...

  }, {
    key: 'loadData',
    value: function loadData() {
      var _this7 = this;

      this.resetData();

      ros.getTopics(function (topics) {
        angular.forEach(topics, function (name) {
          _this7.data.topics.push({ name: name });

          ros.getTopicType(name, function (type) {
            _.findWhere(_this7.data.topics, { name: name }).type = type;
          });
        });
      });

      ros.getServices(function (services) {
        angular.forEach(services, function (name) {
          _this7.data.services.push({ name: name });

          ros.getServiceType(name, function (type) {
            _.findWhere(_this7.data.services, { name: name }).type = type;
          });
        });
      });

      ros.getParams(function (params) {
        angular.forEach(params, function (name) {
          var param = new ROSLIB.Param({ ros: ros, name: name });
          _this7.data.parameters.push({ name: name });

          param.get(function (value) {
            _.findWhere(_this7.data.parameters, { name: name }).value = value;
          });
        });
      });

      ros.getNodes(function (nodes) {
        angular.forEach(nodes, function (name) {
          _this7.data.nodes.push({ name: name });
        });
      });
    }
  }]);

  return ControlController;
}();

angular.module('roscc').component('ccControl', {
  templateUrl: 'app/control/control.html',
  controller: ControlController
});
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

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var NavbarController = function () {
  function NavbarController($location) {
    _classCallCheck(this, NavbarController);

    this.$location = $location;
  }

  _createClass(NavbarController, [{
    key: 'isPath',
    value: function isPath(path) {
      return this.$location.path() === path;
    }
  }]);

  return NavbarController;
}();

angular.module('roscc').component('ccNavbar', {
  templateUrl: 'app/navbar/navbar.html',
  controller: NavbarController
});
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var ParameterController = function () {
  function ParameterController() {
    _classCallCheck(this, ParameterController);

    this.param = new ROSLIB.Param({ ros: ros, name: this.parameter.name });
  }

  _createClass(ParameterController, [{
    key: 'setValue',
    value: function setValue(value) {
      this.param.set(value);
    }
  }]);

  return ParameterController;
}();

angular.module('roscc').component('ccParameter', {
  bindings: { parameter: '=' },
  templateUrl: 'app/parameters/parameters.html',
  controller: ParameterController
});
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var ServiceController = function () {
  function ServiceController($scope, $http) {
    var _this = this;

    _classCallCheck(this, ServiceController);

    var path = 'app/services/';
    this.fileName = path + 'default.html';

    // Check if file exists
    $scope.$watch('service.type', function () {
      if (!_this.service.type) {
        return;
      }
      var fileName = '' + path + _this.service.type + '.html';
      $http.get(fileName).then(function (result) {
        if (result.data) {
          _this.fileName = fileName;
        }
      });
    });
  }

  _createClass(ServiceController, [{
    key: 'callService',
    value: function callService(input, isJSON) {
      var _this2 = this;

      var data = isJSON ? angular.fromJson(input) : input;
      var ROSservice = new ROSLIB.Service({
        ros: ros,
        name: this.service.name,
        serviceType: this.service.type
      });
      var request = new ROSLIB.ServiceRequest(data);

      ROSservice.callService(request, function (result) {
        _this2.result = result;
      });
    }
  }]);

  return ServiceController;
}();

angular.module('roscc').component('ccService', {
  bindings: { service: '=' },
  template: '<ng-include src="$ctrl.fileName"></ng-include>',
  controller: ServiceController
});
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

angular.module('roscc').component('ccSettings', {
  templateUrl: 'app/settings/settings.html',
  controller: SettingsController
});
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
  }]);

  return SettingsService;
}();

angular.module('roscc').service('Settings', SettingsService);
'use strict';

var _createClass = function () { function defineProperties(target, props) { for (var i = 0; i < props.length; i++) { var descriptor = props[i]; descriptor.enumerable = descriptor.enumerable || false; descriptor.configurable = true; if ("value" in descriptor) descriptor.writable = true; Object.defineProperty(target, descriptor.key, descriptor); } } return function (Constructor, protoProps, staticProps) { if (protoProps) defineProperties(Constructor.prototype, protoProps); if (staticProps) defineProperties(Constructor, staticProps); return Constructor; }; }();

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

var TopicController = function () {
  function TopicController($scope, $http, Settings, Quaternions) {
    var _this = this;

    _classCallCheck(this, TopicController);

    this.roslibTopic = new ROSLIB.Topic({
      ros: ros,
      name: this.topic.name,
      messageType: this.topic.type
    });

    this.isSubscribing = false;
    this.setting = Settings.get();
    this.Quaternions = Quaternions;

    var path = 'app/topics/';
    this.fileName = path + 'default.html';

    // Check if file exists
    $scope.$watch('topic.type', function () {
      if (!_this.topic.type) {
        return;
      }
      var fileName = '' + path + _this.topic.type + '.html';
      $http.get(fileName).then(function (result) {
        if (result.data) {
          _this.fileName = fileName;
        }
      });
    });
  }

  _createClass(TopicController, [{
    key: 'toggleSubscription',
    value: function toggleSubscription(data) {
      var _this2 = this;

      if (!data) {
        this.roslibTopic.subscribe(function (message) {
          _this2.message = message;
        });
      } else {
        this.roslibTopic.unsubscribe();
      }
      this.isSubscribing = !data;
    }
  }, {
    key: 'publishMessage',
    value: function publishMessage(input, isJSON) {
      var data = isJSON ? angular.fromJson(input) : input;
      var message = new ROSLIB.Message(data);
      this.roslibTopic.publish(message);
    }
  }]);

  return TopicController;
}();

angular.module('roscc').component('ccTopic', {
  bindings: { topic: '=' },
  template: '<ng-include src="$ctrl.fileName"></ng-include>',
  controller: TopicController
});