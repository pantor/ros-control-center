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
