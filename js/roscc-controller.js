var app = angular.module('roscc', ['ngRoute', 'ui.bootstrap']); // Dependencies
var ros = new ROSLIB.Ros({url: 'ws://' + config.address + ':' + config.port});; // Main ros element for ROSLIB

// Routing
app.config(['$routeProvider', function($routeProvider) {
  $routeProvider
    .when('/', { templateUrl: 'views/control.html', controller: 'control-ctrl'})
    .when('/info', { templateUrl: 'views/info.html', controller: 'info-ctrl'})
    .otherwise({ redirectTo: '/', controller: 'control-ctrl'});
}]);


// Main controller of the connection
app.controller('main-ctrl', function($scope, $timeout, $interval, $location) {
  $scope.is_connected = false;
  $scope.config = config;
  
  $scope.getPath = function() {
    return $location.path();
  };
  
  ros.on('connection', function() {
    console.log('Connected');
    $timeout(function() {
      $scope.is_connected = true;
    });
  });

  ros.on('error', function() {
    if ($scope.is_connected)
      console.log('Error');
    $timeout(function() {
      $scope.is_connected = false;
    });
  });

  ros.on('close', function() {
    if ($scope.is_connected)
      console.log('Close');
    $timeout(function() {
      $scope.is_connected = false;
    });
  });
  
  /* var newRosConnection = function() {
    if (!$scope.is_connected) {
      ros = new ROSLIB.Ros({url: 'ws://' + config.address + ':' + config.port});
      
      ros.on('connection', function() {
        console.log('Connected');
        $timeout(function() {
          $scope.is_connected = true;
        });
      });

      ros.on('error', function() {
        if ($scope.is_connected)
          console.log('Error');
        $timeout(function() {
          $scope.is_connected = false;
        });
      });

      ros.on('close', function() {
        if ($scope.is_connected)
          console.log('Close');
        $timeout(function() {
          $scope.is_connected = false;
        });
      });
    }
  };
  
  // Load ROS connection and keep trying if it fails
  newRosConnection();
  $interval(function() {
    newRosConnection();
  }, 1000); // [ms] */
});

// Info controller
app.controller('info-ctrl', function($scope) {
  $scope.config = config;
});

// Control controller
app.controller('control-ctrl', function($scope, $timeout, DomainHelper) {
  $scope.data = {
    rosout: [],
    topics: [],
    nodes: [],
    parameters: [],
    services: []
  };
  
  // The active domain shows further information in the center view
  $scope.activeDomain = '';
  $scope.setActiveDomain = function(domain) {
    $scope.activeDomain = domain;
  };

  // Setup of console (in the right sidebar)
  var max_length = 100;
  var topic_rosout = new ROSLIB.Topic({ros: ros, name: config.log, messageType: 'rosgraph_msgs/Log'});
  topic_rosout.subscribe(function(message) {
    $timeout(function() {
      var split = message.name.split('/');
      message.abbr = message.name;
      if (split.length > 1)
        message.abbr = split[1];
      
      // String formatting of message time and date
      function z(i) { return i < 10 ? "0" + i : i; }
      var d = new Date(message.header.stamp.secs * 1000 + message.header.stamp.nsecs / 1000000);
      message.date_string = z(d.getHours()) + ":" + z(d.getMinutes()) + ":" + z(d.getSeconds()) + "." + z(d.getMilliseconds());
      $scope.data.rosout.unshift(message);
      
      if ($scope.data.rosout.length > max_length)
        $scope.data.rosout.pop();
    });
  });
  
  $scope.filterAdvanced = function(e, a) {
    return DomainHelper.filterAdvanced(e, a);
  };

  $scope.getDomains = function(advanced) {
    var all_data = $scope.data.topics.concat($scope.data.services, $scope.data.nodes);
    var domains = DomainHelper.getDomains(all_data);
    
    // Set active domain
    if (!$scope.activeDomain)
      $scope.activeDomain = domains[0];

    return domains;
  };

  $scope.getGlobalParameters = function(advanced) {
    return DomainHelper.getGlobalParameters($scope.data.parameters);
  };
  
  // Load structure all data, parameters, topics, services, nodes...
  $scope.loadData = function() {
    ros.getTopics(function(topics) {
      $timeout(function() {
        $scope.data.topics = [];
        topics.forEach(function(topic) {
          $scope.data.topics.push({name: topic});

          ros.getTopicType(topic, function(type) {
            $timeout(function() {
              _.findWhere($scope.data.topics, {name: topic}).type = type;
            });
          });
        });
      });
    });

    ros.getServices(function(services) {
      $timeout(function() {
        $scope.data.services = [];
        services.forEach(function(service) {
          $scope.data.services.push({name: service});

          ros.getServiceType(service, function(type) {
            $timeout(function() {
              _.findWhere($scope.data.services, {name: service}).type = type;
            });
          });
        });
      });
    });

    ros.getParams(function(params) {
      $timeout(function() {
        $scope.data.parameters = [];
        params.forEach(function(param) {
          $scope.data.parameters.push({name: param});

          var p = new ROSLIB.Param({ros: ros, name: param});
          p.get(function(value) {
            $timeout(function() {
              _.findWhere($scope.data.parameters, {name: param}).value = value;
            });
          });
        });
      });
    });

    ros.getNodes(function(nodes) {
      $timeout(function() {
        $scope.data.nodes = [];
        nodes.forEach(function(e) {
          $scope.data.nodes.push({name: e});
        });
      });
    });
  };

  $scope.loadData();
});
