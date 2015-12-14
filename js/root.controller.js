// Root controller of the connection
angular.module('roscc')
    .controller('RootController', function($scope, $timeout, $interval, $location) {
        $scope.isConnected = false;
        $scope.config = config;
      
        $scope.getPath = function() {
            return $location.path();
        };
      
        function newRosConnection() {
            if (ros) {
                ros.close(); // Close old connection
            }
            ros = new ROSLIB.Ros({ url: 'ws://' + config.address + ':' + config.port });
        
            ros.on('connection', function() {
                console.log('Connected');
                $timeout(function() {
                    $scope.$broadcast('CONNECTED');
                    $scope.isConnected = true;
                });
            });

            ros.on('error', function() {
                if ($scope.isConnected) {
                    console.log('Error');
                }
                $timeout(function() {
                    $scope.isConnected = false;
                });
            });

            ros.on('close', function() {
                if ($scope.isConnected) {
                    console.log('Close');
                }
                $timeout(function() {
                    $scope.isConnected = false;
                });
            });
        }
      
        // Load ROS connection and keep trying if it fails
        newRosConnection();
        $interval(function() {
            if (!$scope.isConnected) {
                newRosConnection();
            }
        }, 1000); // [ms]
    });