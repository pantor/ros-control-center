var ros;

// Root controller of the connection
angular.module('roscc')
    .controller('RootController', function($scope, $location, $timeout, $interval) {
        $scope.config = config;
        $scope.isConnected = false;
      
        $scope.getPath = function() {
            return $location.path();
        };
        
        $scope.newRosConnection = function() {
            if (ros) {
                ros.close(); // Close old connection
            }
            
            ros = new ROSLIB.Ros({url: 'ws://' + config.address + ':' + config.port});
            
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
                if ($scope.is_connected) {
                    console.log('Close');
                }
                $timeout(function() {
                    $scope.isConnected = false;
                });
            });
        };
    });