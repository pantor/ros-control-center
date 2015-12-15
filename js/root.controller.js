var ros;

// Root controller of the connection
angular.module('roscc')
    .controller('RootController', function($scope, $location, $interval, $timeout, localStorageService) {
        $scope.selectedSettingIndex = localStorageService.get('selectedSettingIndex');
        $scope.settings = JSON.parse(localStorageService.get('settings'));
        $scope.config = $scope.settings[$scope.selectedSettingIndex];
        
        if (!$scope.config) {
            $location.path('/settings').replace();
        }
        
        $scope.isConnected = false;
        
        $scope.getPath = function() {
            return $location.path();
        };
        
        $scope.newRosConnection = function() {
            if (ros) {
                ros.close(); // Close old connection
            }
            
            ros = new ROSLIB.Ros({url: 'ws://' + $scope.config.address + ':' + $scope.config.port});
            
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
        
        // Load ROS connection and keep trying if it fails
        if (!$scope.isConnected) {
            $scope.newRosConnection();
        }
        $interval(function() {
            if (!$scope.isConnected) {
                $scope.newRosConnection();
            }
        }, 1000); // [ms]
    });