angular.module('roscc')
    .controller('SettingsController', function($scope, localStorageService) {
        $scope.add = function() {
            $scope.settings.push( JSON.parse(JSON.stringify(defaultSetting)) ); // Clone object
            $scope.selectedSettingIndex = String($scope.settings.length - 1);
        };
        
        $scope.remove = function() {
            $scope.settings.splice($scope.selectedSettingIndex, 1);
            $scope.selectedSettingIndex = '0';
            
            if (!$scope.settings.length) {
                $scope.add();
            }
        };
        
        $scope.selectedSettingIndex = localStorageService.get('selectedSettingIndex');
        $scope.settings = JSON.parse(localStorageService.get('settings'));
        
        var defaultSetting = {
            name: 'New Setting',
            address: location.hostname,
            port: 9090,
            log: '/rosout',
            imagePreview: { port: 0, quality: 70, width: 640, height: 480 },
            battery: true,
            batteryTopic: '',
        };
        
        if (!$scope.settings) {
            $scope.settings = [defaultSetting];
        }
        
        if (!$scope.selectedSettingIndex || $scope.selectedSettingIndex > $scope.settings.length) {
            $scope.selectedSettingIndex = '0';
        }
        
        $scope.$watch('selectedSettingIndex', function() {
            localStorageService.set('selectedSettingIndex', $scope.selectedSettingIndex);
        });
        
        $scope.$watch('settings', function() {
            localStorageService.set('settings', JSON.stringify($scope.settings));
        }, true);
    });
