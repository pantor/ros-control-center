// Filter advanced topics, services, parameters by checking the beginning capital letter
angular.module('roscc')
    .factory('Config', function(localStorageService) {
        var config;
        
        return {
            load: function() {
                var selectedSettingIndex = localStorageService.get('selectedSettingIndex');
                var settings = JSON.parse(localStorageService.get('settings'));
                config = settings[selectedSettingIndex];
            },
            get: function() {
                return config;
            },
        };
    });