angular.module('roscc', [
    'ngRoute', 
    'ui.bootstrap',
    'LocalStorageModule',
])
    .config(function($routeProvider, localStorageServiceProvider) {
        $routeProvider
            .when('/', { templateUrl: 'views/main.html', controller: 'MainController' })
            .when('/settings', { templateUrl: 'views/settings.html', controller: 'SettingsController' })
            .otherwise({ redirectTo: '/', controller: 'MainController' });
            
        localStorageServiceProvider
            .setPrefix('roscc');
    });