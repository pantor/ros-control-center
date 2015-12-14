var ros; // Main ros element for ROSLIB

angular.module('roscc', [
    'ngRoute', 
    'ui.bootstrap',
])
    .config(['$routeProvider', function($routeProvider) {
        $routeProvider
            .when('/', { templateUrl: 'views/main.html', controller: 'MainController' })
            .when('/info', { templateUrl: 'views/info.html', controller: 'InfoController' })
            .otherwise({ redirectTo: '/', controller: 'MainController' });
    }]);