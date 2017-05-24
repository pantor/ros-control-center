function ROSCCConfig($routeProvider, localStorageServiceProvider) {
  $routeProvider
    .when('/', { template: '<cc-control></cc-control>' })
    .when('/settings', { template: '<cc-settings></cc-settings>' })
    .otherwise({ redirectTo: '/' });

  localStorageServiceProvider
    .setPrefix('roscc');
}

angular.module('roscc', ['ngRoute', 'ui.bootstrap', 'LocalStorageModule']).config(ROSCCConfig);
