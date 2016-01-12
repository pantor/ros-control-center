function ROSCCConfig($routeProvider, localStorageServiceProvider) {
  $routeProvider
    .when('/', {
      templateUrl: 'app/control/control.html',
      controller: 'ControlController',
      controllerAs: 'vm',
    })
    .when('/settings', {
      templateUrl: 'app/settings/settings.html',
      controller: 'SettingsController',
      controllerAs: 'vm',
    })
    .otherwise({ redirectTo: '/' });

  localStorageServiceProvider
    .setPrefix('roscc');
}

angular.module('roscc', ['ngRoute', 'ui.bootstrap', 'LocalStorageModule']).config(ROSCCConfig);
