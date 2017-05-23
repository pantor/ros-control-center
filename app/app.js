function ROSCCConfig($routeProvider, localStorageServiceProvider) {
  $routeProvider
    .when('/', {
      templateUrl: 'app/control/control.html',
      controller: 'ControlController',
      controllerAs: 'ctrl',
    })
    .when('/settings', {
      templateUrl: 'app/settings/settings.html',
      controller: 'SettingsController',
      controllerAs: 'ctrl',
    })
    .otherwise({ redirectTo: '/' });

  localStorageServiceProvider
    .setPrefix('roscc');
}

angular.module('roscc', ['ngRoute', 'ui.bootstrap', 'LocalStorageModule']).config(ROSCCConfig);
