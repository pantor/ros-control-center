function serviceDirective() {
  return {
    scope: { service: '=' },
    template: '<ng-include src="ctrl.fileName"></ng-include>',
    controllerAs: 'ctrl',
    controller($scope, $timeout, $http) {
      const path = 'app/services/';

      this.service = $scope.service;
      this.callService = callService;
      this.fileName = `${path}default.html`;

      // Check if file exists
      $scope.$watch('service.type', () => {
        if (!$scope.service.type) {
          return;
        }
        const fileName = `${path}${$scope.service.type}.html`;

        this.service = $scope.service;
        $http.get(fileName).then((result) => {
          if (result.data) {
            this.fileName = fileName;
          }
        }, () => {
          // console.log(fileName + ' not found, use default service component.');
        });
      });

      function callService(input, isJSON) {
        const data = isJSON ? angular.fromJSON(input) : input;
        const service = new ROSLIB.Service({
          ros,
          name: this.service.name,
          serviceType: this.service.type,
        });
        const request = new ROSLIB.ServiceRequest(data);

        service.callService(request, (result) => {
          $timeout(() => {
            this.result = result;
          });
        });
      }
    },
  };
}

angular.module('roscc').directive('ccService', serviceDirective);
