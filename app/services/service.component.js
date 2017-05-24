class ServiceController {
  constructor($scope, $http) {
    const path = 'app/services/';
    this.fileName = `${path}default.html`;

    // Check if file exists
    $scope.$watch('service.type', () => {
      if (!this.service.type) {
        return;
      }
      const fileName = `${path}${this.service.type}.html`;
      $http.get(fileName).then((result) => {
        if (result.data) {
          this.fileName = fileName;
        }
      });
    });
  }

  callService(input, isJSON) {
    const data = isJSON ? angular.fromJson(input) : input;
    const ROSservice = new ROSLIB.Service({
      ros,
      name: this.service.name,
      serviceType: this.service.type,
    });
    const request = new ROSLIB.ServiceRequest(data);

    ROSservice.callService(request, (result) => {
      this.result = result;
    });
  }
}

angular.module('roscc').component('ccService', {
  bindings: { service: '=' },
  template: '<ng-include src="$ctrl.fileName"></ng-include>',
  controller: ServiceController,
});
