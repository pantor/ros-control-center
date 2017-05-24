class ServiceController {
  constructor($scope, $http) {
    this.$scope = $scope;
    this.$http = $http;
  }

  $onInit() {
    const path = 'app/services/';
    this.fileName = `${path}default.html`;

    // Check if file exists
    this.$scope.$watch('service.type', () => {
      if (!this.service.type) {
        return;
      }
      const fileName = `${path}${this.service.type}.html`;
      this.$http.get(fileName).then((result) => {
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
