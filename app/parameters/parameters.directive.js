function ParamaterDirective() {
  return {
    scope: { parameter: '=' },
    templateUrl: 'app/parameters/parameters.html',
    controllerAs: 'vm',
    controller($scope) {
      const param = new ROSLIB.Param({ ros, name: $scope.parameter.name });

      this.parameter = $scope.parameter;
      this.setValue = setValue;

      function setValue(value) {
        param.set(value);
      }
    },
  };
}

angular.module('roscc').directive('ccParameter', ParamaterDirective);
