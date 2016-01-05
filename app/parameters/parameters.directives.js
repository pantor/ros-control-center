function ParamaterDirective() {
  return {
    scope: { parameter: '=' },
    templateUrl: 'app/parameters/parameters.html',
    controllerAs: 'vm',
    controller: function ($scope) {
      var vm = this;
      var p = new ROSLIB.Param({ ros: ros, name: $scope.parameter.name });

      vm.parameter = $scope.parameter;
      vm.setValue = setValue;

      function setValue(value) {
        p.set(value);
      }
    },
  };
}

angular.module('roscc').directive('ccParameter', ParamaterDirective);
