function serviceDirective(fileName) {
  return function () {
    return {
      scope: { service: '=' },
      templateUrl: 'app/services/' + fileName + '.html',
      controllerAs: 'vm',
      controller: function ($scope, $timeout) {
        var vm = this;
        vm.service = $scope.service;
        vm.callService = callService;
        vm.callServiceJSON = callServiceJSON;

        function callService(data) {
          var service = new ROSLIB.Service({
            ros: ros,
            name: vm.service.name,
            serviceType: vm.service.type,
          });
          var request = new ROSLIB.ServiceRequest(data);

          service.callService(request, function (result) {
            $timeout(function () {
              vm.result = result;
            });
          });
        }

        function callServiceJSON(data) {
          callService(angular.fromJSON(data));
        }
      },
    };
  };
}


angular.module('roscc')
  .directive('ccServiceDefault', serviceDirective('default'))

  .directive('ccServiceEmpty', serviceDirective('std_srvs/empty'))
  .directive('ccServiceTrigger', serviceDirective('std_srvs/trigger'))
  .directive('ccServiceSetBool', serviceDirective('std_srvs/set-bool'))

  .directive('ccServiceMovingpiBool', serviceDirective('movingpi/bool'));
