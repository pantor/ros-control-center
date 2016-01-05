function NavbarDirective($location) {
  return {
    templateUrl: 'app/navbar/navbar.html',
    controllerAs: 'vm',
    controller: function () {
      var vm = this;
      vm.isPath = isPath;

      function isPath(path) {
        return $location.path() === path;
      }
    },
  };
}

angular.module('roscc').directive('ccNavbar', NavbarDirective);
