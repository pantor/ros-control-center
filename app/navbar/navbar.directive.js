angular.module('roscc').component('ccNavbar', {
  templateUrl: 'app/navbar/navbar.html',
  controller($location) {
    function isPath(path) {
      return $location.path() === path;
    }

    this.isPath = isPath;
  },
});
