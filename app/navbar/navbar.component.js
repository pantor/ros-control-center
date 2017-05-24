class NavbarController {
  constructor($location) {
    this.$location = $location;
  }

  isPath(path) {
    return this.$location.path() === path;
  }
}

angular.module('roscc').component('ccNavbar', {
  templateUrl: 'app/navbar/navbar.html',
  controller: NavbarController,
});
