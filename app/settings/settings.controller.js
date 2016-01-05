function SettingsController(localStorageService, Settings) {
  var vm = this;
  vm.add = add;
  vm.remove = remove;
  vm.save = save;
  vm.settings = Settings.getSettings() || [Settings.getDefaultSetting()];
  vm.index = Settings.getIndex();


  if (!vm.index || vm.index > vm.settings.length) {
    vm.index = '0';
  }

  function save() {
    Settings.save(vm.settings, vm.index);
  }

  function add() {
    vm.settings.push(Settings.getDefaultSetting()); // Clone object
    vm.index = String(vm.settings.length - 1);
    save();
  }

  function remove() {
    vm.settings.splice(vm.index, 1);
    vm.index = '0';

    if (!vm.settings.length) {
      vm.add();
    }
    save();
  }
}

angular.module('roscc').controller('SettingsController', SettingsController);
