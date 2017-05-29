class SettingsController {
  constructor(localStorageService, Settings) {
    this.Settings = Settings;

    this.settings = Settings.getSettings() || [Settings.getDefaultSetting()];
    this.index = Settings.getIndex();

    if (!this.index || this.index > this.settings.length) {
      this.index = '0';
    }

    this.save(); // Save current setting again (if it's the first time)
  }

  save() {
    this.Settings.save(this.settings, this.index);
  }

  add() {
    this.settings.push(this.Settings.getDefaultSetting()); // Clone object
    this.index = String(this.settings.length - 1);
    this.save();
  }

  remove() {
    this.settings.splice(this.index, 1);
    this.index = '0';

    if (!this.settings.length) {
      this.add();
    }
    this.save();
  }
}

angular.module('roscc').component('ccSettings', {
  templateUrl: 'app/settings/settings.html',
  controller: SettingsController,
});
