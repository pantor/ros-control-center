class SettingsService {
  constructor($location, localStorageService) {
    this.$location = $location;
    this.localStorageService = localStorageService;
  }

  load() {
    this.index = this.localStorageService.get('selectedSettingIndex');
    this.settings = this.localStorageService.get('settings');
    if (this.settings && this.index) {
      this.setting = this.settings[this.index];
    }

    // If there are no saved settings, redirect to /settings for first setting input
    if (!this.setting) {
      this.$location.path('/settings').replace();
    }
  }

  save(newSettings, newIndex) {
    this.settings = newSettings;
    this.index = newIndex;
    this.localStorageService.set('selectedSettingIndex', newIndex);
    this.localStorageService.set('settings', newSettings);
  }

  get() {
    if (!this.setting) {
      this.load();
    }

    return this.setting;
  }

  getIndex() {
    if (!this.setting) {
      this.load();
    }

    return this.index;
  }

  getSettings() {
    if (!this.setting) {
      this.load();
    }

    return this.settings;
  }

  getDefaultSetting() {
    return {
      name: 'Robot Name',
      address: '127.0.0.1', // use localhost
      port: 9090, // default port of rosbridge_server
      log: '/rosout',
      imagePreview: { port: 0, quality: 70, width: 640, height: 480 },
      battery: false,
      batteryTopic: '',
      advanced: false,
    };
  }
}

angular.module('roscc').service('Settings', SettingsService);
