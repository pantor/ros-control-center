function SettingsFactory($location, localStorageService) {
  var setting;
  var settings;
  var index;

  return {
    load: function () {
      index = localStorageService.get('selectedSettingIndex');
      settings = localStorageService.get('settings');
      if (settings && index) {
        setting = settings[index];
      }

      // If there are no saved settings, redirect to /settings for first setting input
      if (!setting) {
        $location.path('/settings').replace();
      }
    },
    save: function (newSettings, newIndex) {
      settings = newSettings;
      index = newIndex;
      localStorageService.set('selectedSettingIndex', newIndex);
      localStorageService.set('settings', newSettings);
    },
    get: function () {
      if (!setting) {
        this.load();
      }

      return setting;
    },
    getIndex: function () {
      if (!setting) {
        this.load();
      }

      return index;
    },
    getSettings: function () {
      if (!setting) {
        this.load();
      }

      return settings;
    },
    getDefaultSetting: function () {
      return {
        name: 'New Setting',
        address: location.hostname,
        port: 9090,
        log: '/rosout',
        imagePreview: { port: 0, quality: 70, width: 640, height: 480 },
        battery: true,
        batteryTopic: '',
        advanced: false,
      };
    },
  };
}

angular.module('roscc').factory('Settings', SettingsFactory);
