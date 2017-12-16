export class Setting {
  name: string;
  address: string;
  port: number;
  log: string;
  imagePreview: { port: number, quality: number, width: number, height: number };
  battery: boolean;
  batteryTopic: string;
  advanced: boolean;

  static getDefault(): Setting {
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

  static getCurrent(): Setting {
    const settings = JSON.parse(localStorage.getItem('roscc2-settings')) || [ Setting.getDefault() ];
    const index = JSON.parse(localStorage.getItem('roscc2-index')) || 0;

    return settings[index];
  }
}
