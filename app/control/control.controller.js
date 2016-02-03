let ros;
let isConnected = false;

class ControlController {
  constructor($timeout, $interval, Settings, Domains) {
    this.$timeout = $timeout;
    this.Domains = Domains;

    this.isConnected = isConnected;
    this.setting = Settings.get();
    this.maxConsoleEntries = 200;

    // Load ROS connection and keep trying if it fails
    this.newRosConnection();
    $interval(() => {
      this.newRosConnection();
    }, 1000); // [ms]

    this.resetData();
    if (isConnected) {
      this.onConnected();
    }
  }

  // The active domain shows further information in the center view
  setActiveDomain(domain) {
    this.activeDomain = domain;
  }

  getDomains() {
    const allData = this.data.topics.concat(this.data.services, this.data.nodes);
    const domains = this.Domains.getDomains(allData);

    if (!this.activeDomain) {
      this.setActiveDomain(domains[0]);
    }
    return domains;
  }

  getGlobalParameters() {
    return this.Domains.getGlobalParameters(this.data.parameters);
  }

  resetData() {
    this.data = {
      rosout: [],
      topics: [],
      nodes: [],
      parameters: [],
      services: [],
    };
  }

  newRosConnection() {
    if (isConnected || !this.setting) {
      return;
    }

    if (ros) {
      ros.close(); // Close old connection
      ros = false;
      return;
    }

    ros = new ROSLIB.Ros({ url: `ws://${this.settings.address}:${this.settings.port}` });

    ros.on('connection', () => {
      this.onConnected();
      isConnected = true;
      this.isConnected = isConnected;
    });

    ros.on('error', () => {
      isConnected = false;
      this.isConnected = isConnected;
    });

    ros.on('close', () => {
      isConnected = false;
      this.isConnected = isConnected;
    });
  }

  onConnected() {
    // wait a moment until ROS is loaded and initialized
    this.$timeout(() => {
      this.loadData();

      this.setConsole();
      if (this.setting.battery) {
        this.setBattery();
      }
    }, 1000); // [ms]
  }

  // Setup of console (in the right sidebar)
  setConsole() {
    const consoleTopic = new ROSLIB.Topic({
      ros,
      name: this.setting.log,
      messageType: 'rosgraph_msgs/Log',
    });
    consoleTopic.subscribe(message => {
      const nameArray = message.name.split('/');
      const d = new Date(message.header.stamp.secs * 1E3 + message.header.stamp.nsecs * 1E-6);

      message.abbr = (nameArray.length > 1) ? nameArray[1] : message.name;

      // String formatting of message time and date
      function addZero(i) { return i < 10 ? `0${i}` : `${i}`; }
      message.dateString = `${addZero(d.getHours())}:
      ${addZero(d.getMinutes())}:
      ${addZero(d.getSeconds())}.
      ${addZero(d.getMilliseconds())}`;
      this.data.rosout.unshift(message);

      if (this.data.rosout.length > this.maxConsoleEntries) {
        this.data.rosout.pop();
      }
    });
  }

  // Setup battery status
  setBattery() {
    const batteryTopic = new ROSLIB.Topic({
      ros,
      name: this.setting.batteryTopic,
      messageType: 'std_msgs/Float32',
    });
    batteryTopic.subscribe(message => {
      this.batteryStatus = message.data;
    });
  }

  // Load structure, all data, parameters, topics, services, nodes...
  loadData() {
    this.resetData();

    ros.getTopics(topics => {
      angular.forEach(topics, name => {
        this.data.topics.push({ name });

        ros.getTopicType(name, type => {
          _.findWhere(this.data.topics, { name }).type = type;
        });
      });
    });

    ros.getServices(services => {
      angular.forEach(services, name => {
        this.data.services.push({ name });

        ros.getServiceType(name, type => {
          _.findWhere(this.data.services, { name }).type = type;
        });
      });
    });

    ros.getParams(params => {
      angular.forEach(params, name => {
        const param = new ROSLIB.Param({ ros, name });
        this.data.parameters.push({ name });

        param.get((value) => {
          _.findWhere(this.data.parameters, { name }).value = value;
        });
      });
    });

    ros.getNodes(nodes => {
      angular.forEach(nodes, name => {
        this.data.nodes.push({ name });
      });
    });
  }
}

angular.module('roscc').controller('ControlController', ControlController);
