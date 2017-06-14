import { Component, OnInit } from '@angular/core';

declare var ROSLIB: any;
import '../../assets/roslib.js';
import * as _ from 'underscore';

import { DomainsService } from '../domains.service';
import { Setting } from '../setting';


export let ros;
let isConnected = false;


@Component({
  selector: 'app-dashboard',
  templateUrl: './dashboard.component.html',
  styleUrls: ['./dashboard.component.css'],
  providers: [DomainsService]
})
export class DashboardComponent implements OnInit {
  data: any;
  activeDomain: string;
  isConnected: boolean;
  setting: Setting;
  maxConsoleEntries: number;
  batteryStatus: any;

  constructor(private domainsService: DomainsService) {
    this.isConnected = isConnected;
    this.setting = Setting.getCurrent();
    this.maxConsoleEntries = 200;

    // Load ROS connection and keep trying if it fails
    this.newRosConnection();
    setInterval(() => {
      this.newRosConnection();
    }, 1000); // [ms]

    this.resetData();
    if (isConnected) {
      this.onConnected();
    }
  }

  ngOnInit() { }

  // The active domain shows further information in the center view
  setActiveDomain(domain) {
    this.activeDomain = domain;
  }

  getDomains() {
    const allData = this.data.topics.concat(this.data.services, this.data.nodes);
    const domains = this.domainsService.getDomains(allData);

    if (!this.activeDomain) {
      this.setActiveDomain(domains[0]);
    }
    return domains;
  }

  hasFilteredDomains(advanced) {
    return _.some(_.map(this.getDomains(), dom => this.domainsService.filterAdvanced(dom, advanced)));
  }

  getGlobalParameters() {
    return this.domainsService.getGlobalParameters(this.data.parameters);
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
    if (isConnected) {
      return;
    }

    if (ros) {
      ros.close(); // Close old connection
      ros = false;
      return;
    }

    ros = new ROSLIB.Ros({ url: `ws://${this.setting.address}:${this.setting.port}` });

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
    setTimeout(() => {
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
    consoleTopic.subscribe((message) => {
      const nameArray = message.name.split('/');
      const d = new Date((message.header.stamp.secs * 1E3) + (message.header.stamp.nsecs * 1E-6));

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
    batteryTopic.subscribe((message) => {
      this.batteryStatus = message.data;
    });
  }

  // Load structure, all data, parameters, topics, services, nodes...
  loadData(): void {
    this.resetData();

    ros.getTopics((topics) => { // Topics now has topics and types arrays
      for (let name of topics.topics) {
        this.data.topics.push({ name });

        ros.getTopicType(name, (type) => {
          (_.findWhere(this.data.topics, { name }) as any).type = type;
        });
      };
    });

    ros.getServices((services) => {
      for (let name of services) {
        this.data.services.push({ name });

        ros.getServiceType(name, (type) => {
          (_.findWhere(this.data.services, { name }) as any).type = type;
        });
      }
    });

    ros.getParams((params) => {
      for (let name of params) {
        const param = new ROSLIB.Param({ ros, name });
        this.data.parameters.push({ name });

        param.get((value) => {
          (_.findWhere(this.data.parameters, { name }) as any).value = value;
        });
      };
    });

    ros.getNodes((nodes) => {
      for (let name of nodes) {
        this.data.nodes.push({ name });
      };
    });
  }
}
