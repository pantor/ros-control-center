import { Component, OnInit } from '@angular/core';
import { Observable } from 'rxjs/Observable';

declare var ROSLIB: any;
import 'roslib/build/roslib.js';
import 'rxjs/add/operator/map';
import 'rxjs/add/operator/mergeMap';
import 'rxjs/add/observable/forkJoin';

import { Setting } from '../settings/setting';


export let ros;
let isConnected = false;


@Component({
  selector: 'app-dashboard',
  templateUrl: './dashboard.component.html',
  styleUrls: ['./dashboard.component.css'],
  providers: []
})
export class DashboardComponent implements OnInit {
  data: {
    rosout: any[],
    nodes: Node[],
    globalParameters: Parameter[],
  };
  activeNode: Node;
  isConnected: boolean;
  setting: Setting;
  maxConsoleEntries = 200;
  batteryStatus: any;
  ready = false;
  hasNodes = true;

  ngOnInit() {
    this.isConnected = isConnected;
    this.setting = Setting.getCurrent();

    // Load ROS connection and keep trying if it fails
    this.newRosConnection();
    setInterval(() => {
      this.newRosConnection();
    }, 1000); // [ms]

    this.data = {
      rosout: [],
      nodes: [],
      globalParameters: [],
    };

    if (isConnected) {
      this.onConnected();
    }
  }

  // The active node shows further information in the center view
  setActiveNode(node: Node): void {
    this.activeNode = node;
  }

  newRosConnection(): void {
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

  onConnected(): void {
    this.data = {
      rosout: [],
      nodes: [],
      globalParameters: [],
    };

    this.loadData();

    this.setConsole();
    if (this.setting.battery) {
      this.setBattery();
    }
  }

  // Setup of console (in the right sidebar)
  setConsole(): void {
    const consoleTopic = new ROSLIB.Topic({
      ros,
      name: this.setting.log,
      messageType: 'rosgraph_msgs/Log',
    });
    consoleTopic.subscribe(message => {
      if (!this.setting.advanced && !this.filterString(message.name)) {
        return;
      }

      const nameArray = message.name.split('/');
      const d = new Date((message.header.stamp.secs * 1E3) + (message.header.stamp.nsecs * 1E-6));

      message.name = (nameArray.length > 1) ? nameArray[1] : message.name;

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
  setBattery(): void {
    const batteryTopic = new ROSLIB.Topic({
      ros,
      name: this.setting.batteryTopic,
      messageType: 'std_msgs/Float32',
    });
    batteryTopic.subscribe(message => {
      this.batteryStatus = message.data;
    });
  }


  parseTypedef(typedef: any): Type {
    const info = typedef.map(inf => ({
      type: inf.type,
      members: inf.examples.map((e, i) => ({
        example: inf.examples[i],
        name: inf.fieldnames[i],
        length: inf.fieldarraylen[i],
        type: inf.fieldtypes[i],
      })),
    }));

    function findMembers(obj) {
      for (const e of obj.members) {
        if (info.find(i => e.type === i.type)) {
          e.members = info.find(i => e.type === i.type).members;
        }
        if (e.members) {
          findMembers(e);
        }
      }
    }

    findMembers(info[0]);
    return info[0];
  }


  getNodes(): Observable<Node[]> {
    return Observable.create(obs => {
      ros.getNodes(data => {
        obs.next(data);
        obs.complete();
      });
    }).flatMap(data => {
      return Observable.forkJoin(data.map(name => Observable.create(obs => {
        const detailClient = new ROSLIB.Service({
          ros: ros,
          name: '/rosapi/node_details',
          serviceType: 'rosapi/NodeDetails'
        });
        const request = new ROSLIB.ServiceRequest({ node: name });
        detailClient.callService(request, data2 => {
          obs.next({ name, publishing: data2.publishing, subscribing: data2.subscribing, services: data2.services });
          obs.complete();
        });
      })));
    });
  }

  getTopics(): Observable<Topic[]> {
    return Observable.create(obs => {
      ros.getTopics(data => {
        obs.next(data);
        obs.complete();
      });
    }).flatMap(data => {
      return Observable.forkJoin(data.topics.map((name, i) => Observable.create(obs => {
        ros.getMessageDetails(data.types[i], typedef => {
          obs.next({ name, type: data.types[i], info: this.parseTypedef(typedef) });
          obs.complete();
        }, error => {
          obs.next({ name, type: data.types[i] });
          obs.complete();
        });
      })));
    });
  }

  getServices(): Observable<Service[]> {
    return Observable.create(obs => {
      ros.getServices(data => {
        obs.next(data);
        obs.complete();
      });
    }).flatMap(data => {
      return Observable.forkJoin(data.map(name => Observable.create(obs => {
        ros.getServiceType(name, type => {
          obs.next({ name, type });
          obs.complete();
        });
      })));
    }).flatMap(data => {
      return Observable.forkJoin(data.map(data2 => Observable.create(obs => {
        const detailClient = new ROSLIB.Service({
          ros: ros,
          name: '/rosapi/service_request_details',
          serviceType: 'rosapi/ServiceRequestDetails'
        });
        const request = new ROSLIB.ServiceRequest({ type: data2.type });
        detailClient.callService(request, data3 => {
          obs.next({ ...data2, request_info: this.parseTypedef(data3.typedefs) });
          obs.complete();
        });
      })));
    }).flatMap(data => {
      return Observable.forkJoin(data.map(data2 => Observable.create(obs => {
        const detailClient = new ROSLIB.Service({
          ros: ros,
          name: '/rosapi/service_response_details',
          serviceType: 'rosapi/ServiceResponseDetails'
        });
        const request = new ROSLIB.ServiceRequest({ type: data2.type });
        detailClient.callService(request, data3 => {
          obs.next({ ...data2, response_info: this.parseTypedef(data3.typedefs) });
          obs.complete();
        });
      })));
    })
  }

  getParams(): Observable<Parameter[]> {
    return Observable.create(obs => {
      ros.getParams(data => {
        obs.next(data);
        obs.complete();
      });
    }).flatMap(data => {
      return Observable.forkJoin(data.map(name => Observable.create(obs => {
        const param = new ROSLIB.Param({ ros, name });
        const nameArray = name.split('/');
        param.get(value => {
          if (nameArray.length > 2) {
            obs.next({ node: '/' + nameArray[1], name, value });
          } else {
            obs.next({ name, value });
          }
          obs.complete();
        });
      })));
    });
  }

  filterString(s: any): boolean {
    if (s.name) {
      s = s.name;
    }

    if ([
      '/rosapi',
      '/rosout',
      '/rosbridge_websocket',
      '/rosversion',
      '/run_id',
      '/rosdistro',
    ].includes(s)) {
      return false;
    }

    const nameArray = s.split('/');
    if ([
      'set_logger_level',
      'get_loggers',
    ].includes(nameArray[nameArray.length - 1])) {
      return false;
    }

    return true;
  }

  // Load structure, all data, parameters, topics, services, nodes...
  loadData(): void {
    Observable.forkJoin(this.getNodes(), this.getTopics(), this.getServices(), this.getParams())
    .subscribe(([nodes, topics, services, params]) => {

      if (!this.setting.advanced) {
        nodes = nodes.filter(this.filterString);
        params = params.filter(this.filterString);

        for (const n of nodes) {
          n.services = n.services.filter(this.filterString);
          n.publishing = n.publishing.filter(this.filterString);
          n.subscribing = n.subscribing.filter(this.filterString);
        }
      }

      this.data.nodes = nodes;
      this.data.globalParameters = params.filter(param => !param.node);

      for (const n of this.data.nodes) {
        n.services = n.services.map(name => services.find(e => e.name === name));
        n.topics = n.publishing.concat(n.subscribing).map(name => topics.find(e => e.name === name));
        n.params = params.filter(param => param.node === n.name);
      }

      this.setActiveNode(this.data.nodes[0]);
      this.ready = true;
    });
  }
}
