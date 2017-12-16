import { Component, OnInit } from '@angular/core';
import { Observable } from 'rxjs/Observable';

declare var ROSLIB: any;
import 'roslib/build/roslib.js';
import 'rxjs/add/operator/map';
import 'rxjs/add/observable/forkJoin';

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
  data: {
    rosout: any[],
    topics: Topic[],
    nodes: Node[],
    parameters: Parameter[],
    services: Service[],
  };
  domains: Domain[];
  filteredDomains: Domain[];
  globalParameters: Parameter[];
  filteredGlobalParameters: Parameter[];
  activeDomain: Domain;
  isConnected: boolean;
  setting: Setting;
  maxConsoleEntries: number;
  batteryStatus: any;
  ready = false;

  constructor(private domainsService: DomainsService) {
    this.isConnected = isConnected;
    this.setting = Setting.getCurrent();
    this.maxConsoleEntries = 200;

    // Load ROS connection and keep trying if it fails
    this.newRosConnection();
    setInterval(() => {
      this.newRosConnection();
    }, 1000); // [ms]

    this.data = {
      rosout: [],
      topics: [],
      nodes: [],
      parameters: [],
      services: [],
    };

    if (isConnected) {
      this.onConnected();
    }
  }

  ngOnInit() { }

  // The active domain shows further information in the center view
  setActiveDomain(domain: Domain): void {
    this.activeDomain = domain;
  }

  getDomains(): Domain[] {
    return this.setting.advanced ? this.domains : this.filteredDomains;
  }

  hasDomains(): boolean {
    return (this.getDomains() && this.getDomains().length > 0);
  }

  getGlobalParameters(): Parameter[] {
    return this.setting.advanced ? this.globalParameters : this.filteredGlobalParameters;
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
  setConsole(): void {
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
  setBattery(): void {
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
    const nodeObservable: Observable<Node[]> = Observable.create(observer => {
      ros.getNodes(data => {
        observer.next(data);
        observer.complete();
      });
    })
    .map(data => data.map(name => ({ name })));

    const topicObservable: Observable<Topic[]> = Observable.create(observer => {
      ros.getTopics(data => {
        observer.next(data);
        observer.complete();
      });
    })
    .map(data => data.topics.map((name, i) => ({ name, type: data.types[i], abbr: '' })));

    const serviceObservable: Observable<Service[]> = Observable.create(observer => {
      ros.getServices(data => {
        const obs = data.map(name => Observable.create(observer2 => {
          ros.getServiceType(name, type => {
            observer2.next({ name, type, 'abbr': '' });
            observer2.complete();
          });
        }));
        Observable.forkJoin(...obs).subscribe(data2 => {
          observer.next(data2);
          observer.complete();
        });
      });
    });

    const paramObservable: Observable<Parameter[]> = Observable.create(observer => {
      ros.getParams(data => {
        const obs = data.map(name => Observable.create(observer2 => {
          const param = new ROSLIB.Param({ ros, name });
          param.get(value => {
            observer2.next({ name, value, abbr: '' });
            observer2.complete();
          });
        }));
        Observable.forkJoin(...obs).subscribe(data2 => {
          observer.next(data2);
          observer.complete();
        });
      });
    });

    Observable.forkJoin(nodeObservable, topicObservable, serviceObservable, paramObservable)
    .subscribe(([nodes, topics, services, params]) => {
      this.data = {
        rosout: [] as any[],
        nodes: nodes,
        topics: topics,
        services: services,
        parameters: params,
      }

      const allData: any[] = this.data.topics.concat(this.data.services, this.data.nodes as any[]);
      this.domains = this.domainsService.getDomains(allData);
      this.filteredDomains = this.domains.filter(dom => this.domainsService.filterAdvanced(dom));
      this.globalParameters = this.domainsService.getGlobalParameters(this.data.parameters);
      this.filteredGlobalParameters = this.globalParameters.filter(param => this.domainsService.filterAdvanced(param.name));

      const domains = this.setting.advanced ? this.domains : this.filteredDomains;
      this.setActiveDomain(domains[0]);
      this.ready = true;
    });
  }
}
