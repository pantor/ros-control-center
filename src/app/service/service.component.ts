import { Component, OnInit, Input } from '@angular/core';

declare var ROSLIB: any;
import 'roslib/build/roslib.js';

import { ros } from '../dashboard/dashboard.component';


@Component({
  selector: 'app-service',
  templateUrl: './service.component.html',
  styleUrls: ['./service.component.css']
})
export class ServiceComponent implements OnInit {
  @Input() service: Service;
  result: any;
  fileName: string;
  input: any;

  constructor() { }

  ngOnInit() {
    const path = 'app/services/';
    this.fileName = `${path}default.html`;
  }

  callService(input: any, isJSON: boolean): void {
    const data = isJSON ? JSON.parse(input) : input;
    const roslibService = new ROSLIB.Service({
      ros,
      name: this.service.name,
      serviceType: this.service.type,
    });
    const request = new ROSLIB.ServiceRequest(data);

    roslibService.callService(request, (result) => {
      this.result = result;
    });
  }
}
