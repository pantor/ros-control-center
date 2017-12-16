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
  roslibService: any;
  result: any;
  input: any;

  constructor() { }

  ngOnInit() {
    this.roslibService = new ROSLIB.Service({
      ros,
      name: this.service.name,
      serviceType: this.service.type,
    });
  }

  callService(input: any, isJSON: boolean): void {
    const data = isJSON ? JSON.parse(input) : input;
    const request = new ROSLIB.ServiceRequest(data);

    this.roslibService.callService(request, result => {
      this.result = result;
    });
  }
}
