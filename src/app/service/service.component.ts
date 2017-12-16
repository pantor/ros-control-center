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
  response: any;

  constructor() { }

  ngOnInit() {
    this.roslibService = new ROSLIB.Service({
      ros,
      name: this.service.name,
      serviceType: this.service.type,
    });
  }

  callService(data: any): void {
    const request = new ROSLIB.ServiceRequest(data);
    this.roslibService.callService(request, response => {
      this.response = response;
    });
  }
}
