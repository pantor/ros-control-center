import { Component, ViewChild, OnInit, Input } from '@angular/core';

declare var ROSLIB: any;
import 'roslib/build/roslib.js';

import { ros } from '../dashboard/dashboard.component';
import { TypeComponent } from '../type/type.component';


@Component({
  selector: 'app-service',
  templateUrl: './service.component.html',
  styleUrls: ['./service.component.css']
})
export class ServiceComponent implements OnInit {
  @ViewChild(TypeComponent) child: TypeComponent;

  @Input() service: Service;
  private roslibService: any;
  public input = {};
  public response: any;

  ngOnInit() {
    this.roslibService = new ROSLIB.Service({
      ros,
      name: this.service.name,
      serviceType: this.service.type,
    });
  }

  callService(): void {
    const request = new ROSLIB.ServiceRequest(this.child.data);
    this.roslibService.callService(request, response => {
      this.response = response;
    });
  }
}
