import { Component, OnInit, Input } from '@angular/core';

declare var ROSLIB: any;
import 'roslib/build/roslib.js';

import { ros } from '../dashboard/dashboard.component';


@Component({
  selector: 'app-parameter',
  templateUrl: './parameter.component.html',
  styleUrls: ['./parameter.component.css']
})
export class ParameterComponent implements OnInit {
  @Input() parameter: Parameter;
  private roslibParam: any;

  ngOnInit() {
    this.roslibParam = new ROSLIB.Param({ ros, name: this.parameter.name });
  }

  setValue(): void {
    this.roslibParam.set(this.parameter.value);
  }
}
