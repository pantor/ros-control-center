import { Component, OnInit, Input } from '@angular/core';

declare var ROSLIB: any;
import '../../assets/roslib.js';

import { ros } from '../dashboard/dashboard.component';


@Component({
  selector: 'app-parameter',
  templateUrl: './parameter.component.html',
  styleUrls: ['./parameter.component.css']
})
export class ParameterComponent implements OnInit {
  @Input() parameter: any;
  param: any;

  constructor() { }

  ngOnInit() {
    this.param = new ROSLIB.Param({ ros, name: this.parameter.name });
  }

  setValue(value): void {
    this.param.set(value);
  }
}
