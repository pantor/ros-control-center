import { Component, EventEmitter, Input, Output } from '@angular/core';

@Component({
  selector: 'app-service-std-srvs-trigger',
  templateUrl: './Trigger.component.html'
})
export class ServiceStdSrvsTriggerComponent {
  @Output() call = new EventEmitter<any>();
  private response: any;

  callService() {
    this.call.emit({});
  }

  @Input()
  set onResponse(response: any) {
    this.response = response;
  }
}
