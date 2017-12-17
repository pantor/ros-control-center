import { Component, EventEmitter, Input, Output } from '@angular/core';

@Component({
  selector: 'app-service-default',
  templateUrl: './default.component.html'
})
export class ServiceDefaultComponent {
  @Output() call = new EventEmitter<any>();
  private response: any;
  private input: string;

  callService() {
    if (this.input) {
      this.call.emit(JSON.parse(this.input));
    } else {
      this.call.emit({});
    }
  }

  @Input()
  set onResponse(response: any) {
    this.response = JSON.stringify(response);
  }
}
