import { Component, EventEmitter, Input, Output } from '@angular/core';

@Component({
  selector: 'app-topic-default',
  templateUrl: './default.component.html'
})
export class DefaultComponent {
  @Input() isSubscribing = false;
  @Output() publish = new EventEmitter<{ message: any, isJSON?: boolean }>();
  private message: any;

  publishMessage(message) {
    this.publish.emit(message);
  }

  @Input()
  set onNewMessage(onNewMessage: any) {
    this.message = JSON.stringify(onNewMessage);
  }
}
