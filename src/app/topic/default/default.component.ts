import { Component, EventEmitter, Input, Output } from '@angular/core';

@Component({
  selector: 'app-topic-default',
  templateUrl: './default.component.html'
})
export class DefaultTopicComponent {
  @Input() isSubscribing = false;
  @Output() publish = new EventEmitter<any>();
  private message: any;

  publishMessage(message) {
    this.publish.emit(JSON.parse(message));
  }

  @Input()
  set onNewMessage(message: any) {
    this.message = JSON.stringify(message);
  }
}
