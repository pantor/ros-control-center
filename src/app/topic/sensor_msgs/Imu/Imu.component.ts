import { Component, EventEmitter, Input, Output } from '@angular/core';

@Component({
  selector: 'app-topic-sensor-msgs-imu',
  templateUrl: './Imu.component.html'
})
export class TopicSensorMsgsImuComponent {
  @Input() isSubscribing = false;
  @Output() publish = new EventEmitter<any>();
  private message: { data: number };

  publishMessage(message: any) {
    this.publish.emit(message);
  }

  @Input()
  set onNewMessage(message: any) {
    this.message = message;
  }
}
