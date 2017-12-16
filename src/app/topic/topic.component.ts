import { Component, OnInit, Input } from '@angular/core';

declare var ROSLIB: any;
import '../../assets/roslib.js';

import { ros } from '../dashboard/dashboard.component';


@Component({
  selector: 'app-topic',
  templateUrl: './topic.component.html',
  styleUrls: ['./topic.component.css']
})
export class TopicComponent implements OnInit {
  @Input() topic: Topic;
  roslibTopic: any;
  message: any;
  isSubscribing: boolean = false;

  constructor() { }

  ngOnInit() {
    this.roslibTopic = new ROSLIB.Topic({
      ros,
      name: this.topic.name,
      messageType: this.topic.type,
    });
  }

  toggleSubscription(isSubscribing: boolean): void {
    if (!isSubscribing) {
      this.roslibTopic.subscribe((message) => {
        this.message = message;
      });
    } else {
      this.roslibTopic.unsubscribe();
    }
    this.isSubscribing = isSubscribing;
  }

  publish(data: any): void {
    const data2 = data.isJSON ? JSON.stringify(data.message) : data.message;
    const message = new ROSLIB.Message(data2);
    this.roslibTopic.publish(message);
  }
}
