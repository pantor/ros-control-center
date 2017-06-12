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
  @Input() topic: any;
  roslibTopic: any;
  fileName: string;
  message: any;
  isSubscribing: boolean;

  constructor() { }

  ngOnInit() {
    this.roslibTopic = new ROSLIB.Topic({
      ros,
      name: this.topic.name,
      messageType: this.topic.type,
    });

    const path = 'app/topics/';
    this.fileName = `${path}default.html`;
  }


  toggleSubscription(data): void {
    if (!data) {
      this.roslibTopic.subscribe((message) => {
        this.message = message;
      });
    } else {
      this.roslibTopic.unsubscribe();
    }
    this.isSubscribing = !data;
  }

  publishMessage(input, isJSON: boolean): void {
    const data = isJSON ? JSON.stringify(input) : input;
    const message = new ROSLIB.Message(data);
    this.roslibTopic.publish(message);
  }
}
