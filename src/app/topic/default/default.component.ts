import { Component, OnInit, EventEmitter, Input, Output } from '@angular/core';

@Component({
  selector: 'app-topic-default',
  templateUrl: './default.component.html'
})
export class DefaultComponent implements OnInit {
  @Input() isSubscribing: boolean = false;
  @Output() publish = new EventEmitter<{ message: any, isJSON: boolean }>();
  private message: any;

  constructor() { }

  ngOnInit() { }

  publishMessage(message, isJSON) {
    this.publish.emit({ message: message, isJSON: isJSON });
  }

  @Input()
  set onNewMessage(onNewMessage: string) {
    this.message = onNewMessage;
  }
}
