import { Component, OnInit, Input, Output, EventEmitter } from '@angular/core';

@Component({
  selector: 'app-type',
  templateUrl: './type.component.html',
  styleUrls: ['./type.component.css']
})
export class TypeComponent implements OnInit {
  @Output() childEvent = new EventEmitter();
  @Input() info: Type;
  @Input() data: any;
  @Input() readonly: boolean;
  @Input() level = 0;
  public horizontal = false;

  ngOnInit() {
    this.horizontal = this.info && this.info.members && !this.info.members.some(e => typeof e.members !== 'undefined');
  }

  getLooseType(primitiveType: string): string {
    const numbers = [
      'int8',
      'uint8',
      'int16',
      'uint16',
      'int32',
      'uint32',
      'int64',
      'uint64',
      'float32',
      'float64',
    ];
    if (numbers.includes(primitiveType)) {
      return 'number';
    }
    return primitiveType;
  }

  update(object: { names: string[], value: any }) {
    if (this.level > 0) {
      object.names.unshift(this.info.name);
      this.childEvent.emit(object);
    } else {
      if (this.data === undefined) {
        this.data = {};
      }
      let dataElement = this.data;
      for (const n of object.names.slice(0, -1)) {
        if (dataElement[n] === undefined) {
          dataElement[n] = {};
        }
        dataElement = dataElement[n];
      }
      dataElement[object.names[object.names.length - 1]] = object.value;
    }
  }
}
