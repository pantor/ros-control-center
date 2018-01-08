import { Component, OnInit, Input } from '@angular/core';

@Component({
  selector: 'app-type',
  templateUrl: './type.component.html',
  styleUrls: ['./type.component.css']
})
export class TypeComponent implements OnInit {
  @Input() info: { name: string, members: any[] };
  @Input() data: any;
  @Input() readonly: boolean;
  @Input() level = 0;
  horizontal = false;

  ngOnInit() {
    this.horizontal = this.info && this.info.members && !this.info.members.some(e => e.members);
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
    if (numbers.includes(primitiveType)) return 'number';
    return primitiveType;
  }
}
