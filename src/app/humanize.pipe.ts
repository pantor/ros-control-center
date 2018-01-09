import { Pipe, PipeTransform } from '@angular/core';

@Pipe({
  name: 'humanize'
})
export class HumanizePipe implements PipeTransform {
  transform(value: any, args?: any): any {
    return value.replace(/_/g, ' ').replace(/\//g, ' ').trim().split(' ').map(e => e.charAt(0).toUpperCase() + e.slice(1)).join(' ');
  }
}
