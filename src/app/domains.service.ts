import { Injectable } from '@angular/core';

@Injectable()
export class DomainsService {

  constructor() { }

  filterAdvanced(entry: string, advanced = false) {
    if (advanced) {
      return true;
    }

    const entryArray = entry.split('/');
    if (!entry || !entryArray || entryArray.length === 0) {
      return false;
    }

    // Don't show the default nodes, params, topics and services
    return [
      'rosapi',
      'rosbridge_websocket',
      'rosout',
      'rosout_agg',
      'rosversion',
      'run_id',
      'rosdistro',
      'get_loggers',
      'set_logger_level',
    ].indexOf(entryArray[entryArray.length - 1]) < 0;
  }

  getDomains(array: any[]): Domain[] {
    return array
      .filter(entry => entry.name.split('/').length > 1)
      .map(entry => entry.name.split('/')[1])
      .filter((v, i, a) => a.indexOf(v) === i)
      .sort();
  }

  getGlobalParameters(array: Parameter[]): Parameter[] {
    const result: Parameter[] = [];
    for (const entry of array) {
      const nameArray = entry.name.split('/');
      if (nameArray.length === 2) {
        entry.abbr = nameArray.pop();
        result.push(entry);
      }
    };
    return result;
  }

  getDataForDomain(array: any[], domain: Domain, advanced: boolean) {
    const result = [];
    for (const entry of array) {
      const nameArray = entry.name.split('/');
      if (
        nameArray.length > 1 &&
        nameArray[1] === domain &&
        this.filterAdvanced(entry.name, advanced)
      ) {
        entry.abbr = nameArray.slice(2).join(' ');
        result.push(entry);
      }
    };
    return result;
  }
}
