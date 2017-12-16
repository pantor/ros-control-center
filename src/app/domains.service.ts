import { Injectable } from '@angular/core';

import * as _ from 'underscore';

@Injectable()
export class DomainsService {

  constructor() { }

  filterAdvanced(entry: string, advanced: boolean) {
    if (advanced) {
      return true;
    }

    const entryArray = entry.split('/');
    if (!entry || _.isEmpty(entryArray)) {
      return false;
    }

    // Don't show the default nodes, params, topics and services
    return (!_.contains([
      'rosapi',
      'rosbridge_websocket',
      'rosout',
      'rosout_agg',
      'rosversion',
      'run_id',
      'rosdistro',
      'get_loggers',
      'set_logger_level',
    ], _.last(entryArray)));
  }

  getDomains(array: any[]): Domain[] {
    const result = [];
    for (let entry of array) {
      const nameArray = entry.name.split('/');
      if (nameArray.length > 1) {
        result.push(nameArray[1]);
      }
    };
    return _.uniq(result).sort();
  }

  getGlobalParameters(array: any[]): Parameter[] {
    const result = [];
    for (let entry of array) {
      const nameArray = entry.name.split('/');
      if (nameArray.length === 2) {
        entry.abbr = _.last(nameArray);
        result.push(entry);
      }
    };
    return result;
  }

  getDataForDomain(array: any[], domain: Domain, advanced: boolean) {
    const result = [];
    for (let entry of array) {
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
