class DomainsService {
  filterAdvanced(entry, advanced) {
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

  getDomains(array) {
    const result = [];
    angular.forEach(array, (entry) => {
      const nameArray = entry.name.split('/');
      if (nameArray.length > 1) {
        result.push(nameArray[1]);
      }
    });
    return _.uniq(result).sort();
  }

  getGlobalParameters(array) {
    const result = [];
    angular.forEach(array, (entry) => {
      const nameArray = entry.name.split('/');
      if (nameArray.length === 2) {
        entry.abbr = _.last(nameArray);
        result.push(entry);
      }
    });
    return result;
  }

  getDataForDomain(array, domainName, advanced) {
    const result = [];
    angular.forEach(array, (entry) => {
      const nameArray = entry.name.split('/');
      if (
        nameArray.length > 1 &&
        nameArray[1] === domainName &&
        this.filterAdvanced(entry.name, advanced)
      ) {
        entry.abbr = nameArray.slice(2).join(' ');
        result.push(entry);
      }
    });
    return result;
  }
}

// Filter advanced topics, services, parameters by checking the beginning capital letter
angular.module('roscc').service('Domains', DomainsService);
