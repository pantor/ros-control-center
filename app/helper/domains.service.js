class DomainsService {
  static filterAdvanced(entry, advanced) {
    const entryArray = entry.split('/');
    if (advanced) {
      return true;
    }
    if (!entry || _.isEmpty(entryArray)) {
      return false;
    }
    return (_.last(entryArray)[0] === _.last(entryArray)[0].toUpperCase());
  }

  static getDomains(array) {
    const result = [];
    angular.forEach(array, (entry) => {
      const nameArray = entry.name.split('/');
      if (nameArray.length > 1) {
        result.push(nameArray[1]);
      }
    });
    return _.uniq(result).sort();
  }

  static getGlobalParameters(array) {
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

  static getDataForDomain(array, domainName, advanced) {
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
