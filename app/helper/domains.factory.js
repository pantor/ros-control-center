function DomainsFactory() {
  return {
    filterAdvanced: function (entry, advanced) {
      var entryArray = entry.split('/');

      if (advanced) {
        return true;
      }

      if (!entry || _.isEmpty(entryArray)) {
        return false;
      }

      return (_.last(entryArray)[0] === _.last(entryArray)[0].toUpperCase());
    },
    getDomains: function (array) {
      var result = [];
      angular.forEach(array, function (entry) {
        var nameArray = entry.name.split('/');
        if (nameArray.length > 1) {
          result.push(nameArray[1]);
        }
      });
      return _.uniq(result).sort();
    },
    getGlobalParameters: function (array) {
      var result = [];
      angular.forEach(array, function (entry) {
        var nameArray = entry.name.split('/');
        if (nameArray.length === 2) {
          entry.abbr = _.last(nameArray);
          result.push(entry);
        }
      });
      return result;
    },
    getDataForDomain: function (array, domainName) {
      var result = [];
      angular.forEach(array, function (entry) {
        var nameArray = entry.name.split('/');
        if (nameArray.length > 1 && nameArray[1] === domainName) {
          entry.abbr = nameArray.slice(2).join(' ');
          result.push(entry);
        }
      });
      return result;
    },
  };
}

// Filter advanced topics, services, parameters by checking the beginning capital letter
angular.module('roscc').factory('Domains', DomainsFactory);
