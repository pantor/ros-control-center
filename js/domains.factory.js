// Filter advanced topics, services, parameters by checking the beginning capital letter
angular.module('roscc')
    .factory('Domains', function() {
        return {
            filterAdvanced: function(entry, advanced) {
                if (advanced) {
                    return true;
                }

                var split = entry.split('/');
                if (_.isEmpty(split)) {
                    return false;
                }

                return (_.last(split)[0] == _.last(split)[0].toUpperCase());
            },
            getDomains: function(array) {
                var result = _.filter(array, function(entry) {
                    return (entry.name.split('/').length > 1);
                });
                return _.uniq(result).sort();
            },
            getGlobalParameters: function(array) {
                return _.filter(array, function(entry) {
                    var split = entry.name.split('/');
                    entry.abbr = _.last(split);
                    return (split.length == 2);
                });
            },
            getDataForDomain: function(array, domainName) {
                return _.filter(array, function(entry) {
                    var split = entry.name.split('/');
                    entry.abbr = split.slice(2).join(' ');
                    return (split.length > 1 && split[1] == domainName);
                });
            },
        };
    });