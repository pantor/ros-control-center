// Filter advanced topics, services, parameters by checking the beginning capital letter
angular.module('roscc')
    .factory('Domains', function() {
        var msgs = [];
        return {
            filterAdvanced: function(entry, advanced) {
                if (advanced) {
                    return true;
                }

                var split = entry.split('/');
                if (split.length < 1) {
                    return false;
                }

                if (split[split.length - 1][0] == split[split.length - 1][0].toUpperCase()) {
                    return true;
                }

                return false;
            },
            getDomains: function(array) {
                var result = [];
                array.forEach(function(entry) {
                    var split = entry.name.split('/');
                    if (split.length > 1) {
                        result.push(split[1]);
                    }
                });
                return _.uniq(result).sort();
            },
            getGlobalParameters: function(array) {
                var result = [];
                array.forEach(function(e) {
                    var split = e.name.split('/');
                    if (split.length == 2) {
                        e.abbr = split[split.length - 1];
                        result.push(e);
                    }
                });
                return result;
            },
            getDataForDomain: function(array, domainName) {
                var result = [];
                array.forEach(function(entry) {
                    var split = entry.name.split('/');
                    if (split.length > 1) {
                        if (split[1] == domainName) {
                            entry.abbr = split.slice(2).join(' ');
                            result.push(entry);
                        }
                    }
                });
                return result;
            }
        };
    });