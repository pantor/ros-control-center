angular.module('roscc')
    .controller('MainController', function($scope, $timeout, Domains) {
        $scope.data = {
            rosout: [],
            topics: [],
            nodes: [],
            parameters: [],
            services: [],
        };
      
        // The active domain shows further information in the center view
        $scope.activeDomain = '';
        $scope.setActiveDomain = function(domain) {
            $scope.activeDomain = domain;
        };
      
        $scope.filterAdvanced = function(entry, advanced) {
            return Domains.filterAdvanced(entry, advanced);
        };

        $scope.getDomains = function(advanced) {    
            var all_data = $scope.data.topics.concat($scope.data.services, $scope.data.nodes);
            var domains = Domains.getDomains(all_data);
        
            // Set active domain
            if (!$scope.activeDomain) {
                $scope.activeDomain = domains[0];
            }

            return domains;
        };

        $scope.getGlobalParameters = function(advanced) {
            return Domains.getGlobalParameters($scope.data.parameters);
        };
      
        // Setup of console (in the right sidebar)
        var max_length = 200;
        var setConsole = function() {
            var topic_rosout = new ROSLIB.Topic({ ros: ros, name: config.log, messageType: 'rosgraph_msgs/Log' });
            topic_rosout.subscribe(function(message) {
                $timeout(function() {
                    var split = message.name.split('/');
                    message.abbr = message.name;
                    if (split.length > 1) {
                        message.abbr = split[1];
                    }
                    
                    // String formatting of message time and date
                    function z(i) { return i < 10 ? "0" + i : i; }
                    var d = new Date(message.header.stamp.secs * 1000 + message.header.stamp.nsecs / 1000000);
                    message.date_string = z(d.getHours()) + ":" + z(d.getMinutes()) + ":" + z(d.getSeconds()) + "." + z(d.getMilliseconds());
                    $scope.data.rosout.unshift(message);
                    
                    if ($scope.data.rosout.length > max_length) {
                        $scope.data.rosout.pop();
                    }
                });
            });
        };
      
        function setBattery() {
            var topic_rosout = new ROSLIB.Topic({ ros: ros, name: config.battery_topic, messageType: 'std_msgs/Float32' });
            topic_rosout.subscribe(function(message) {
                $timeout(function() {
                    $scope.battery_status = message.data;
                });
            });
        }
      
        // Load structure all data, parameters, topics, services, nodes...
        function loadData() {
            ros.getTopics(function(topics) {
                $timeout(function() {
                    $scope.data.topics = [];
                    topics.forEach(function(topic) {
                        $scope.data.topics.push({ name: topic });

                        ros.getTopicType(topic, function(type) {
                            $timeout(function() {
                                _.findWhere($scope.data.topics, { name: topic }).type = type;
                            });
                        });
                    });
                });
            });

            ros.getServices(function(services) {
                $timeout(function() {
                    $scope.data.services = [];
                    services.forEach(function(service) {
                        $scope.data.services.push({ name: service });

                        ros.getServiceType(service, function(type) {
                            $timeout(function() {
                                _.findWhere($scope.data.services, { name: service }).type = type;
                            });
                        });
                    });
                });
            });

            ros.getParams(function(params) {
                $timeout(function() {
                    $scope.data.parameters = [];
                    params.forEach(function(param) {
                        $scope.data.parameters.push({ name: param });

                        var p = new ROSLIB.Param({ ros: ros, name: param });
                        p.get(function(value) {
                            $timeout(function() {
                                _.findWhere($scope.data.parameters, { name: param }).value = value;
                            });
                        });
                    });
                });
            });

            ros.getNodes(function(nodes) {
                $timeout(function() {
                    $scope.data.nodes = [];
                    nodes.forEach(function(e) {
                        $scope.data.nodes.push({name: e});
                    });
                });
            });
        }
      
        $scope.$on('CONNECTED', function() {
            $timeout(function() {
                $scope.data = {
                    rosout: [],
                    topics: [],
                    nodes: [],
                    parameters: [],
                    services: [],
                };
                
                setConsole();
                if (config.battery) {
                    setBattery();
                }
                loadData();
            }, 500);
        });
    });
