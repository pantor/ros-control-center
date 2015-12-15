function generateDirectiveServiceConfig(file_name, link) {
    link = typeof link !== 'undefined' ? link : null;
    
    return function() {
        return {
            templateUrl: 'directives/service_views/' + file_name + '.html',
            require: '^ccServiceTemplate',
            link: link,
        };
    };
}

function calcRoll(q) {
    return 180 / Math.PI * Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
}

function calcPitch(q) {
    return 180 / Math.PI *  Math.asin(2 * (q.w * q.y - q.z * q.x));
}

function calcYaw(q) {
    return 180 / Math.PI * Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

function generateDirectiveConfig(file_name, link) {
    link = typeof link !== 'undefined' ? link : null;
    return function() {
        return {
            templateUrl: 'directives/topic_views/' + file_name + '.html',
            require: '^ccTopicTemplate',
            link: link,
        };
    };
}


angular.module('roscc')
    .directive('ccDetails', function() {
        return {
            scope: { data: '=', name: '=', advanced: '=' },
            templateUrl: 'directives/details.html',
            controller: function($scope, Domains) {
                $scope.hasNode = function() {
                    return (_.pluck($scope.data.nodes, 'name').indexOf('/' + $scope.name) != -1);
                };

                $scope.filterAdvanced = function(entry, advanced) {
                    return Domains.filterAdvanced(entry, advanced);
                };
                    
                $scope.getDataForDomain = function(array) {
                    return Domains.getDataForDomain(array, $scope.name);
                };
            },
        };
    })



    // --- Parameters --- 

    .directive('ccParameter', function() {
        return {
            scope: { parameter: '=' },
            templateUrl: 'directives/parameter.html',
            controller: function($scope) {
                $scope.p = new ROSLIB.Param({ ros: ros, name: $scope.parameter.name });
                $scope.setValue = function(value) { $scope.p.set(value); };
            },
        };
    })



    // --- Services --- 

    .directive('ccServiceTemplate', function() {
        return {
            scope: { service: '=' },
            templateUrl: 'directives/service.html',
            controller: function($scope, $timeout) {
                $scope.callService = function(data) {
                    var service = new ROSLIB.Service({ ros: ros, name: $scope.service.name, serviceType: $scope.service.type });
                    var request = new ROSLIB.ServiceRequest(data);
                        
                    service.callService(request, function(result) {
                        $timeout(function () {
                            $scope.result = result;
                        });
                    });
                };
                
                $scope.callServiceJSON = function(data) {
                    $scope.callService(JSON.parse(data));
                };
            },
        };
    })

    .directive('ccServiceDefault', generateDirectiveServiceConfig('default'))

    .directive('ccServiceEmpty', generateDirectiveServiceConfig('std_srvs/empty'))
    .directive('ccServiceTrigger', generateDirectiveServiceConfig('std_srvs/trigger'))
    .directive('ccServiceMovingpiBool', generateDirectiveServiceConfig('movingpi/bool'))



    // --- Topics ---

    .directive('ccTopicTemplate', function() {
        return {
            scope: { topic: '=' },
            templateUrl: 'directives/topic.html',
            controller: function($scope, $timeout) {
                var roslibTopic = new ROSLIB.Topic({ ros: ros, name: $scope.topic.name, messageType: $scope.topic.type });
                
                $scope.can_subscribe = true;
                $scope.is_subscribing = false;
                $scope.toggleSubscription = function(data) {
                    if (!data) {
                        roslibTopic.subscribe(function(message) {
                            $timeout(function() {
                                $scope.latest_message = message;
                            });
                        });
                        
                        $scope.is_subscribing = true;
                    } else {
                        roslibTopic.unsubscribe();
                        $scope.is_subscribing = false;
                    }
                };

                $scope.publishMessage = function(data) {
                    var message = new ROSLIB.Message(data);
                    roslibTopic.publish(message);
                };
                    
                $scope.publishMessageJSON = function(data) {
                    $scope.publishMessage(JSON.parse(input));
                };
            },
        };
    })



    .directive('ccTopicDefault', generateDirectiveConfig('default'))

    .directive('ccTopicNumber', generateDirectiveConfig('std_msgs/number'))

    .directive('ccTopicFluidPressure', generateDirectiveConfig('sensor_msgs/fluid-pressure'))
    .directive('ccTopicIlluminance', generateDirectiveConfig('sensor_msgs/illuminance'))
    .directive('ccTopicImage', generateDirectiveConfig('sensor_msgs/image', function(scope) {
        scope.config = config;
        scope.quality = 75;
        scope.$parent.can_subscribe = false;
    }))
    .directive('ccTopicImu', generateDirectiveConfig('sensor_msgs/imu'))
    .directive('ccTopicJoy', generateDirectiveConfig('sensor_msgs/joy'))
    .directive('ccTopicMagneticField', generateDirectiveConfig('sensor_msgs/magnetic-field'))
    .directive('ccTopicRange', generateDirectiveConfig('sensor_msgs/range'))
    .directive('ccTopicRelativeHumidity', generateDirectiveConfig('sensor_msgs/relative-humidity'))
    .directive('ccTopicTemperature', generateDirectiveConfig('sensor_msgs/temperature'))

    .directive('ccTopicPose', generateDirectiveConfig('geometry_msgs/pose', function(scope) {
        function getOrientation() {
            if (scope.$parent.latest_message) {
                return scope.$parent.latest_message.orientation;
            }
            return { w: 1, x: 0, y: 0, z: 0 };
        }
        
        scope.getRoll = function() { return calcRoll( getOrientation() ); };
        scope.getPitch = function() { return calcPitch( getOrientation() ); };
        scope.getYaw = function() { return calcYaw( getOrientation() ); };
    }))
    .directive('ccTopicPoseStamped', generateDirectiveConfig('geometry_msgs/pose-stamped', function(scope) {
        function getOrientation() {
            if (scope.$parent.latest_message) {
                return scope.$parent.latest_message.pose.orientation;
            }
            return { w: 1, x: 0, y: 0, z: 0 };
        }
        
        scope.getRoll = function() { return calcRoll( getOrientation() ); };
        scope.getPitch = function() { return calcPitch( getOrientation() ); };
        scope.getYaw = function() { return calcYaw( getOrientation() ); };
    }))
    .directive('ccTopicPose2d', generateDirectiveConfig('geometry_msgs/pose2D'))

    .directive('ccTopicMovingpiRange', generateDirectiveConfig('movingpi/range'))

    .directive('ccTopicFlypiMotorsThrust', generateDirectiveConfig('flypi/motors-thrust'))
    .directive('ccTopicFlypiSteering', generateDirectiveConfig('flypi/steering'))

    .directive('ccTopicVisionodometryMotion', generateDirectiveConfig('visionodometry/motion', function(scope) {
        function getOrientation() {
            if (scope.$parent.latest_message) {
                return scope.$parent.latest_message.motion.orientation;
            }
            return { w: 1, x: 0, y: 0, z: 0 };
        }
        
        scope.getRoll = function() { return calcRoll( getOrientation() ); };
        scope.getPitch = function() { return calcPitch( getOrientation() ); };
        scope.getYaw = function() { return calcYaw( getOrientation() ); };
    }));
