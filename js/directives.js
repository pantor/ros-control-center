var app = angular.module('roscc');

app.directive('ccInfo', function() {
	return {
		scope: { data: '=', name: '=', advanced: '=' },
		templateUrl: 'directives/info.html',
    controller: function($scope, DomainHelper) {
      $scope.hasNode = function() {
        return _.pluck($scope.data.nodes, 'name').indexOf('/' + $scope.name) != -1;
      };

      $scope.filterAdvanced = function(entry, advanced) {
				return DomainHelper.filterAdvanced(entry, advanced);
			};
			
      $scope.getDataForDomain = function(array) {
				return DomainHelper.getDataForDomain(array, $scope.name);
			};
    }
  };
});



/* --- Parameters --- */

app.directive('ccParameter', function() {
	return {
		scope: { parameter: '=' },
		templateUrl: 'directives/parameter.html',
    controller: function($scope) {
      $scope.p = new ROSLIB.Param({ros: ros, name: $scope.parameter.name});

      $scope.setValue = function(value) { $scope.p.set(value); };
    }
  };
});



/* --- Services --- */

app.directive('ccServiceTemplate', function() {
	return {
		scope: { service: '=' },
		templateUrl: 'directives/service.html',
    controller: function($scope, $timeout) {
      $scope.callService = function(data) {
        var service = new ROSLIB.Service({ros: ros, name: $scope.service.name, serviceType: $scope.service.type});
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
    }
  };
});

var generateDirectiveServiceConfig = function(file_name, link) {
	link = typeof link !== 'undefined' ? link : null;
	
	return function() {
		return {
			templateUrl: 'directives/service_views/' + file_name + '.html',
			require: '^ccServiceTemplate',
			link: link
	  };
	};
};

app.directive('ccServiceDefault', generateDirectiveServiceConfig('default'));

app.directive('ccServiceEmpty', generateDirectiveServiceConfig('std_srvs/empty'));
app.directive('ccServiceTrigger', generateDirectiveServiceConfig('std_srvs/trigger'));
app.directive('ccServiceMovingpiBool', generateDirectiveServiceConfig('movingpi/bool'));



/* --- Topics --- */

app.directive('ccTopicTemplate', function() {
	return {
		scope: { topic: '=' },
		templateUrl: 'directives/topic.html',
		controller: function($scope, $timeout) {
			var roslib_topic = new ROSLIB.Topic({ros: ros, name: $scope.topic.name, messageType: $scope.topic.type});
			
			$scope.can_subscribe = true;
			$scope.is_subscribing = false;
			$scope.toggleSubscription = function(data) {
        if (!data) {
          roslib_topic.subscribe(function(message) {
            $timeout(function() {
              $scope.latest_message = message;
            });
          });
					$scope.is_subscribing = true;
        
				} else {
          roslib_topic.unsubscribe();
					$scope.is_subscribing = false;
        }
      };

      $scope.publishMessage = function(data) {
        var message = new ROSLIB.Message(data);
        roslib_topic.publish(message);
      };
			
			$scope.publishMessageJSON = function(data) {
        $scope.publishMessage(JSON.parse(input));
      };
    }
  };
});

var generateDirectiveConfig = function(file_name, link) {
	link = typeof link !== 'undefined' ? link : null;
	
	return function() {
		return {
			templateUrl: 'directives/topic_views/' + file_name + '.html',
			require: '^ccTopicTemplate',
			link: link
	  };
	};
};

app.directive('ccTopicDefault', generateDirectiveConfig('default'));

app.directive('ccTopicNumber', generateDirectiveConfig('std_msgs/number'));

app.directive('ccTopicFluidPressure', generateDirectiveConfig('sensor_msgs/fluid-pressure'));
app.directive('ccTopicIlluminance', generateDirectiveConfig('sensor_msgs/illuminance'));
app.directive('ccTopicImage', generateDirectiveConfig('sensor_msgs/image', function(scope) {
	scope.config = config;
	scope.quality = 75;
	scope.$parent.can_subscribe = false;
}));
app.directive('ccTopicImu', generateDirectiveConfig('sensor_msgs/imu'));
app.directive('ccTopicJoy', generateDirectiveConfig('sensor_msgs/joy'));
app.directive('ccTopicMagneticField', generateDirectiveConfig('sensor_msgs/magnetic-field'));
app.directive('ccTopicRange', generateDirectiveConfig('sensor_msgs/range'));
app.directive('ccTopicRelativeHumidity', generateDirectiveConfig('sensor_msgs/relative-humidity'));
app.directive('ccTopicTemperature', generateDirectiveConfig('sensor_msgs/temperature'));

app.directive('ccTopicPose', generateDirectiveConfig('geometry_msgs/pose'), function(scope) {
	var getOrientation = function() {
		if (scope.$parent.latest_message)
			return scope.$parent.latest_message.orientation;
		return {w: 1, x: 0, y: 0, z: 0};
	};
	
	scope.getRoll = function() {
		var q = getOrientation();
		return 180 / Math.PI * Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
	};
	
	scope.getPitch = function() {
		var q = getOrientation();
		return 180 / Math.PI *  Math.asin(2 * (q.w * q.y - q.z * q.x));
	};
	
	scope.getYaw = function() {
		var q = getOrientation();
		return 180 / Math.PI * Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
	};
});
app.directive('ccTopicPoseStamped', generateDirectiveConfig('geometry_msgs/pose-stamped'), function(scope) {
	var getOrientation = function() {
		if (scope.$parent.latest_message)
			return scope.$parent.latest_message.pose.orientation;
		return {w: 1, x: 0, y: 0, z: 0};
	};
	
	scope.getRoll = function() {
		var q = getOrientation();
		return 180 / Math.PI * Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
	};
	
	scope.getPitch = function() {
		var q = getOrientation();
		return 180 / Math.PI *  Math.asin(2 * (q.w * q.y - q.z * q.x));
	};
	
	scope.getYaw = function() {
		var q = getOrientation();
		return 180 / Math.PI * Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
	};
});
app.directive('ccTopicPose2d', generateDirectiveConfig('geometry_msgs/pose2D'));

app.directive('ccTopicMovingpiRange', generateDirectiveConfig('movingpi/range'));

app.directive('ccTopicFlypiMotorsThrust', generateDirectiveConfig('flypi/motors-thrust'));
app.directive('ccTopicFlypiSteering', generateDirectiveConfig('flypi/steering'));
