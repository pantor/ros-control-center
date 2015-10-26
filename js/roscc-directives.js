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
		templateUrl: 'directives/services/template.html',
    controller: function($scope, $timeout) {
      $scope.callService = function(data) {
        var service = new ROSLIB.Service({ros: ros, name: $scope.service.name, serviceType: $scope.service.type});
        var request = new ROSLIB.ServiceRequest(data);
				
				console.log(data);

        service.callService(request, function(result) {
          $timeout(function () {
            $scope.result = result;
          });
        });
      };
    }
  };
});

app.directive('ccServiceDefault', function() {
	return {
		templateUrl: 'directives/services/default.html',
		require: '^ccServiceTemplate',
    link: function(scope) {
      scope.callService = function(input) {
				scope.$parent.callService(JSON.parse(input));
      };
    }
  };
});

app.directive('ccServiceEmpty', function() {
	return {
		templateUrl: 'directives/services/empty.html',
		require: '^ccServiceTemplate'
  };
});

app.directive('ccServiceTrigger', function() {
	return {
		templateUrl: 'directives/services/trigger.html',
		require: '^ccServiceTemplate'
  };
});

app.directive('ccServiceMovingpiBool', function() {
	return {
		templateUrl: 'directives/services/movingpi-bool.html',
		require: '^ccServiceTemplate'
  };
});



/* --- Topics --- */

app.directive('ccTopicTemplate', function() {
	return {
		scope: { topic: '=' },
		templateUrl: 'directives/topics/template.html',
		controller: function($scope, $timeout) {
			var roslib_topic = new ROSLIB.Topic({ros: ros, name: $scope.topic.name, messageType: $scope.topic.type});
			
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
				console.log(message);
        roslib_topic.publish(message);
      };
    }
  };
});

app.directive('ccTopicDefault', function() {
	return {
		templateUrl: 'directives/topics/default.html',
		require: '^ccTopicTemplate',
    link: function(scope) {
      scope.publishMessage = function(input) {
        scope.$parent.publishMessage(JSON.parse(input));
      };
    }
  };
});

app.directive('ccTopicNumber', function() {
	return {
		templateUrl: 'directives/topics/number.html',
		require: '^ccTopicTemplate'
  };
});

app.directive('ccTopicTemperature', function() {
	return {
		templateUrl: 'directives/topics/temperature.html',
		require: '^ccTopicTemplate'
  };
});

app.directive('ccTopicFluidPressure', function() {
	return {
		templateUrl: 'directives/topics/fluid-pressure.html',
		require: '^ccTopicTemplate'
  };
});

app.directive('ccTopicImu', function() {
	return {
		templateUrl: 'directives/topics/imu.html',
		require: '^ccTopicTemplate'
  };
});

app.directive('ccTopicMagneticField', function() {
	return {
		templateUrl: 'directives/topics/magnetic-field.html',
		require: '^ccTopicTemplate'
  };
});

app.directive('ccTopicJoy', function() {
	return {
		templateUrl: 'directives/topics/joy.html',
		require: '^ccTopicTemplate'
  };
});

app.directive('ccTopicRange', function() {
	return {
		templateUrl: 'directives/topics/range.html',
		require: '^ccTopicTemplate'
  };
});

app.directive('ccTopicIlluminance', function() {
	return {
		templateUrl: 'directives/topics/illuminance.html',
		require: '^ccTopicTemplate'
  };
});

app.directive('ccTopicPose', function() {
	return {
		templateUrl: 'directives/topics/pose.html',
		require: '^ccTopicTemplate',
    link: function(scope) {
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
    }
  };
});

app.directive('ccTopicPose2d', function() {
	return {
		templateUrl: 'directives/topics/pose2D.html',
		require: '^ccTopicTemplate'
  };
});

app.directive('ccTopicImage', function() {
	return {
		templateUrl: 'directives/topics/image.html',
		require: '^ccTopicTemplate',
    link: function(scope) {
			scope.config = config;
			scope.quality = 75;
    }
  };
});

app.directive('ccTopicMovingpiRange', function() {
	return {
		templateUrl: 'directives/topics/movingpi-range.html',
		require: '^ccTopicTemplate'
  };
});

app.directive('ccTopicFlypiMotorsThrust', function() {
	return {
		templateUrl: 'directives/topics/flypi-motors-thrust.html',
		require: '^ccTopicTemplate'
  };
});

app.directive('ccTopicFlypiSteering', function() {
	return {
		templateUrl: 'directives/topics/flypi-steering.html',
		require: '^ccTopicTemplate'
  };
});
