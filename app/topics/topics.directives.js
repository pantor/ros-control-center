function topicDirective(fileName) {
  return function () {
    return {
      scope: { topic: '=' },
      templateUrl: 'app/topics/' + fileName + '.html',
      controllerAs: 'vm',
      controller: function ($scope, $timeout, Settings, Quaternions) {
        var vm = this;
        var roslibTopic = new ROSLIB.Topic({
          ros: ros,
          name: $scope.topic.name,
          messageType: $scope.topic.type,
        });

        vm.topic = $scope.topic;
        vm.toggleSubscription = toggleSubscription;
        vm.publishMessage = publishMessage;
        vm.publishMessageJSON = publishMessageJSON;
        vm.isSubscribing = false;
        vm.setting = Settings.get();
        vm.Quaternions = Quaternions;


        function toggleSubscription(data) {
          if (!data) {
            roslibTopic.subscribe(function (message) {
              $timeout(function () {
                vm.message = message;
              });
            });
          } else {
            roslibTopic.unsubscribe();
          }

          vm.isSubscribing = !data;
        }

        function publishMessage(data) {
          var message = new ROSLIB.Message(data);
          roslibTopic.publish(message);
        }

        function publishMessageJSON(data) {
          publishMessage(angular.fromJSON(data));
        }
      },
    };
  };
}


angular.module('roscc')
  .directive('ccTopicDefault', topicDirective('default'))

  .directive('ccTopicNumber', topicDirective('std_msgs/number'))

  .directive('ccTopicFluidPressure', topicDirective('sensor_msgs/fluid-pressure'))
  .directive('ccTopicIlluminance', topicDirective('sensor_msgs/illuminance'))
  .directive('ccTopicImage', topicDirective('sensor_msgs/image'))
  .directive('ccTopicImu', topicDirective('sensor_msgs/imu'))
  .directive('ccTopicJoy', topicDirective('sensor_msgs/joy'))
  .directive('ccTopicMagneticField', topicDirective('sensor_msgs/magnetic-field'))
  .directive('ccTopicRange', topicDirective('sensor_msgs/range'))
  .directive('ccTopicRelativeHumidity', topicDirective('sensor_msgs/relative-humidity'))
  .directive('ccTopicTemperature', topicDirective('sensor_msgs/temperature'))

  .directive('ccTopicPose', topicDirective('geometry_msgs/pose'))
  .directive('ccTopicPoseStamped', topicDirective('geometry_msgs/pose-stamped'))
  .directive('ccTopicPose2d', topicDirective('geometry_msgs/pose2D'))

  .directive('ccTopicFlypiMotorsThrust', topicDirective('flypi/motors-thrust'))
  .directive('ccTopicFlypiSteering', topicDirective('flypi/steering'))

  .directive('ccTopicVisionodometryMotion', topicDirective('visionodometry/motion'));
