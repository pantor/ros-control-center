function topicDirective() {
  return {
    scope: { topic: '=' },
    template: '<ng-include src=\"vm.fileName\"></ng-include>',
    controllerAs: 'vm',
    controller($scope, $timeout, $http, Settings, Quaternions) {
      const roslibTopic = new ROSLIB.Topic({
        ros,
        name: $scope.topic.name,
        messageType: $scope.topic.type,
      });
      const path = 'app/topics/';

      this.topic = $scope.topic;
      this.toggleSubscription = toggleSubscription;
      this.publishMessage = publishMessage;
      this.isSubscribing = false;
      this.setting = Settings.get();
      this.Quaternions = Quaternions;
      this.fileName = `${path}default.html`;

      // Check if file exists
      $scope.$watch('topic.type', () => {
        if (!$scope.topic.type) {
          return;
        }
        const fileName = '${path}${$scope.topic.type}.html';

        this.topic = $scope.topic;
        $http.get(fileName).then(result => {
          if (result.data) {
            this.fileName = fileName;
          }
        });
      });

      function toggleSubscription(data) {
        if (!data) {
          roslibTopic.subscribe(message => {
            $timeout(() => {
              this.message = message;
            });
          });
        } else {
          roslibTopic.unsubscribe();
        }
        this.isSubscribing = !data;
      }

      function publishMessage(input, isJSON) {
        const data = isJSON ? angular.fromJSON(input) : input;
        const message = new ROSLIB.Message(data);
        roslibTopic.publish(message);
      }
    },
  };
}

angular.module('roscc').directive('ccTopic', topicDirective);
