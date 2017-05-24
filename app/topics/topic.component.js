class TopicController {
  constructor($scope, $http, Settings, Quaternions) {
    this.roslibTopic = new ROSLIB.Topic({
      ros,
      name: this.topic.name,
      messageType: this.topic.type,
    });

    this.isSubscribing = false;
    this.setting = Settings.get();
    this.Quaternions = Quaternions;

    const path = 'app/topics/';
    this.fileName = `${path}default.html`;

    // Check if file exists
    $scope.$watch('topic.type', () => {
      if (!this.topic.type) {
        return;
      }
      const fileName = `${path}${this.topic.type}.html`;
      $http.get(fileName).then((result) => {
        if (result.data) {
          this.fileName = fileName;
        }
      });
    });
  }

  toggleSubscription(data) {
    if (!data) {
      this.roslibTopic.subscribe((message) => {
        this.message = message;
      });
    } else {
      this.roslibTopic.unsubscribe();
    }
    this.isSubscribing = !data;
  }

  publishMessage(input, isJSON) {
    const data = isJSON ? angular.fromJson(input) : input;
    const message = new ROSLIB.Message(data);
    this.roslibTopic.publish(message);
  }
}

angular.module('roscc').component('ccTopic', {
  bindings: { topic: '=' },
  template: '<ng-include src="$ctrl.fileName"></ng-include>',
  controller: TopicController,
});
