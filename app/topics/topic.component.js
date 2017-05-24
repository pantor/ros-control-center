class TopicController {
  constructor($scope, $http, Settings, Quaternions) {
    this.$scope = $scope;
    this.$http = $http;
    this.setting = Settings.get();
    this.Quaternions = Quaternions;

    this.isSubscribing = false;
  }

  $onInit() {
    this.roslibTopic = new ROSLIB.Topic({
      ros,
      name: this.topic.name,
      messageType: this.topic.type,
    });

    const path = 'app/topics/';
    this.fileName = `${path}default.html`;

    // Check if file exists
    this.$scope.$watch('topic.type', () => {
      if (!this.topic.type) {
        return;
      }
      const fileName = `${path}${this.topic.type}.html`;
      this.$http.get(fileName).then((result) => {
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
