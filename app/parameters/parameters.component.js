class ParameterController {
  $onInit() {
    this.param = new ROSLIB.Param({ ros, name: this.parameter.name });
  }

  setValue(value) {
    this.param.set(value);
  }
}

angular.module('roscc').component('ccParameter', {
  bindings: { parameter: '=' },
  templateUrl: 'app/parameters/parameters.html',
  controller: ParameterController,
});
