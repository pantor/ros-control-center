angular.module('roscc').component('ccParameter', {
  bindings: { parameter: '=' },
  templateUrl: 'app/parameters/parameters.html',
  controller() {
    function $onInit() {
      this.param = new ROSLIB.Param({ ros, name: this.parameter.name });
    }

    function setValue(value) {
      this.param.set(value);
    }

    this.$onInit = $onInit;
    this.setValue = setValue;
  },
});
