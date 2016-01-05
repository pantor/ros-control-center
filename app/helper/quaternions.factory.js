function QuaternionsFactory() {
  return {
    getRoll: function (q) {
      var rad = Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
      return 180 / Math.PI * rad;
    },
    getPitch: function (q) {
      var rad = Math.asin(2 * (q.w * q.y - q.z * q.x));
      return 180 / Math.PI * rad;
    },
    getYaw: function (q) {
      var rad = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      return 180 / Math.PI * rad;
    },
    getInit: function () {
      return { w: 1, x: 0, y: 0, z: 0 };
    },
  };
}

// Filter advanced topics, services, parameters by checking the beginning capital letter
angular.module('roscc').factory('Quaternions', QuaternionsFactory);
