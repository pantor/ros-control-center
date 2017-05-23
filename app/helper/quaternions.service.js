class QuaternionsService {
  getRoll(q) {
    if (!q) {
      return '';
    }
    const rad = Math.atan2(2 * ((q.w * q.x) + (q.y * q.z)), 1 - (2 * ((q.x * q.x) + (q.y * q.y))));
    return (180 / Math.PI) * rad;
  }

  getPitch(q) {
    if (!q) {
      return '';
    }
    const rad = Math.asin(2 * ((q.w * q.y) - (q.z * q.x)));
    return (180 / Math.PI) * rad;
  }

  getYaw(q) {
    if (!q) {
      return '';
    }
    const rad = Math.atan2(2 * ((q.w * q.z) + (q.x * q.y)), 1 - (2 * ((q.y * q.y) + (q.z * q.z))));
    return (180 / Math.PI) * rad;
  }

  getInit() {
    return { w: 1, x: 0, y: 0, z: 0 };
  }
}

// Quaternions to Euler angles converter
angular.module('roscc').service('Quaternions', QuaternionsService);
