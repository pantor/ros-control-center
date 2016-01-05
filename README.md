# ROS Control Center

This control center is a universal tool for controlling robots running ROS. It runs in the browser using a websocket connection and `roslibjs` from [RobotWebTools](http://robotwebtools.org).

![screenshot](https://raw.githubusercontent.com/gaug-cns/ros-control-center/master/images/screenshot.png)


## Installation & Tutorial

Please see the [GitHub Wiki](https://github.com/gaug-cns/ros-control-center/wiki).


## Contributing

For contributing, [gulp](http://gulpjs.com) needs to be installed on your local development machine. It is used for concat and linting JavaScript files. Gulp depends on the [Node.js](https://nodejs.org/en/) package manager [NPM](https://www.npmjs.com). Via `npm install` it will automatically all development dependencies for ROS Control Center, which are listed in `package.json`. Type `gulp` in the terminal for automatically concatenation of JS files and `gulp js-lint` for linting.

Contributions are always welcome!


## License

ROS Control Center is released with a BSD license. For full terms and conditions, see the [LICENSE](https://github.com/gaug-cns/ros-control-center/blob/master/LICENSE) file.


## Authors

See [AUTHORS.md](https://github.com/gaug-cns/ros-control-center/blob/master/AUTHORS.md) for a full list of contributors.
