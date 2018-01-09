# ROS Control Center

This control center is a universal tool for controlling robots running ROS. It runs in the browser using a websocket connection and `roslibjs` from [RobotWebTools](http://robotwebtools.org).

[![screenshot](https://raw.githubusercontent.com/pantor/ros-control-center/master/src/assets/screenshot.png)](http://pantor.github.io/ros-control-center)

In general, ROS Control Center offers an easy way to
 - show nodes, topics and service names.
 - subscribe and publish messages.
 - call services.
 - show and change parameters.

Further features like a console (rosout by default) output, a battery status view, a camera stream view or an advanced mode for better usability are implemented. [Check it out!](http://pantor.github.io/ros-control-center)


## Installation

On your ROS robot, the `rosbridge_suite` from [Robot Web Tools](http://robotwebtools.org) needs to run. So to use the control center with your existing project, call `roslaunch rosbridge_server rosbridge_websocket.launch` to launch the websocket server. The control center runs on any computer (in particular without ROS...) in the same network. Open the ROS Control Center at [https://pantor.github.io/ros-control-center](http://pantor.github.io/ros-control-center). In the settings tab, you need to enter the IP address and port of your robot. Open the `Control` tab and reload.


## Features

You can have multiple saved settings for quick changes and several robots.

ROS Control Center supports images and camera streams via the `web_video_server` package. If camera and camera info messages are published according to the `web_video_server` standards, the stream is shown with the settings.

For your own custom message and service types, you can download this repository and start a server via `http-server` in the console. Then, navigate to `index.html` in a browser. You can add individual `html` templates into the `app/topics/` or `app/services/` folder. The path of your file must correspond to the ROS service or message type name. See the included ROS common messages and standard services as an examples. In your html file, you can write `AngularJS` code for additional customizing and formatting.

The right sidebar shows a logger output (`rosout` by default). On the left, group names are shown. ROS topics,
services and parameters can be grouped together for a better overview. This works as follows:
- Every topic, service or parameter name should correspond to `/group-name/element-name` like a URL. Global parameters should have no group name and are shown below the group list on the left side.
- Every element name should start with a capital letter, as they are shown in the normal view mode. All elements (including the lowercase-names from system services, topics and parameters) can be shown by enabling the `advanced` view mode.

In the right bottom corner, a battery status bar can be shown; the battery topic can be adapted in the settings tab.


## Contributing

For development, [Node.js](https://nodejs.org/en/) needs to be installed. Via `npm install` it will automatically download all development dependencies (from `package.json`). Type `npm start` in the terminal for a live development server. With `vagrant ssh` and `roslaunch ros_test_project main.launch`, you can start a test project in a virtual ubuntu machine.

ROS Control Center depends on:
 - [Angular](https://www.angular.io/) as the general JavaScript and routing framework.
 - [Bootstrap](https://getbootstrap.com/) for design.
 - [roslib.js](https://github.com/RobotWebTools/roslibjs) for ROS connection.

Contributions are always welcome!


## License

ROS Control Center is released with a BSD license. For full terms and conditions, see the [LICENSE](https://github.com/pantor/ros-control-center/blob/master/LICENSE) file.


## Contributors

See [here](https://github.com/pantor/ros-control-center/graphs/contributors) for a full list of contributors.
