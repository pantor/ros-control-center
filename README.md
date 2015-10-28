# ROS Control Center

This control center is a universal tool for controlling robots running ROS. It runs in the browser using a websocket connection with the robot and [roslibjs](http://robotwebtools.org).

![screenshot](https://raw.githubusercontent.com/gaug-cns/ros-control-center/master/images/screenshot.png)



## Installation

On your ROS robot, the `rosbridge_suite` from [Robot Web Tools](http://robotwebtools.org) needs to run.

The control center runs on any other computer (in particular without ROS...) in the same network. Simply rename `config.example.json` to `config.json` and change the IP address to your robot. Finally, open `index.html` in a browser.



## Guidance

The right sidebar shows a logger output (`rosout` by default). On the left, group names are shown. ROS topics, 
services and parameters can be grouped together for a better overview. This works as follows:
- Every topic, service or parameter name should correspond to `/group-name/element-name` like a URL. Global parameters should have no group name and are shown below the group list on the left side. 
- Every element name should start with a capital letter, as they are shown in the normal view mode. All elements (including the lowercase-names from system services, topics and parameters) can be shown by enabling the `advanced` view mode.

ROS Control Center supports images and camera streams via the `web_video_server` package. If camera and camera info messages are published according to the `web_video_server` standards, the stream is shown with the settings made in `config.json`.

Further preferences can be adapted in `config.json`.


## Development

ROS Control Center depends on:
- [AngularJS](https://www.angularjs.org) as a JavaScript and routing framework.
- [Bootstrap](http://getbootstrap.com) for CSS designs.
- [_underscore.js](http://underscorejs.org) for a general-purpose JavaScript library.
- [roslib.js](http://robotwebtools.org) for ROS connection. 

Every message type can have its own view, so that all data can be formatted very readable. Therefore, angular directives are made for the most common message types; you can add your own very easily.


## License

ROS Control Center is released with a BSD license. For full terms and conditions, see the LICENSE file.