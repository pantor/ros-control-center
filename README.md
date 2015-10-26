# ROS Control Center

The control center is a universal tool for all ROS robots.


## Using

The control center is purely written in HTML and JavaScript, so that it can be startet by opening `index.html` in a browser.

The right sidebar shows a logger output (`rosout` by default). On the left, group name are shown. ROS topics, 
services and parameters can be grouped together for a better overview. This works as follows:

Every topic, service or parameter name should correspond to `/group-name/element-name` like a URL. 
Global parameters should have no group name and are shown below the group list on the left side. 

Every element name should start with a capital letter, as they are shown in the normal view mode.
All elements (including the lowercase-names from system services, topics and parameters) can be shown by enabling the `advanced` view mode.


There is a config file at `config.example.json`, which can be adapted and renamed to `config.json`. 
It includes the robot url (given by the rosbridge_suite package) and the logger topic name.

On the robot, the `rosbridge_suite` from the [Robot Web Tools](http://robotwebtools.org) needs to run.


## Development

Every topic and service message can have its own angular-directive. In this way, all data can be formatted very readable.

The control center depends on:

- [AngularJS](https://www.angularjs.org) as a JavaScript framework
- [Bootstrap](http://getbootstrap.com) for CSS-designs.
- [_underscore.js](http://underscorejs.org) for a general JavaScript library.
- [roslib.js](http://robotwebtools.org) for ROS connection. 