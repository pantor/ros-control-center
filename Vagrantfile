# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure(2) do |config|
  config.vm.box = "nicolov/xenial-ros"
  config.vm.network "forwarded_port", guest: 9090, host: 9090
  config.vm.synced_folder "ros_test_project", "/home/vagrant/catkin_ws/src/ros_test_project"

  config.vm.provision "shell", inline: <<-SHELL
    sudo apt-get install ros-kinetic-rosbridge-server
  SHELL
end
