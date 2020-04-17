tams_midi_publisher
===================

Add Midi reader / message support to ROS.

This package provides two executables:

`midi_publisher` opens a Midi port and publishes arriving events as ROS messages.

`midi_reader` subscribes to a ROS topic and sends messages to the Midi system.
This is useful for replaying Midi recorded in rosbags.
