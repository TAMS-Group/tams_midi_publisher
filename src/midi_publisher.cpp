#include <ros/ros.h>
#include "RtMidi.h"
#include <tams_midi_publisher/MidiMessage.h>

class MidiPublisher
{
public:
  MidiPublisher() : midiin()
  {
    ros::NodeHandle pn("~");
    int midi_port;
    std::string midi_name;
    std::string midi_name_substring;
    bool ignore_sysex;
    bool ignore_timing;
    bool ignore_active_sensing;
    pn.param("midi_port", midi_port, -1);
    pn.param("midi_name", midi_name, std::string(""));
    pn.param("midi_name_substring", midi_name_substring, std::string(""));
    pn.param("ignore_sysex", ignore_sysex, false);
    pn.param("ignore_timing", ignore_timing, false);
    pn.param("ignore_active_sensing", ignore_active_sensing, false);

    if(midi_port == -1 && midi_name == "" && midi_name_substring == "")
    {
      ROS_ERROR_STREAM("no midi port or name provided. shutting down.");
      ros::shutdown();
    }

    size_t port_count = midiin.getPortCount();
    bool found_interface = false;
    for(size_t i = 0; i < port_count; ++i)
    {
      std::string port_id = midiin.getPortName(i);
      std::size_t found = port_id.find_last_of(" ");
      std::string port_name = port_id.substr(0, found);

      port_id = port_id.substr(found+1);
      port_id = port_id.substr(0, port_id.size() - 2);
      int port_number = std::stoi(port_id);

      if(port_number == midi_port || (!midi_name.empty() && port_name == midi_name) || (!midi_name_substring.empty() &&  (port_name.find(midi_name_substring) != std::string::npos)))
      {
        midiin.openPort(i);
        ROS_INFO_STREAM("found interface: " << port_name);
        found_interface = true;
        break;
      }
    }
    if(!found_interface)
    {
      ROS_ERROR_STREAM("interface not found. shutting down.");
      ros::shutdown();
    }

    midiin.ignoreTypes(ignore_sysex, ignore_timing, ignore_active_sensing);

    ros::NodeHandle nh;
    midi_pub_ = nh.advertise<tams_midi_publisher::MidiMessage>("midi_message", 1000);
  }
  
  void run()
  {
    std::vector<unsigned char> message;
    tams_midi_publisher::MidiMessage mm;
    ros::Rate r(100);

    // Periodically check input queue.
    ROS_INFO_STREAM("Reading MIDI messages. Quit with Ctrl-C.");
    while (ros::ok())
    {
      midiin.getMessage(&message);
      if(message.size() != 3)
        continue;
      mm.header.stamp = ros::Time::now();
      // to only compare the first four bits they are shifted to the right
      message[0] >>= 4;
      if(message[0] == 0b00001000)
        mm.note_on = false;
      else if(message[0] == 0b00001001)
        mm.note_on = true;
      else
        continue;
      mm.note_number = message[1];
      mm.note_velocity = message[2];
      // for some devices velocity 0 is defined as note off command
      if(mm.note_velocity == 0)
        mm.note_on = false;
      midi_pub_.publish(mm);
      r.sleep();
    }
  }
private:
  RtMidiIn midiin;
  ros::Publisher midi_pub_;
  std::map<int, std::string> num2key_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "midi_pub_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  MidiPublisher mp;
  mp.run();
}
