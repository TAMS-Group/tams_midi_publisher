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
    bool ignore_sysex;
    bool ignore_timing;
    bool ignore_active_sensing;
    pn.param("midi_port", midi_port, -1);
    pn.param("ignore_sysex", ignore_sysex, false);
    pn.param("ignore_timing", ignore_timing, false);
    pn.param("ignore_active_sensing", ignore_active_sensing, false);

    if(midi_port == -1)
    {
      ROS_ERROR_STREAM("no midi port provided. shutting down.");
      ros::shutdown();
    }

    size_t port_count = midiin.getPortCount();
    bool found_interface = false;
    for(size_t i = 0; i < port_count; ++i)
    {
      std::string port_name = midiin.getPortName(i);
      std::size_t found = port_name.find_last_of(" ");
      port_name = port_name.substr(found+1);
      port_name = port_name.substr(0, port_name.size() - 2);
      int port_id = std::stoi(port_name);

      if(port_id == midi_port)
      {
        midiin.openPort(i);
        ROS_INFO_STREAM("found interface.");
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
    int nBytes, i;
    tams_midi_publisher::MidiMessage mm;
    ros::Rate r(100);

    // Periodically check input queue.
    ROS_INFO_STREAM("Reading MIDI messages. Quit with Ctrl-C.");
    while (ros::ok())
    {
      midiin.getMessage(&message);
      if(message.empty())
        continue;
      mm.header.stamp = ros::Time::now();
      mm.data.clear();
      for(size_t i = 0; i < message.size(); ++i)
      {
        mm.data.push_back(message[i]);
      }
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
