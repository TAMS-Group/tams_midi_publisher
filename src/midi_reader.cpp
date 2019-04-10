#include <ros/ros.h>
#include "RtMidi.h"
#include <tams_midi_publisher/MidiMessage.h>

class MidiReader
{
public:
  MidiReader() : midiout()
  {
    ros::NodeHandle pn("~");
    int midi_port;
    pn.param("midi_port", midi_port, -1);

    if(midi_port == -1)
    {
      ROS_ERROR_STREAM("no midi port provided. shutting down.");
      ros::shutdown();
    }

    size_t port_count = midiout.getPortCount();
    bool found_interface = false;
    for(size_t i = 0; i < port_count; ++i)
    {
      std::string port_name = midiout.getPortName(i);
      std::size_t found = port_name.find_last_of(" ");
      port_name = port_name.substr(found+1);
      port_name = port_name.substr(0, port_name.size() - 2);
      int port_id = std::stoi(port_name);

      if(port_id == midi_port)
      {
        midiout.openPort(i);
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

    ros::NodeHandle nh;
    midi_sub_ = nh.subscribe("midi_message", 1000, &MidiReader::midi_cb, this);
  }
private:
  void midi_cb(const tams_midi_publisher::MidiMessage::ConstPtr& msg)
  {
    std::vector<unsigned char> message;
    if(msg->note_on)
    {
      message.push_back(144);
    }
    else
    {
      message.push_back(128);
    }
    message.push_back(static_cast<unsigned char>(msg->note_number));
    message.push_back(static_cast<unsigned char>(msg->note_velocity));

    midiout.sendMessage(&message);
  }

  RtMidiOut midiout;
  ros::Subscriber midi_sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "midi_pub_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  MidiReader mr;
  ros::waitForShutdown();
}
