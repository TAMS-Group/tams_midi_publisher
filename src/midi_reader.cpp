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
    std::string midi_name;
    std::string midi_name_substring;
    double wait_at_start;
    pn.param("midi_port", midi_port, -1);
    pn.param("midi_name", midi_name, std::string(""));
    pn.param("midi_name_substring", midi_name_substring, std::string(""));
    pn.param("wait_at_start", wait_at_start, 0.0);
    ros::Duration(wait_at_start).sleep();

    if(midi_port == -1 && midi_name == "" && midi_name_substring == "")
    {
      ROS_ERROR_STREAM("no midi port  or name provided. shutting down.");
      ros::shutdown();
    }

    size_t port_count = midiout.getPortCount();
    bool found_interface = false;
    for(size_t i = 0; i < port_count; ++i)
    {
      std::string port_id = midiout.getPortName(i);
      std::size_t found = port_id.find_last_of(" ");
      std::string port_name = port_id.substr(0, found);

      port_id = port_id.substr(found+1);
      port_id = port_id.substr(0, port_id.size() - 2);
      int port_number = std::stoi(port_id);

      if(port_number == midi_port || port_name == midi_name || port_name.find(midi_name_substring) != std::string::npos)
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
