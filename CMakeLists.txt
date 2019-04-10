cmake_minimum_required(VERSION 2.8.3)
project(tams_midi_publisher)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  MidiMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_generation message_runtime std_msgs
)

include_directories(
# include
SYSTEM ${catkin_INCLUDE_DIRS}
)

add_executable(midi_publisher src/midi_publisher.cpp)
add_dependencies(midi_publisher ${catkin_EXPORTED_TARGETS} tams_midi_publisher_generate_messages_cpp)
target_link_libraries(midi_publisher ${catkin_LIBRARIES} rtmidi)

add_executable(midi_reader src/midi_reader.cpp)
add_dependencies(midi_reader ${catkin_EXPORTED_TARGETS} tams_midi_reader_generate_messages_cpp)
target_link_libraries(midi_reader ${catkin_LIBRARIES} rtmidi)
