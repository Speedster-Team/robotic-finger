#ifndef SERIAL_INTERFACE_HPP
#define SERIAL_INTERFACE_HPP

#include <string>
#include <iostream>
#include <cstdio>
#include <memory>
#include <unistd.h>
#include <vector>
#include <armadillo>
#include "serial/serial.h"

/// \brief State variable monitoring the feedback status of a message sent. State is NO_STATUS until feedback recieved
enum MessageStatus
{
  NO_STATUS,
  SUCCESS,
  FAILURE,
};

/// \brief State variable showing state of position feedback from motors
enum FeedbackStatus
{
  NEW_FEEDBACK,
  NOTHING_NEW,
};

/// \brief Class abstraction of serial communication with teensy to send commands and recieve feedback
class SerialInterface
{
public:
  /// \brief Create instance of SerialInterface thus initializing and opening serial port
  SerialInterface();

  /// \brief Parse feedback sent from teensy, either ack or position feedback
  void parse_response();

  /// \brief Send a command containing trajectory to the teensy
  /// \param q_motor_list - A nx3 matrix (vector of vectors) containing motor commands at 100hz
  /// \param length - The length of the data message
  /// \param repeat - A 1 or 0 indicating if trajectory should be repeated
  /// \param mode - The control mode type for this trajectory
  void send_command(std::vector<std::vector<float>> q_motor_list, int length, int repeat, char mode);

  /// \brief Send go command to teensy
  void send_start();

  /// \brief Send stop command to teensy
  void send_stop();

  /// \brief Access the message state machine state
  MessageStatus get_message_status();

  /// \brief Access the feedback state machine state
  FeedbackStatus get_feedback_status();

  /// \brief Get the position feedback sent by the teensy
  std::vector<float> get_feedback();

private:
  /// \brief The name of the port to connect to the teensy
  std::string _port;
  /// \brief The baud rate of serial communicatino
  uint32_t _baud;
  /// \brief State variable containing state of sent messages
  MessageStatus _msg_status;
  /// \brief State variable containing state of sent messages
  FeedbackStatus _fdbk_status;
  /// \brief The serial interface variable
  std::shared_ptr<serial::Serial> _serial;
  /// \brief The crc8 lookup table
  uint8_t _crc_table[256];
  /// \brief The MCP splay motor position
  float _mcp_splay_motor_pos;
  /// \brief The MCP felx motor position
  float _mcp_flex_motor_pos;
  /// \brief The PIP flex motor position
  float _pip_flex_motor_pos;
    /// \brief The MCP splay motor position command
  float cmd_mcp_splay_motor_pos_;
  /// \brief The MCP felx motor position command
  float cmd_mcp_flex_motor_pos_;
  /// \brief The PIP flex motor position command
  float cmd_pip_flex_motor_pos_;
  /// \brief State variable showing if control is active
  float _active;


};

#endif
