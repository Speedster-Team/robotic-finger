#include "serial_interface/serial_interface.hpp"


SerialInterface::SerialInterface()
: _port ("/dev/ttyACM0"),
  _baud (115200),
  _msg_status (MessageStatus::NO_STATUS),
  _fdbk_status (FeedbackStatus::NOTHING_NEW),
  _serial (std::make_shared<serial::Serial>(_port, _baud, serial::Timeout::simpleTimeout(1)))
{
  // check if it was successfully opened
  if (_serial->isOpen()) {
    std::cout << "Serial port opened successfully" << std::endl;
  } else {
    std::cerr << "Failed to open serial port" << std::endl;
  }

  // define lambda function to generate crc table
  auto build_table = [this]() {
      for (int i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (int b = 0; b < 8; b++) {
          crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
        _crc_table[i] = crc;
      }
    };

  // build the table
  build_table();
}

void SerialInterface::send_command(
  std::vector<std::vector<float>> q_motor_list, int length,
  int repeat, char mode)
{
  // set message status
  _msg_status = MessageStatus::NO_STATUS;

  // Build data lines first so we can CRC them the same way Teensy does
  std::vector<std::string> data_lines;
  for (auto & q_motor : q_motor_list) {
    data_lines.push_back(
            std::to_string(q_motor.at(0)) + " " +
            std::to_string(q_motor.at(1)) + " " +
            std::to_string(q_motor.at(2))   // no \n yet — match Teensy's strlen behavior
    );
  }

  // compute checksum
  uint8_t checksum = 0x00;
  for (auto & line : data_lines) {
    for (unsigned char c : line) {
      checksum = _crc_table[checksum ^ c];
    }
  }

  // Assemble payload
  std::string payload = "D " + std::to_string(length) + " " + std::to_string(repeat) + " " + mode + "\n";
  for (auto & line : data_lines) {
    payload += line + "\n";
  }
  payload += std::to_string(checksum) + "\n";
  payload += "end\n";     // "end" on its own line

  std::cout << "Sending " << data_lines.size() << " motor position lines." << std::endl;
  _serial->write(payload);


}

void SerialInterface::send_start()
{

  // set message status
  _msg_status = MessageStatus::NO_STATUS;

  // Assemble payload
  std::string payload = "G\n";
  payload += "end\n";     // "end" on its own line

  std::cout << "Sending " << "Go." << std::endl;
  _serial->write(payload);
}

void SerialInterface::send_stop()
{
  // set message status
  _msg_status = MessageStatus::NO_STATUS;

  // Assemble payload
  std::string payload = "S\n";
  payload += "end\n";     // "end" on its own line

  std::cout << "Sending " << "stop." << std::endl;
  _serial->write(payload);
}

void SerialInterface::parse_response()
{
  // get data, wait for 1ms if no \n char
  std::string response = _serial->readline();

  // check if a message was returned, if not return
  if (response.empty()) {
    return;
  }

  // Parse and validate
  int resp_success;

  if (sscanf(response.c_str(), "%f %f %f %f %f %f %f", &_mcp_splay_motor_pos, &_mcp_flex_motor_pos,
    &_pip_flex_motor_pos, &cmd_mcp_splay_motor_pos_, &cmd_mcp_flex_motor_pos_,
    &cmd_pip_flex_motor_pos_, &_active) == 7)
  {
    _fdbk_status = FeedbackStatus::NEW_FEEDBACK;

  } else if (sscanf(response.c_str(), "%d", &resp_success) == 1) {
    // check for success in message reception
    if (resp_success == 1) {
      _msg_status = MessageStatus::SUCCESS;
    } else {
      _msg_status = MessageStatus::FAILURE;
    }
  } else {
    std::cerr << "Could not parse Teensy response: " << response << std::endl;
  }
}

MessageStatus SerialInterface::get_message_status()
{
  return _msg_status;
}

FeedbackStatus SerialInterface::get_feedback_status()
{
  return _fdbk_status;
}

std::vector<float> SerialInterface::get_feedback()
{
  _fdbk_status = FeedbackStatus::NOTHING_NEW;
  return std::vector<float> {_mcp_splay_motor_pos, _mcp_flex_motor_pos, _pip_flex_motor_pos,
    cmd_mcp_splay_motor_pos_, cmd_mcp_flex_motor_pos_, cmd_pip_flex_motor_pos_, _active};

}
