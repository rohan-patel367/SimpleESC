#include <SimpleCAN.h>
#include <SimpleFOC.h>

class CANBroker : public CANNotifications
{
public:
  CANBroker(Commander *command, BLDCMotor *motor, byte ID)
  {
    command_ = command;
    motor_ = motor;
    ID_ = ID;
    // CANDevice_ = CANDevice;
  };
  void setEnable(byte index)
  {
    if (index == 1)
    {
      motor_->enable();
    }
    else
    {
      motor_->disable();
    }
  }
  void setTorqueType(byte index)
  {
    motor_->torque_controller = (TorqueControlType)index;
  }
  void setLimit(byte index, float value)
  {
    switch (index)
    {
    case CAN_MESSAGE::LIMIT::CURRENT_LIMIT:
      motor_->current_limit = value;
      break;
    case CAN_MESSAGE::LIMIT::VOLTAGE_LIMIT:
      motor_->voltage_limit = value;
      motor_->PID_current_d.limit = value;
      motor_->PID_current_q.limit = value;
      break;
    case CAN_MESSAGE::LIMIT::VELOCITY_LIMIT:
      motor_->velocity_limit = value;
      motor_->P_angle.limit = value;
      break;
    default:
      return;
    }
  }
  void setPID(byte command, byte index, float value)
  {
    switch (command)
    {
    case CAN_MESSAGE::SET_Q_PID_GAIN:
      switch (index)
      {
      case CAN_MESSAGE::CONTROLLER::P_GAIN:
        motor_->PID_current_q.P = value;
        break;
      case CAN_MESSAGE::CONTROLLER::I_GAIN:
        motor_->PID_current_q.I = value;
        break;
      case CAN_MESSAGE::CONTROLLER::D_GAIN:
        motor_->PID_current_q.D = value;
        break;
      default:
        return;
      }
      break;
    case CAN_MESSAGE::SET_D_PID_GAIN:
      switch (index)
      {
      case CAN_MESSAGE::CONTROLLER::P_GAIN:
        motor_->PID_current_d.P = value;
        break;
      case CAN_MESSAGE::CONTROLLER::I_GAIN:
        motor_->PID_current_d.I = value;
        break;
      case CAN_MESSAGE::CONTROLLER::D_GAIN:
        motor_->PID_current_d.D = value;
        break;
      default:
        return;
      }
      break;
    case CAN_MESSAGE::SET_V_PID_GAIN:
      switch (index)
      {
      case CAN_MESSAGE::CONTROLLER::P_GAIN:
        motor_->PID_velocity.P = value;
        break;
      case CAN_MESSAGE::CONTROLLER::I_GAIN:
        motor_->PID_velocity.I = value;
        break;
      case CAN_MESSAGE::CONTROLLER::D_GAIN:
        motor_->PID_velocity.D = value;
        break;
      default:
        return;
      }
      break;

    case CAN_MESSAGE::SET_A_PID_GAIN:
      switch (index)
      {
      case CAN_MESSAGE::CONTROLLER::P_GAIN:
        motor_->P_angle.P = value;
        break;
      case CAN_MESSAGE::CONTROLLER::I_GAIN:
        motor_->P_angle.I = value;
        break;
      case CAN_MESSAGE::CONTROLLER::D_GAIN:
        motor_->P_angle.D = value;
        break;
      }
      break;

    default:
      return;
    }
  }

  void setTarget(const float value)
  {
    motor_->target = value;
  }
  void setControlMode(const int mode)
  {
    motor_->controller = (MotionControlType)mode;
  }
  uint8_t *getPosition()
  {
    Serial.printf("Sending: %f rad\n\r", motor_->shaft_angle);
    Serial.println();
    float angle = motor_->shaft_angle;
    constructMessage(CAN_MESSAGE::GET_POSITION, angle);
    return (uint8_t *)&message_;
  };

  uint8_t *getControlMode()
  {
    Serial.print("Sending control mode");
    int mode = motor_->controller;
    Serial.println(mode);
    constructMessage(CAN_MESSAGE::GET_CONTROL_MODE, mode);
    return (uint8_t *)&message_;
  };
  uint8_t *getPID(char command, byte index)
  {
  }

  void constructMessage(const byte cmd, int data)
  {
    uint8_t *bytes = reinterpret_cast<uint8_t *>(&data);

    message_[0] = cmd;
    message_[1] = 0;
    message_[2] = 0;
    message_[3] = 0;
    message_[4] = bytes[0];
    message_[5] = bytes[1];
    message_[6] = bytes[2];
    message_[7] = bytes[3];
  }
  void constructMessage(const byte cmd, float data)
  {
    uint8_t *bytes = reinterpret_cast<uint8_t *>(&data);

    message_[0] = cmd;
    message_[1] = 0;
    message_[2] = 0;
    message_[3] = 0;
    message_[4] = bytes[0];
    message_[5] = bytes[1];
    message_[6] = bytes[2];
    message_[7] = bytes[3];
  }

  byte lastMessageType;
  float position = 0.0;
  float torque = 0.0;

  Commander *command_;
  BLDCMotor *motor_;
  CANHandler *CANDevice_;
  byte ID_;
  byte message_[8];
};