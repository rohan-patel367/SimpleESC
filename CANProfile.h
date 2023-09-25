#include <SimpleCAN.h>

// Message IDs
namespace CAN_MESSAGE
{
    const byte NONE = 0x00;
    const byte SET_TARGET = 0x01;
    const byte SET_POSITION = 0x02;
    const byte SET_VELOCITY = 0x03;
    const byte SET_Q_PID_GAIN = 0x04;
    const byte SET_D_PID_GAIN = 0x05;
    const byte SET_V_PID_GAIN = 0x06;
    const byte SET_A_PID_GAIN = 0x07;
    const byte SET_CONTROL_MODE = 0x08;
    const byte SET_LIMIT = 0x09;
    const byte SET_TORQUE_TYPE = 0x0A;
    const byte SET_ENABLE = 0x0B;
    const byte GET_POSITION = 0x80;
    const byte GET_VELOCITY = 0x81;
    const byte GET_Q_PID_GAIN = 0x82;
    const byte GET_D_PID_GAIN = 0x83;
    const byte GET_V_PID_GAIN = 0x84;
    const byte GET_A_PID_GAIN = 0x85;
    const byte GET_CONTROL_MODE = 0x86;
    const byte GET_LIMIT = 0x87;
    const byte GET_TORQUE_TYPE = 0x88;
    const byte GET_ENABLE = 0x89;

    namespace CONTROLLER
    {
        const byte P_GAIN = 0x01;
        const byte I_GAIN = 0x02;
        const byte D_GAIN = 0x03;
        const byte SATURATION_LIMIT = 0x04;
        const byte RAMP_PARAMETER = 0x05;
        const byte LOWPASS_TC = 0x06;
    }

    namespace CONTROL_MODE
    {
        const byte TORQUE_CONTROL = 0;
        const byte VELOCITY_CONTROL = 1;
        const byte POSITION_CONTROL = 2;
        const byte VELOCITY_OPENLOOP_CONTROL = 3;
        const byte POSITION_OPENLOOP_CONTROL = 4;
    }
    namespace TORQUE_TYPE
    {
        const byte VOLTAGE = 0x01;
        const byte DC_CURRENT = 0x02;
        const byte FOC_CURRENT = 0x03;
    }
    namespace LIMIT
    {
        const byte CURRENT_LIMIT = 0x01;
        const byte VOLTAGE_LIMIT = 0x02;
        const byte VELOCITY_LIMIT = 0x03;
    }

    namespace STATUS
    {
        const byte ENABLE = 0x01;
        const byte DISABLE = 0x02;
    }
}
#define CAN_MAKE_ID(Device, Message) (((Device) << 8) | (Message))
#define CAN_GET_MESSAGE(CanID) (CanID & 0xff)
#define CAN_GET_DEVICE_ID(CanID) (CanID >> 8)
#define MAX_STRLEN 16

class CANNotifications
{
public:
    virtual void setTarget(float value) = 0;
    virtual void setControlMode(int mode) = 0;
    virtual uint8_t *getControlMode() = 0;
    virtual void setPID(byte command, byte index, float value) = 0;
    virtual uint8_t *getPID(char command, byte index) = 0;
    virtual void setLimit(byte index, float value) = 0;
    virtual void setTorqueType(byte index) = 0;
    virtual uint8_t *getPosition() = 0;
    virtual void setEnable(byte index) = 0;
    //virtual void getParam(byte index) = 0;
};

class CANHandler : public SimpleCANProfile
{
public:
    CANHandler(SimpleCan *pCan, CANNotifications *_pBroker, byte ID) : SimpleCANProfile(pCan)
    {
        pBroker = _pBroker;
        ID_ = ID;
    }

    void HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData)
    {
        digitalToggle(LED_BUILTIN);

        char str[MAX_STRLEN];
        float value = 0;
        int val = 0;
        if (rxHeader.Identifier == ID_ || rxHeader.Identifier == broadcastID_)
        {
            switch (*rxData)
            {
            case CAN_MESSAGE::GET_POSITION:
                pMsg = pBroker->getPosition();
                Can1->SendMessage(pMsg, msgLength_, returnID_ + ID_);
                break;
            case CAN_MESSAGE::SET_Q_PID_GAIN:
                value = CANGetFloat(rxData + 4);
                pBroker->setPID(rxData[0], rxData[1], value);
                break;
            case CAN_MESSAGE::SET_D_PID_GAIN:
                value = CANGetFloat(rxData + 4);
                pBroker->setPID(rxData[0], rxData[1], value);
            case CAN_MESSAGE::SET_V_PID_GAIN:
                value = CANGetFloat(rxData + 4);
                pBroker->setPID(rxData[0], rxData[1], value);
            case CAN_MESSAGE::SET_A_PID_GAIN:
                value = CANGetFloat(rxData + 4);
                pBroker->setPID(rxData[0], rxData[1], value);
            case CAN_MESSAGE::SET_TARGET:
                value = CANGetFloat(rxData + 4);
                pBroker->setTarget(value);
                break;
            case CAN_MESSAGE::SET_CONTROL_MODE:
                value = CANGetInt(rxData + 4);
                pBroker->setControlMode(value);
                break;
            case CAN_MESSAGE::GET_CONTROL_MODE:
                pMsg = pBroker->getControlMode();
                Can1->SendMessage(pMsg, msgLength_, returnID_ + ID_);
                break;
            case CAN_MESSAGE::SET_LIMIT:
                value = CANGetFloat(rxData + 4);
                pBroker->setLimit(rxData[1], value);
                break;
            case CAN_MESSAGE::SET_TORQUE_TYPE:
                pBroker->setTorqueType(rxData[1]);
                break;
            case CAN_MESSAGE::SET_ENABLE:
                pBroker->setEnable(rxData[1]);
                break;

            default:
                Serial.printf("y:0x%x DLC=0x%x ", rxHeader.Identifier, rxHeader.DataLength);
            }
        }
    }

private:
    CANNotifications *pBroker;
    byte ID_;
    uint8_t *pMsg;
    int msgLength_ = 8;
    int returnID_ = 0x240;
    int broadcastID_ = 0x280;
};