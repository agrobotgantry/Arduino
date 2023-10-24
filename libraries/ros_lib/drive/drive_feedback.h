#ifndef _ROS_drive_drive_feedback_h
#define _ROS_drive_drive_feedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace drive
{

  class drive_feedback : public ros::Msg
  {
    public:
      typedef bool _x_movingForward_type;
      _x_movingForward_type x_movingForward;
      typedef bool _x_movingBackwards_type;
      _x_movingBackwards_type x_movingBackwards;
      typedef bool _x_movingLeft_type;
      _x_movingLeft_type x_movingLeft;
      typedef bool _x_movingRight_type;
      _x_movingRight_type x_movingRight;
      typedef uint8_t _i_currentSpeed_type;
      _i_currentSpeed_type i_currentSpeed;

    drive_feedback():
      x_movingForward(0),
      x_movingBackwards(0),
      x_movingLeft(0),
      x_movingRight(0),
      i_currentSpeed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_movingForward;
      u_x_movingForward.real = this->x_movingForward;
      *(outbuffer + offset + 0) = (u_x_movingForward.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_movingForward);
      union {
        bool real;
        uint8_t base;
      } u_x_movingBackwards;
      u_x_movingBackwards.real = this->x_movingBackwards;
      *(outbuffer + offset + 0) = (u_x_movingBackwards.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_movingBackwards);
      union {
        bool real;
        uint8_t base;
      } u_x_movingLeft;
      u_x_movingLeft.real = this->x_movingLeft;
      *(outbuffer + offset + 0) = (u_x_movingLeft.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_movingLeft);
      union {
        bool real;
        uint8_t base;
      } u_x_movingRight;
      u_x_movingRight.real = this->x_movingRight;
      *(outbuffer + offset + 0) = (u_x_movingRight.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_movingRight);
      *(outbuffer + offset + 0) = (this->i_currentSpeed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->i_currentSpeed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_movingForward;
      u_x_movingForward.base = 0;
      u_x_movingForward.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_movingForward = u_x_movingForward.real;
      offset += sizeof(this->x_movingForward);
      union {
        bool real;
        uint8_t base;
      } u_x_movingBackwards;
      u_x_movingBackwards.base = 0;
      u_x_movingBackwards.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_movingBackwards = u_x_movingBackwards.real;
      offset += sizeof(this->x_movingBackwards);
      union {
        bool real;
        uint8_t base;
      } u_x_movingLeft;
      u_x_movingLeft.base = 0;
      u_x_movingLeft.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_movingLeft = u_x_movingLeft.real;
      offset += sizeof(this->x_movingLeft);
      union {
        bool real;
        uint8_t base;
      } u_x_movingRight;
      u_x_movingRight.base = 0;
      u_x_movingRight.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_movingRight = u_x_movingRight.real;
      offset += sizeof(this->x_movingRight);
      this->i_currentSpeed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->i_currentSpeed);
     return offset;
    }

    virtual const char * getType() override { return "drive/drive_feedback"; };
    virtual const char * getMD5() override { return "e61e2c8b32d2c2d45e0f8686410ed0ec"; };

  };

}
#endif
