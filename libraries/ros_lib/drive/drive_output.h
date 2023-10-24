#ifndef _ROS_drive_drive_output_h
#define _ROS_drive_drive_output_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace drive
{

  class drive_output : public ros::Msg
  {
    public:
      typedef bool _x_moveForward_type;
      _x_moveForward_type x_moveForward;
      typedef bool _x_moveBackwards_type;
      _x_moveBackwards_type x_moveBackwards;
      typedef bool _x_moveLeft_type;
      _x_moveLeft_type x_moveLeft;
      typedef bool _x_moveRight_type;
      _x_moveRight_type x_moveRight;
      typedef uint8_t _i_targetSpeed_type;
      _i_targetSpeed_type i_targetSpeed;

    drive_output():
      x_moveForward(0),
      x_moveBackwards(0),
      x_moveLeft(0),
      x_moveRight(0),
      i_targetSpeed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_moveForward;
      u_x_moveForward.real = this->x_moveForward;
      *(outbuffer + offset + 0) = (u_x_moveForward.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_moveForward);
      union {
        bool real;
        uint8_t base;
      } u_x_moveBackwards;
      u_x_moveBackwards.real = this->x_moveBackwards;
      *(outbuffer + offset + 0) = (u_x_moveBackwards.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_moveBackwards);
      union {
        bool real;
        uint8_t base;
      } u_x_moveLeft;
      u_x_moveLeft.real = this->x_moveLeft;
      *(outbuffer + offset + 0) = (u_x_moveLeft.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_moveLeft);
      union {
        bool real;
        uint8_t base;
      } u_x_moveRight;
      u_x_moveRight.real = this->x_moveRight;
      *(outbuffer + offset + 0) = (u_x_moveRight.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_moveRight);
      *(outbuffer + offset + 0) = (this->i_targetSpeed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->i_targetSpeed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_moveForward;
      u_x_moveForward.base = 0;
      u_x_moveForward.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_moveForward = u_x_moveForward.real;
      offset += sizeof(this->x_moveForward);
      union {
        bool real;
        uint8_t base;
      } u_x_moveBackwards;
      u_x_moveBackwards.base = 0;
      u_x_moveBackwards.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_moveBackwards = u_x_moveBackwards.real;
      offset += sizeof(this->x_moveBackwards);
      union {
        bool real;
        uint8_t base;
      } u_x_moveLeft;
      u_x_moveLeft.base = 0;
      u_x_moveLeft.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_moveLeft = u_x_moveLeft.real;
      offset += sizeof(this->x_moveLeft);
      union {
        bool real;
        uint8_t base;
      } u_x_moveRight;
      u_x_moveRight.base = 0;
      u_x_moveRight.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_moveRight = u_x_moveRight.real;
      offset += sizeof(this->x_moveRight);
      this->i_targetSpeed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->i_targetSpeed);
     return offset;
    }

    virtual const char * getType() override { return "drive/drive_output"; };
    virtual const char * getMD5() override { return "549a80154df6f05e6e5830bddbf998a0"; };

  };

}
#endif
