#ifndef _ROS_gripper_gripper_output_h
#define _ROS_gripper_gripper_output_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gripper
{

  class gripper_output : public ros::Msg
  {
    public:
      typedef bool _x_gripperOpen_type;
      _x_gripperOpen_type x_gripperOpen;
      typedef bool _x_gripperClose_type;
      _x_gripperClose_type x_gripperClose;

    gripper_output():
      x_gripperOpen(0),
      x_gripperClose(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_gripperOpen;
      u_x_gripperOpen.real = this->x_gripperOpen;
      *(outbuffer + offset + 0) = (u_x_gripperOpen.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_gripperOpen);
      union {
        bool real;
        uint8_t base;
      } u_x_gripperClose;
      u_x_gripperClose.real = this->x_gripperClose;
      *(outbuffer + offset + 0) = (u_x_gripperClose.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_gripperClose);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_gripperOpen;
      u_x_gripperOpen.base = 0;
      u_x_gripperOpen.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_gripperOpen = u_x_gripperOpen.real;
      offset += sizeof(this->x_gripperOpen);
      union {
        bool real;
        uint8_t base;
      } u_x_gripperClose;
      u_x_gripperClose.base = 0;
      u_x_gripperClose.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_gripperClose = u_x_gripperClose.real;
      offset += sizeof(this->x_gripperClose);
     return offset;
    }

    virtual const char * getType() override { return "gripper/gripper_output"; };
    virtual const char * getMD5() override { return "01d7d6e5032306a378ca03e8151b2ce8"; };

  };

}
#endif
