#ifndef _ROS_gripper_gripper_feedback_h
#define _ROS_gripper_gripper_feedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gripper
{

  class gripper_feedback : public ros::Msg
  {
    public:
      typedef bool _x_gripperOpened_type;
      _x_gripperOpened_type x_gripperOpened;
      typedef bool _x_gripperClosed_type;
      _x_gripperClosed_type x_gripperClosed;

    gripper_feedback():
      x_gripperOpened(0),
      x_gripperClosed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_gripperOpened;
      u_x_gripperOpened.real = this->x_gripperOpened;
      *(outbuffer + offset + 0) = (u_x_gripperOpened.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_gripperOpened);
      union {
        bool real;
        uint8_t base;
      } u_x_gripperClosed;
      u_x_gripperClosed.real = this->x_gripperClosed;
      *(outbuffer + offset + 0) = (u_x_gripperClosed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_gripperClosed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_gripperOpened;
      u_x_gripperOpened.base = 0;
      u_x_gripperOpened.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_gripperOpened = u_x_gripperOpened.real;
      offset += sizeof(this->x_gripperOpened);
      union {
        bool real;
        uint8_t base;
      } u_x_gripperClosed;
      u_x_gripperClosed.base = 0;
      u_x_gripperClosed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_gripperClosed = u_x_gripperClosed.real;
      offset += sizeof(this->x_gripperClosed);
     return offset;
    }

    virtual const char * getType() override { return "gripper/gripper_feedback"; };
    virtual const char * getMD5() override { return "10ef4524c174cc37b311236c93910512"; };

  };

}
#endif
