#ifndef _ROS_portal_portal_sensors_h
#define _ROS_portal_portal_sensors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace portal
{

  class portal_sensors : public ros::Msg
  {
    public:
      typedef bool _x_XendSwitchPositive_type;
      _x_XendSwitchPositive_type x_XendSwitchPositive;
      typedef bool _x_XendSwitchNegative_type;
      _x_XendSwitchNegative_type x_XendSwitchNegative;
      typedef bool _x_YendSwitchPositive_type;
      _x_YendSwitchPositive_type x_YendSwitchPositive;
      typedef bool _x_YendSwitchNegative_type;
      _x_YendSwitchNegative_type x_YendSwitchNegative;
      typedef bool _x_ZendSwitchUp_type;
      _x_ZendSwitchUp_type x_ZendSwitchUp;
      typedef bool _x_ZendSwitchDown_type;
      _x_ZendSwitchDown_type x_ZendSwitchDown;

    portal_sensors():
      x_XendSwitchPositive(0),
      x_XendSwitchNegative(0),
      x_YendSwitchPositive(0),
      x_YendSwitchNegative(0),
      x_ZendSwitchUp(0),
      x_ZendSwitchDown(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_XendSwitchPositive;
      u_x_XendSwitchPositive.real = this->x_XendSwitchPositive;
      *(outbuffer + offset + 0) = (u_x_XendSwitchPositive.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_XendSwitchPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_XendSwitchNegative;
      u_x_XendSwitchNegative.real = this->x_XendSwitchNegative;
      *(outbuffer + offset + 0) = (u_x_XendSwitchNegative.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_XendSwitchNegative);
      union {
        bool real;
        uint8_t base;
      } u_x_YendSwitchPositive;
      u_x_YendSwitchPositive.real = this->x_YendSwitchPositive;
      *(outbuffer + offset + 0) = (u_x_YendSwitchPositive.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_YendSwitchPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_YendSwitchNegative;
      u_x_YendSwitchNegative.real = this->x_YendSwitchNegative;
      *(outbuffer + offset + 0) = (u_x_YendSwitchNegative.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_YendSwitchNegative);
      union {
        bool real;
        uint8_t base;
      } u_x_ZendSwitchUp;
      u_x_ZendSwitchUp.real = this->x_ZendSwitchUp;
      *(outbuffer + offset + 0) = (u_x_ZendSwitchUp.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_ZendSwitchUp);
      union {
        bool real;
        uint8_t base;
      } u_x_ZendSwitchDown;
      u_x_ZendSwitchDown.real = this->x_ZendSwitchDown;
      *(outbuffer + offset + 0) = (u_x_ZendSwitchDown.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_ZendSwitchDown);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_XendSwitchPositive;
      u_x_XendSwitchPositive.base = 0;
      u_x_XendSwitchPositive.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_XendSwitchPositive = u_x_XendSwitchPositive.real;
      offset += sizeof(this->x_XendSwitchPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_XendSwitchNegative;
      u_x_XendSwitchNegative.base = 0;
      u_x_XendSwitchNegative.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_XendSwitchNegative = u_x_XendSwitchNegative.real;
      offset += sizeof(this->x_XendSwitchNegative);
      union {
        bool real;
        uint8_t base;
      } u_x_YendSwitchPositive;
      u_x_YendSwitchPositive.base = 0;
      u_x_YendSwitchPositive.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_YendSwitchPositive = u_x_YendSwitchPositive.real;
      offset += sizeof(this->x_YendSwitchPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_YendSwitchNegative;
      u_x_YendSwitchNegative.base = 0;
      u_x_YendSwitchNegative.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_YendSwitchNegative = u_x_YendSwitchNegative.real;
      offset += sizeof(this->x_YendSwitchNegative);
      union {
        bool real;
        uint8_t base;
      } u_x_ZendSwitchUp;
      u_x_ZendSwitchUp.base = 0;
      u_x_ZendSwitchUp.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_ZendSwitchUp = u_x_ZendSwitchUp.real;
      offset += sizeof(this->x_ZendSwitchUp);
      union {
        bool real;
        uint8_t base;
      } u_x_ZendSwitchDown;
      u_x_ZendSwitchDown.base = 0;
      u_x_ZendSwitchDown.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_ZendSwitchDown = u_x_ZendSwitchDown.real;
      offset += sizeof(this->x_ZendSwitchDown);
     return offset;
    }

    virtual const char * getType() override { return "portal/portal_sensors"; };
    virtual const char * getMD5() override { return "9a2a78ffba2f52576ed050509d8f5c1e"; };

  };

}
#endif
