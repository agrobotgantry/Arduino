#ifndef _ROS_portal_portal_feedback_h
#define _ROS_portal_portal_feedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace portal
{

  class portal_feedback : public ros::Msg
  {
    public:
      typedef bool _x_XgoingPositive_type;
      _x_XgoingPositive_type x_XgoingPositive;
      typedef bool _x_XgoingNegative_type;
      _x_XgoingNegative_type x_XgoingNegative;
      typedef bool _x_YgoingPositive_type;
      _x_YgoingPositive_type x_YgoingPositive;
      typedef bool _x_YgoingNegative_type;
      _x_YgoingNegative_type x_YgoingNegative;
      typedef bool _x_ZgoingUp_type;
      _x_ZgoingUp_type x_ZgoingUp;
      typedef bool _x_ZgoingDown_type;
      _x_ZgoingDown_type x_ZgoingDown;
      typedef bool _x_isHomePosition_type;
      _x_isHomePosition_type x_isHomePosition;
      typedef bool _x_speedFastActive_type;
      _x_speedFastActive_type x_speedFastActive;

    portal_feedback():
      x_XgoingPositive(0),
      x_XgoingNegative(0),
      x_YgoingPositive(0),
      x_YgoingNegative(0),
      x_ZgoingUp(0),
      x_ZgoingDown(0),
      x_isHomePosition(0),
      x_speedFastActive(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_XgoingPositive;
      u_x_XgoingPositive.real = this->x_XgoingPositive;
      *(outbuffer + offset + 0) = (u_x_XgoingPositive.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_XgoingPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_XgoingNegative;
      u_x_XgoingNegative.real = this->x_XgoingNegative;
      *(outbuffer + offset + 0) = (u_x_XgoingNegative.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_XgoingNegative);
      union {
        bool real;
        uint8_t base;
      } u_x_YgoingPositive;
      u_x_YgoingPositive.real = this->x_YgoingPositive;
      *(outbuffer + offset + 0) = (u_x_YgoingPositive.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_YgoingPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_YgoingNegative;
      u_x_YgoingNegative.real = this->x_YgoingNegative;
      *(outbuffer + offset + 0) = (u_x_YgoingNegative.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_YgoingNegative);
      union {
        bool real;
        uint8_t base;
      } u_x_ZgoingUp;
      u_x_ZgoingUp.real = this->x_ZgoingUp;
      *(outbuffer + offset + 0) = (u_x_ZgoingUp.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_ZgoingUp);
      union {
        bool real;
        uint8_t base;
      } u_x_ZgoingDown;
      u_x_ZgoingDown.real = this->x_ZgoingDown;
      *(outbuffer + offset + 0) = (u_x_ZgoingDown.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_ZgoingDown);
      union {
        bool real;
        uint8_t base;
      } u_x_isHomePosition;
      u_x_isHomePosition.real = this->x_isHomePosition;
      *(outbuffer + offset + 0) = (u_x_isHomePosition.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_isHomePosition);
      union {
        bool real;
        uint8_t base;
      } u_x_speedFastActive;
      u_x_speedFastActive.real = this->x_speedFastActive;
      *(outbuffer + offset + 0) = (u_x_speedFastActive.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_speedFastActive);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_XgoingPositive;
      u_x_XgoingPositive.base = 0;
      u_x_XgoingPositive.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_XgoingPositive = u_x_XgoingPositive.real;
      offset += sizeof(this->x_XgoingPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_XgoingNegative;
      u_x_XgoingNegative.base = 0;
      u_x_XgoingNegative.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_XgoingNegative = u_x_XgoingNegative.real;
      offset += sizeof(this->x_XgoingNegative);
      union {
        bool real;
        uint8_t base;
      } u_x_YgoingPositive;
      u_x_YgoingPositive.base = 0;
      u_x_YgoingPositive.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_YgoingPositive = u_x_YgoingPositive.real;
      offset += sizeof(this->x_YgoingPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_YgoingNegative;
      u_x_YgoingNegative.base = 0;
      u_x_YgoingNegative.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_YgoingNegative = u_x_YgoingNegative.real;
      offset += sizeof(this->x_YgoingNegative);
      union {
        bool real;
        uint8_t base;
      } u_x_ZgoingUp;
      u_x_ZgoingUp.base = 0;
      u_x_ZgoingUp.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_ZgoingUp = u_x_ZgoingUp.real;
      offset += sizeof(this->x_ZgoingUp);
      union {
        bool real;
        uint8_t base;
      } u_x_ZgoingDown;
      u_x_ZgoingDown.base = 0;
      u_x_ZgoingDown.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_ZgoingDown = u_x_ZgoingDown.real;
      offset += sizeof(this->x_ZgoingDown);
      union {
        bool real;
        uint8_t base;
      } u_x_isHomePosition;
      u_x_isHomePosition.base = 0;
      u_x_isHomePosition.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_isHomePosition = u_x_isHomePosition.real;
      offset += sizeof(this->x_isHomePosition);
      union {
        bool real;
        uint8_t base;
      } u_x_speedFastActive;
      u_x_speedFastActive.base = 0;
      u_x_speedFastActive.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_speedFastActive = u_x_speedFastActive.real;
      offset += sizeof(this->x_speedFastActive);
     return offset;
    }

    virtual const char * getType() override { return "portal/portal_feedback"; };
    virtual const char * getMD5() override { return "519e7560728eba6bea7f94c7bd288b1e"; };

  };

}
#endif
