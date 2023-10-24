#ifndef _ROS_portal_portal_output_h
#define _ROS_portal_portal_output_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace portal
{

  class portal_output : public ros::Msg
  {
    public:
      typedef bool _x_XgoPositive_type;
      _x_XgoPositive_type x_XgoPositive;
      typedef bool _x_XgoNegative_type;
      _x_XgoNegative_type x_XgoNegative;
      typedef bool _x_YgoPositive_type;
      _x_YgoPositive_type x_YgoPositive;
      typedef bool _x_YgoNegative_type;
      _x_YgoNegative_type x_YgoNegative;
      typedef bool _x_ZgoUp_type;
      _x_ZgoUp_type x_ZgoUp;
      typedef bool _x_ZgoDown_type;
      _x_ZgoDown_type x_ZgoDown;
      typedef bool _x_goHomePosition_type;
      _x_goHomePosition_type x_goHomePosition;
      typedef bool _x_speedFast_type;
      _x_speedFast_type x_speedFast;

    portal_output():
      x_XgoPositive(0),
      x_XgoNegative(0),
      x_YgoPositive(0),
      x_YgoNegative(0),
      x_ZgoUp(0),
      x_ZgoDown(0),
      x_goHomePosition(0),
      x_speedFast(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_XgoPositive;
      u_x_XgoPositive.real = this->x_XgoPositive;
      *(outbuffer + offset + 0) = (u_x_XgoPositive.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_XgoPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_XgoNegative;
      u_x_XgoNegative.real = this->x_XgoNegative;
      *(outbuffer + offset + 0) = (u_x_XgoNegative.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_XgoNegative);
      union {
        bool real;
        uint8_t base;
      } u_x_YgoPositive;
      u_x_YgoPositive.real = this->x_YgoPositive;
      *(outbuffer + offset + 0) = (u_x_YgoPositive.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_YgoPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_YgoNegative;
      u_x_YgoNegative.real = this->x_YgoNegative;
      *(outbuffer + offset + 0) = (u_x_YgoNegative.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_YgoNegative);
      union {
        bool real;
        uint8_t base;
      } u_x_ZgoUp;
      u_x_ZgoUp.real = this->x_ZgoUp;
      *(outbuffer + offset + 0) = (u_x_ZgoUp.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_ZgoUp);
      union {
        bool real;
        uint8_t base;
      } u_x_ZgoDown;
      u_x_ZgoDown.real = this->x_ZgoDown;
      *(outbuffer + offset + 0) = (u_x_ZgoDown.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_ZgoDown);
      union {
        bool real;
        uint8_t base;
      } u_x_goHomePosition;
      u_x_goHomePosition.real = this->x_goHomePosition;
      *(outbuffer + offset + 0) = (u_x_goHomePosition.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_goHomePosition);
      union {
        bool real;
        uint8_t base;
      } u_x_speedFast;
      u_x_speedFast.real = this->x_speedFast;
      *(outbuffer + offset + 0) = (u_x_speedFast.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_speedFast);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_x_XgoPositive;
      u_x_XgoPositive.base = 0;
      u_x_XgoPositive.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_XgoPositive = u_x_XgoPositive.real;
      offset += sizeof(this->x_XgoPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_XgoNegative;
      u_x_XgoNegative.base = 0;
      u_x_XgoNegative.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_XgoNegative = u_x_XgoNegative.real;
      offset += sizeof(this->x_XgoNegative);
      union {
        bool real;
        uint8_t base;
      } u_x_YgoPositive;
      u_x_YgoPositive.base = 0;
      u_x_YgoPositive.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_YgoPositive = u_x_YgoPositive.real;
      offset += sizeof(this->x_YgoPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_YgoNegative;
      u_x_YgoNegative.base = 0;
      u_x_YgoNegative.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_YgoNegative = u_x_YgoNegative.real;
      offset += sizeof(this->x_YgoNegative);
      union {
        bool real;
        uint8_t base;
      } u_x_ZgoUp;
      u_x_ZgoUp.base = 0;
      u_x_ZgoUp.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_ZgoUp = u_x_ZgoUp.real;
      offset += sizeof(this->x_ZgoUp);
      union {
        bool real;
        uint8_t base;
      } u_x_ZgoDown;
      u_x_ZgoDown.base = 0;
      u_x_ZgoDown.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_ZgoDown = u_x_ZgoDown.real;
      offset += sizeof(this->x_ZgoDown);
      union {
        bool real;
        uint8_t base;
      } u_x_goHomePosition;
      u_x_goHomePosition.base = 0;
      u_x_goHomePosition.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_goHomePosition = u_x_goHomePosition.real;
      offset += sizeof(this->x_goHomePosition);
      union {
        bool real;
        uint8_t base;
      } u_x_speedFast;
      u_x_speedFast.base = 0;
      u_x_speedFast.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_speedFast = u_x_speedFast.real;
      offset += sizeof(this->x_speedFast);
     return offset;
    }

    virtual const char * getType() override { return "portal/portal_output"; };
    virtual const char * getMD5() override { return "b4f646cc7ab1bb08920901e87f63d3fa"; };

  };

}
#endif
