#ifndef _ROS_position_crop_crop_position_h
#define _ROS_position_crop_crop_position_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace position_crop
{

  class crop_position : public ros::Msg
  {
    public:
      typedef bool _x_XgoNegative_type;
      _x_XgoNegative_type x_XgoNegative;
      typedef bool _x_XgoPositive_type;
      _x_XgoPositive_type x_XgoPositive;
      typedef bool _x_XisRight_type;
      _x_XisRight_type x_XisRight;
      typedef bool _x_YgoPositive_type;
      _x_YgoPositive_type x_YgoPositive;
      typedef bool _x_YgoNegative_type;
      _x_YgoNegative_type x_YgoNegative;
      typedef bool _x_YisRight_type;
      _x_YisRight_type x_YisRight;

    crop_position():
      x_XgoNegative(0),
      x_XgoPositive(0),
      x_XisRight(0),
      x_YgoPositive(0),
      x_YgoNegative(0),
      x_YisRight(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
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
      } u_x_XgoPositive;
      u_x_XgoPositive.real = this->x_XgoPositive;
      *(outbuffer + offset + 0) = (u_x_XgoPositive.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_XgoPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_XisRight;
      u_x_XisRight.real = this->x_XisRight;
      *(outbuffer + offset + 0) = (u_x_XisRight.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_XisRight);
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
      } u_x_YisRight;
      u_x_YisRight.real = this->x_YisRight;
      *(outbuffer + offset + 0) = (u_x_YisRight.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->x_YisRight);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
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
      } u_x_XgoPositive;
      u_x_XgoPositive.base = 0;
      u_x_XgoPositive.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_XgoPositive = u_x_XgoPositive.real;
      offset += sizeof(this->x_XgoPositive);
      union {
        bool real;
        uint8_t base;
      } u_x_XisRight;
      u_x_XisRight.base = 0;
      u_x_XisRight.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_XisRight = u_x_XisRight.real;
      offset += sizeof(this->x_XisRight);
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
      } u_x_YisRight;
      u_x_YisRight.base = 0;
      u_x_YisRight.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->x_YisRight = u_x_YisRight.real;
      offset += sizeof(this->x_YisRight);
     return offset;
    }

    virtual const char * getType() override { return "position_crop/crop_position"; };
    virtual const char * getMD5() override { return "4916c9c2ed4a6563a1a081ed53993f8a"; };

  };

}
#endif
