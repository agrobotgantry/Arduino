#ifndef _ROS_drive_drive_sensors_h
#define _ROS_drive_drive_sensors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace drive
{

  class drive_sensors : public ros::Msg
  {
    public:
      typedef uint8_t _i_distanceSensorFrontLeft_type;
      _i_distanceSensorFrontLeft_type i_distanceSensorFrontLeft;
      typedef uint8_t _i_distanceSensorFrontRight_type;
      _i_distanceSensorFrontRight_type i_distanceSensorFrontRight;
      typedef uint8_t _i_distanceSensorBackLeft_type;
      _i_distanceSensorBackLeft_type i_distanceSensorBackLeft;
      typedef uint8_t _i_distanceSensorBackRight_type;
      _i_distanceSensorBackRight_type i_distanceSensorBackRight;

    drive_sensors():
      i_distanceSensorFrontLeft(0),
      i_distanceSensorFrontRight(0),
      i_distanceSensorBackLeft(0),
      i_distanceSensorBackRight(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->i_distanceSensorFrontLeft >> (8 * 0)) & 0xFF;
      offset += sizeof(this->i_distanceSensorFrontLeft);
      *(outbuffer + offset + 0) = (this->i_distanceSensorFrontRight >> (8 * 0)) & 0xFF;
      offset += sizeof(this->i_distanceSensorFrontRight);
      *(outbuffer + offset + 0) = (this->i_distanceSensorBackLeft >> (8 * 0)) & 0xFF;
      offset += sizeof(this->i_distanceSensorBackLeft);
      *(outbuffer + offset + 0) = (this->i_distanceSensorBackRight >> (8 * 0)) & 0xFF;
      offset += sizeof(this->i_distanceSensorBackRight);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->i_distanceSensorFrontLeft =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->i_distanceSensorFrontLeft);
      this->i_distanceSensorFrontRight =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->i_distanceSensorFrontRight);
      this->i_distanceSensorBackLeft =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->i_distanceSensorBackLeft);
      this->i_distanceSensorBackRight =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->i_distanceSensorBackRight);
     return offset;
    }

    virtual const char * getType() override { return "drive/drive_sensors"; };
    virtual const char * getMD5() override { return "6cac09db7aa34afe3474dddecc79504a"; };

  };

}
#endif
