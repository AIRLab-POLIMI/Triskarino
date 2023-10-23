#ifndef _ROS_triskarino_msgs_touchdata_h
#define _ROS_triskarino_msgs_touchdata_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace triskarino_msgs
{

  class touchdata : public ros::Msg
  {
    public:
      typedef float _roll_value_type;
      _roll_value_type roll_value;
      typedef float _pitch_value_type;
      _pitch_value_type pitch_value;
      typedef float _flex_value_type;
      _flex_value_type flex_value;
      typedef float _pressure_value_type;
      _pressure_value_type pressure_value;
      typedef ros::Time _timestamp_type;
      _timestamp_type timestamp;

    touchdata():
      roll_value(0),
      pitch_value(0),
      flex_value(0),
      pressure_value(0),
      timestamp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_roll_value;
      u_roll_value.real = this->roll_value;
      *(outbuffer + offset + 0) = (u_roll_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_roll_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_roll_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_roll_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roll_value);
      union {
        float real;
        uint32_t base;
      } u_pitch_value;
      u_pitch_value.real = this->pitch_value;
      *(outbuffer + offset + 0) = (u_pitch_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pitch_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pitch_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pitch_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pitch_value);
      union {
        float real;
        uint32_t base;
      } u_flex_value;
      u_flex_value.real = this->flex_value;
      *(outbuffer + offset + 0) = (u_flex_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_flex_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_flex_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_flex_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->flex_value);
      union {
        float real;
        uint32_t base;
      } u_pressure_value;
      u_pressure_value.real = this->pressure_value;
      *(outbuffer + offset + 0) = (u_pressure_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pressure_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pressure_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pressure_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pressure_value);
      *(outbuffer + offset + 0) = (this->timestamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.sec);
      *(outbuffer + offset + 0) = (this->timestamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_roll_value;
      u_roll_value.base = 0;
      u_roll_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_roll_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_roll_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_roll_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->roll_value = u_roll_value.real;
      offset += sizeof(this->roll_value);
      union {
        float real;
        uint32_t base;
      } u_pitch_value;
      u_pitch_value.base = 0;
      u_pitch_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pitch_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pitch_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pitch_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pitch_value = u_pitch_value.real;
      offset += sizeof(this->pitch_value);
      union {
        float real;
        uint32_t base;
      } u_flex_value;
      u_flex_value.base = 0;
      u_flex_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_flex_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_flex_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_flex_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->flex_value = u_flex_value.real;
      offset += sizeof(this->flex_value);
      union {
        float real;
        uint32_t base;
      } u_pressure_value;
      u_pressure_value.base = 0;
      u_pressure_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pressure_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pressure_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pressure_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pressure_value = u_pressure_value.real;
      offset += sizeof(this->pressure_value);
      this->timestamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.sec);
      this->timestamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.nsec);
     return offset;
    }

    virtual const char * getType() override { return "triskarino_msgs/touchdata"; };
    virtual const char * getMD5() override { return "7632a9acc45b832ed81f2e84094a3f02"; };

  };

}
#endif
