#ifndef _ROS_triskarino_msgs_RawOdometry_h
#define _ROS_triskarino_msgs_RawOdometry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace triskarino_msgs
{

  class RawOdometry : public ros::Msg
  {
    public:
      float odometryPos[3];
      float odometryVel[3];

    RawOdometry():
      odometryPos(),
      odometryVel()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_odometryPosi;
      u_odometryPosi.real = this->odometryPos[i];
      *(outbuffer + offset + 0) = (u_odometryPosi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_odometryPosi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_odometryPosi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_odometryPosi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->odometryPos[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_odometryVeli;
      u_odometryVeli.real = this->odometryVel[i];
      *(outbuffer + offset + 0) = (u_odometryVeli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_odometryVeli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_odometryVeli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_odometryVeli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->odometryVel[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_odometryPosi;
      u_odometryPosi.base = 0;
      u_odometryPosi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_odometryPosi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_odometryPosi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_odometryPosi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->odometryPos[i] = u_odometryPosi.real;
      offset += sizeof(this->odometryPos[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_odometryVeli;
      u_odometryVeli.base = 0;
      u_odometryVeli.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_odometryVeli.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_odometryVeli.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_odometryVeli.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->odometryVel[i] = u_odometryVeli.real;
      offset += sizeof(this->odometryVel[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "triskarino_msgs/RawOdometry"; };
    virtual const char * getMD5() override { return "72a9df76678f7f65559625554ef4e304"; };

  };

}
#endif
