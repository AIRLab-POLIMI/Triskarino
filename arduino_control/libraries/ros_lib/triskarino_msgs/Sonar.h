#ifndef _ROS_triskarino_msgs_Sonar_h
#define _ROS_triskarino_msgs_Sonar_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace triskarino_msgs
{

  class Sonar : public ros::Msg
  {
    public:
      float distances[4];

    Sonar():
      distances()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_distancesi;
      u_distancesi.real = this->distances[i];
      *(outbuffer + offset + 0) = (u_distancesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_distancesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_distancesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_distancesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->distances[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      union {
        float real;
        uint32_t base;
      } u_distancesi;
      u_distancesi.base = 0;
      u_distancesi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_distancesi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_distancesi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_distancesi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->distances[i] = u_distancesi.real;
      offset += sizeof(this->distances[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "triskarino_msgs/Sonar"; };
    virtual const char * getMD5() override { return "76ec30598a8646984a9ca4c262539ef7"; };

  };

}
#endif
