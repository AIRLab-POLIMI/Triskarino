#ifndef _ROS_triskarino_msgs_Sound_h
#define _ROS_triskarino_msgs_Sound_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace triskarino_msgs
{

  class Sound : public ros::Msg
  {
    public:
      typedef const char* _filepath_type;
      _filepath_type filepath;
      typedef float _volume_type;
      _volume_type volume;

    Sound():
      filepath(""),
      volume(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_filepath = strlen(this->filepath);
      varToArr(outbuffer + offset, length_filepath);
      offset += 4;
      memcpy(outbuffer + offset, this->filepath, length_filepath);
      offset += length_filepath;
      union {
        float real;
        uint32_t base;
      } u_volume;
      u_volume.real = this->volume;
      *(outbuffer + offset + 0) = (u_volume.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_volume.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_volume.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_volume.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->volume);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_filepath;
      arrToVar(length_filepath, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_filepath; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_filepath-1]=0;
      this->filepath = (char *)(inbuffer + offset-1);
      offset += length_filepath;
      union {
        float real;
        uint32_t base;
      } u_volume;
      u_volume.base = 0;
      u_volume.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_volume.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_volume.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_volume.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->volume = u_volume.real;
      offset += sizeof(this->volume);
     return offset;
    }

    virtual const char * getType() override { return "triskarino_msgs/Sound"; };
    virtual const char * getMD5() override { return "76d7e30e9dc91b20fc75a45ec728aa95"; };

  };

}
#endif
