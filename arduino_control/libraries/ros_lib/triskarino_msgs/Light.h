#ifndef _ROS_triskarino_msgs_Light_h
#define _ROS_triskarino_msgs_Light_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace triskarino_msgs
{

  class Light : public ros::Msg
  {
    public:
      typedef const char* _action_type;
      _action_type action;
      typedef int32_t _delay_type;
      _delay_type delay;
      int32_t color[3];

    Light():
      action(""),
      delay(0),
      color()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_action = strlen(this->action);
      varToArr(outbuffer + offset, length_action);
      offset += 4;
      memcpy(outbuffer + offset, this->action, length_action);
      offset += length_action;
      union {
        int32_t real;
        uint32_t base;
      } u_delay;
      u_delay.real = this->delay;
      *(outbuffer + offset + 0) = (u_delay.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_delay.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_delay.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_delay.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delay);
      for( uint32_t i = 0; i < 3; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_colori;
      u_colori.real = this->color[i];
      *(outbuffer + offset + 0) = (u_colori.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_colori.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_colori.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_colori.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->color[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_action;
      arrToVar(length_action, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_action; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_action-1]=0;
      this->action = (char *)(inbuffer + offset-1);
      offset += length_action;
      union {
        int32_t real;
        uint32_t base;
      } u_delay;
      u_delay.base = 0;
      u_delay.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_delay.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_delay.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_delay.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->delay = u_delay.real;
      offset += sizeof(this->delay);
      for( uint32_t i = 0; i < 3; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_colori;
      u_colori.base = 0;
      u_colori.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_colori.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_colori.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_colori.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->color[i] = u_colori.real;
      offset += sizeof(this->color[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "triskarino_msgs/Light"; };
    virtual const char * getMD5() override { return "c5fe140ffe64b68edb074e8bc56398f8"; };

  };

}
#endif
