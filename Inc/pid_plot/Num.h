#ifndef _ROS_pid_plot_Num_h
#define _ROS_pid_plot_Num_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pid_plot
{

  class Num : public ros::Msg
  {
    public:
      typedef uint16_t _output_rpm_type;
      _output_rpm_type output_rpm;
      typedef uint16_t _output_controller_type;
      _output_controller_type output_controller;
      typedef uint16_t _input_setpoint_type;
      _input_setpoint_type input_setpoint;

    Num():
      output_rpm(0),
      output_controller(0),
      input_setpoint(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->output_rpm >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->output_rpm >> (8 * 1)) & 0xFF;
      offset += sizeof(this->output_rpm);
      *(outbuffer + offset + 0) = (this->output_controller >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->output_controller >> (8 * 1)) & 0xFF;
      offset += sizeof(this->output_controller);
      *(outbuffer + offset + 0) = (this->input_setpoint >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->input_setpoint >> (8 * 1)) & 0xFF;
      offset += sizeof(this->input_setpoint);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->output_rpm =  ((uint16_t) (*(inbuffer + offset)));
      this->output_rpm |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->output_rpm);
      this->output_controller =  ((uint16_t) (*(inbuffer + offset)));
      this->output_controller |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->output_controller);
      this->input_setpoint =  ((uint16_t) (*(inbuffer + offset)));
      this->input_setpoint |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->input_setpoint);
     return offset;
    }

    virtual const char * getType() override { return "pid_plot/Num"; };
    virtual const char * getMD5() override { return "5652f9c0d51f72a895489fcab75162b1"; };

  };

}
#endif
