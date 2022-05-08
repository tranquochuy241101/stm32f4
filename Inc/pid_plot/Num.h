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
      typedef float _output_rpm_type;
      _output_rpm_type output_rpm;
      typedef float _output_controller_type;
      _output_controller_type output_controller;
      typedef float _input_setpoint_type;
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
      union {
        float real;
        uint32_t base;
      } u_output_rpm;
      u_output_rpm.real = this->output_rpm;
      *(outbuffer + offset + 0) = (u_output_rpm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output_rpm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_output_rpm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_output_rpm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output_rpm);
      union {
        float real;
        uint32_t base;
      } u_output_controller;
      u_output_controller.real = this->output_controller;
      *(outbuffer + offset + 0) = (u_output_controller.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output_controller.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_output_controller.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_output_controller.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output_controller);
      union {
        float real;
        uint32_t base;
      } u_input_setpoint;
      u_input_setpoint.real = this->input_setpoint;
      *(outbuffer + offset + 0) = (u_input_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_input_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_input_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_input_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_setpoint);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_output_rpm;
      u_output_rpm.base = 0;
      u_output_rpm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output_rpm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_output_rpm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_output_rpm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->output_rpm = u_output_rpm.real;
      offset += sizeof(this->output_rpm);
      union {
        float real;
        uint32_t base;
      } u_output_controller;
      u_output_controller.base = 0;
      u_output_controller.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output_controller.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_output_controller.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_output_controller.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->output_controller = u_output_controller.real;
      offset += sizeof(this->output_controller);
      union {
        float real;
        uint32_t base;
      } u_input_setpoint;
      u_input_setpoint.base = 0;
      u_input_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_input_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_input_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_input_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->input_setpoint = u_input_setpoint.real;
      offset += sizeof(this->input_setpoint);
     return offset;
    }

    virtual const char * getType() override { return "pid_plot/Num"; };
    virtual const char * getMD5() override { return "074b7ae52738348822edeb68a11cd203"; };

  };

}
#endif
