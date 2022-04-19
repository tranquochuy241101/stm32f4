/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <pid_plot/Num.h>
#include <stdint.h>
#include <stdio.h>

ros::NodeHandle nh;

void vel_cb(const std_msgs::UInt8& msg);

uint16_t count = 0;
//Publisher
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";

std_msgs::String str_msg_1;
ros::Publisher chatter_1("chatter_1", &str_msg_1);
char hello_1[] = "Hello world Gay!";

//Subscriber
//ros::Subscriber<std_msgs::UInt8> vel_sub("velocity", &vel_cb);

//Create a publisher node to topic PID_data
pid_plot::Num pid_msg;
ros::Publisher pid_pub("PID_data", &pid_msg);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(chatter_1);
  nh.advertise(pid_pub);
//	nh.subscribe(vel_sub);
}

void loop(void)
{
	pid_msg.input_setpoint = 3;
	pid_msg.output_controller = 4;
	pid_msg.output_rpm = count;
	str_msg.data = hello;
	str_msg_1.data = hello_1;
	chatter.publish(&str_msg);
	chatter_1.publish(&str_msg_1);
	pid_pub.publish(&pid_msg);
	nh.spinOnce();
	count = count + 10;
	HAL_Delay(50);
	printf("Gay");
}

void vel_cb(const std_msgs::UInt8& msg){
  
}
