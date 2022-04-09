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

ros::NodeHandle nh;

void vel_cb(const std_msgs::UInt8& msg);

//Publisher
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";

//Subcriber
ros::Subscriber<std_msgs::UInt8> vel_sub("velocity", &vel_cb);

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
	nh.subscribe(vel_sub);
}

void loop(void)
{
  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();

  HAL_Delay(1000);
}
void vel_cb(const std_msgs::UInt8& msg){
  
}
