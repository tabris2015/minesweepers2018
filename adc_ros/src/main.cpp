#include <Arduino.h>
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>

enum {
  SENSORS_NUM = 2,
};

//Set up the ros node and publisher
std_msgs::UInt8 mine_detected_msg;
ros::Publisher pub_mine_detected("/robot/mine_detected", &mine_detected_msg);
std_msgs::UInt16 mine_sensor_0_msg;
ros::Publisher pub_mine_sensor_0("/robot/sensor_0_value", &mine_sensor_0_msg);
std_msgs::UInt16 mine_sensor_1_msg;
ros::Publisher pub_mine_sensor_1("/robot/sensor_1_value", &mine_sensor_1_msg);
ros::NodeHandle nh;

uint32_t last_time;
uint32_t mine_sensors_period = 11;
uint16_t sensors_adc[SENSORS_NUM];
uint16_t sensor_thr[SENSORS_NUM] = {730, 710};
uint8_t publishing[SENSORS_NUM] = {0, 0};
uint8_t published[SENSORS_NUM] = {0, 0};
uint8_t i = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(13,OUTPUT);
  nh.initNode();
  nh.advertise(pub_mine_detected);
  nh.advertise(pub_mine_sensor_0);
  nh.advertise(pub_mine_sensor_1);
  last_time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - last_time >= mine_sensors_period) {
    last_time  = millis();
    sensors_adc[0] = analogRead(A0);
    sensors_adc[1] = analogRead(A1);

    mine_detected_msg.data = 0;
    if (sensors_adc[0] > sensor_thr[0]) {
      mine_detected_msg.data += 1;
    }
    if (sensors_adc[1] > sensor_thr[1]) {
      mine_detected_msg.data += 2;
    }
    if (mine_detected_msg.data != 0) {
      digitalWrite(4, 1);
      digitalWrite(5, 1);
      digitalWrite(13, 1);
      if (mine_detected_msg.data == 3 && (!published[0] || !published[1])) {
        published[0] = 1;
        published[1] = 1;
        pub_mine_detected.publish(&mine_detected_msg);
      }
      else {
        if (mine_detected_msg.data == 2 && (!published[1])) {
          published[1] = 1;
          pub_mine_detected.publish(&mine_detected_msg);
        }
        else if (mine_detected_msg.data == 1 && (!published[0])) {
          published[0] = 1;
          pub_mine_detected.publish(&mine_detected_msg);
        }
      }
    }
    else if (published[0] || published[1]) {
      digitalWrite(4, 0);
      digitalWrite(5, 0);
      digitalWrite(13, 0);
      published[0] = 0;
      published[1] = 0;
      pub_mine_detected.publish(&mine_detected_msg);
    }

    i++;
    if (i == 3) {
      mine_sensor_0_msg.data = sensors_adc[0];
      mine_sensor_1_msg.data = sensors_adc[1];
      pub_mine_sensor_0.publish(&mine_sensor_0_msg);
      pub_mine_sensor_1.publish(&mine_sensor_1_msg);
      i = 0;
    }
  }
  nh.spinOnce();
}