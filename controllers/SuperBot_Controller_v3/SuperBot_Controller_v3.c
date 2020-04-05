/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <base.h>
#include <gripper.h>
#include <webots/keyboard.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/touch_sensor.h>
#include <webots/camera.h>
#include <webots/camera_recognition_object.h>


#define TIME_STEP 32

#define max(a,b) ( ((a)>(b)) ? (a):(b) )
#define min(a,b) ( ((a)>(b)) ? (b):(a) )
#define abs(a) ( ((a)<(0)) ? (0):(a))

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

static void display_helper_message() {
  printf("Control commands:\n");
  printf(" Arrows:       Move the robot\n");
  printf(" Page Up/Down: Rotate the robot\n");
  printf(" +/-:          (Un)grip\n");
  printf(" Shift + arrows:   Handle the arm\n");
  printf(" Space: Reset\n");
}

#define MAX_WIDTH 0.2f
#define MIN_WIDTH 0.0f
WbDeviceTag motorL;
WbDeviceTag motorR;
WbDeviceTag forceL;
WbDeviceTag forceR;

double width = 0.0; //抓手目标值
double height = 0.0;
void ClawControll(double width)
{
  width = max(min(abs(width),MAX_WIDTH),MIN_WIDTH);
  wb_motor_set_position(motorL, (MAX_WIDTH-width)/2);
  wb_motor_set_position(motorR,(MAX_WIDTH-width)/2);
}

#define MAX_HEIGHT 0.4f
#define MIN_HEIGHT 0.03f
WbDeviceTag motorM;
static void UpDownControll(double height)
{
  height = max(min(abs(height),MAX_HEIGHT),MIN_HEIGHT);
  wb_motor_set_position(motorM,  height);
}

static WbDeviceTag gripper_motors[3];
#define GRIPPER_MOTOR_MAX_SPEED 0.1
void lift(double position) {
  wb_motor_set_velocity(gripper_motors[0], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[0], position);
}

void moveFingers(double position) {
  wb_motor_set_velocity(gripper_motors[1], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_velocity(gripper_motors[2], GRIPPER_MOTOR_MAX_SPEED);
  wb_motor_set_position(gripper_motors[1], position);
  wb_motor_set_position(gripper_motors[2], position);
}

void init_all(){
// 机器人初始化
  wb_robot_init();
  base_init();
  passive_wait(2.0);

  WbDeviceTag camera_top = wb_robot_get_device("camera_top"); //相机初始化
  WbDeviceTag camera_front = wb_robot_get_device("camera_front");
  wb_camera_enable(camera_top, TIME_STEP);
  wb_camera_recognition_enable(camera_top, TIME_STEP);
  wb_camera_enable(camera_front, TIME_STEP);
  wb_camera_recognition_enable(camera_front, TIME_STEP);

  display_helper_message();

  wb_keyboard_enable(TIME_STEP);

  gripper_motors[0] = wb_robot_get_device("lift motor");
  gripper_motors[1] = wb_robot_get_device("left finger motor");
  gripper_motors[2] = wb_robot_get_device("right finger motor");

  motorM = wb_robot_get_device("linear motorMain");
  motorL = wb_robot_get_device("linear motorL");
  motorR = wb_robot_get_device("linear motorR");
  //forceL = wb_robot_get_device("Left_touch sensor");
  //forceR = wb_robot_get_device("Right_touch sensor");
  //wb_touch_sensor_enable(forceL, TIME_STEP);
  //wb_touch_sensor_enable(forceR, TIME_STEP);
}

int keyboard_control(int c){
// 键盘控制基本运动
  if ((c >= 0)) { //&& c != pc) {//不要求键值变化
    switch (c)
    {
    case WB_KEYBOARD_UP:
      printf("Go forwards\n");
      base_forwards();
      break;
    case WB_KEYBOARD_DOWN:
      printf("Go backwards\n");
      base_backwards();
      break;
    case WB_KEYBOARD_LEFT:
      printf("Strafe left\n");
      base_strafe_left();
      break;
    case WB_KEYBOARD_RIGHT:
      printf("Strafe right\n");
      base_strafe_right();
      break;
    case WB_KEYBOARD_PAGEUP:
      printf("Turn left\n");
      base_turn_left();
      break;
    case WB_KEYBOARD_PAGEDOWN:
      printf("Turn right\n");
      base_turn_right();
      break;
    case WB_KEYBOARD_END:
    case ' ':
      printf("Reset\n");
      base_reset();
      // arm_reset();
      break;
    case '+':
    case 388:
    case 65585:
      printf("Grip\n");
      //  gripper_grip();
      break;
    case '-':
    case 390:
      printf("Ungrip\n");
      //  gripper_release();
      break;
    case 332:
    case WB_KEYBOARD_UP | WB_KEYBOARD_SHIFT:
      //UpDownControll(Target_Height+=0.02);
      lift(height += 0.005);
      printf("Increase arm height\n");
      break;
    case 326:
    case WB_KEYBOARD_DOWN | WB_KEYBOARD_SHIFT:

      //UpDownControll(Target_Height-=0.02);
      lift(height -= 0.005);
      printf("Decrease arm height\n");
      // arm_decrease_height();
      break;
    case 330:
    case WB_KEYBOARD_RIGHT | WB_KEYBOARD_SHIFT:
      printf("Close the Claws\n");
      //ClawControll(Target_Width-=0.01);
      moveFingers(width -= 0.001);
      break;
    case 328:
    case WB_KEYBOARD_LEFT | WB_KEYBOARD_SHIFT:
      printf("Open the Claws\n");
      //ClawControll(Target_Width+=0.01);
      moveFingers(width += 0.001);
      break;
    default:
      fprintf(stderr, "Wrong keyboard input\n");
      break;
    }
  }
  return 0;
}

void BasicMove(double position, double pose, double* dist_sensor){
// 指定位姿 PTP 局部避障
}

int FindEmpty(int state){
// 寻找空货架 给一个固定的巡逻轨迹 返回货架位置 上下层 商品种类
  return state;
}

int FindGoods(int state, int goods_class){
// 前部摄像头寻找指定商品 给一个固定的巡逻轨迹 靠近直到顶部摄像头能捕捉
  return state;
}

int AimandGrasp(int state){
// 顶部摄像头校准并抓取
  return state;
}

int ReturnandLoad(int state, double targetplace)
{
// 返回货架放置货物 不写导航的话可以手动插补一下 最多插一次就够了
  return state;
}

int main(int argc, char **argv) {
  init_all();
  int main_state = 0;
  // 主状态机
  // 0 寻找空货架
  // 1 寻找商品
  // 2 抓取
  // 3 返回并放置
  while (true) {
  // const double force_valueL = wb_touch_sensor_get_value(forceL);
  // const double force_valueR = wb_touch_sensor_get_value(forceR);
  // if (force_valueL > 0.01||force_valueR > 0.01) {
  // printf("Collision of (%g , %g) N\n", force_valueL,force_valueR);
  // }
    step();
    keyboard_control(wb_keyboard_get_key());
  }

  wb_robot_cleanup();

  return 0;
}
