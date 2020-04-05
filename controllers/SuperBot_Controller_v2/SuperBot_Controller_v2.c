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

#include <webots/keyboard.h>
#include <webots/robot.h>

#include <webots/motor.h>
#include <webots/touch_sensor.h>

#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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



int main(int argc, char **argv) {
  wb_robot_init();

  base_init();
  passive_wait(2.0);

  display_helper_message();

  int pc = 0;
  wb_keyboard_enable(TIME_STEP);
  
  
  motorM = wb_robot_get_device("linear motorMain");
  motorL = wb_robot_get_device("linear motorL");
  motorR = wb_robot_get_device("linear motorR");
 // forceL = wb_robot_get_device("Left_touch sensor");
  //forceR = wb_robot_get_device("Right_touch sensor");
  //wb_touch_sensor_enable(forceL, TIME_STEP);
  //wb_touch_sensor_enable(forceR, TIME_STEP);
  double Target_Height = MIN_HEIGHT;
  double Target_Width = MAX_WIDTH;
  while (true) {
   // const double force_valueL = wb_touch_sensor_get_value(forceL);
  //  const double force_valueR = wb_touch_sensor_get_value(forceR);
   // if (force_valueL > 0.01||force_valueR > 0.01) {
   // printf("Collision of (%g , %g) N\n", force_valueL,force_valueR);
   // }
    step();
    int c = wb_keyboard_get_key();
    if ((c >= 0) && c != pc) {
      switch (c) {
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
          printf("Increase arm height\n");
          UpDownControll(Target_Height+=0.02);

          break;
        case 326:
        case WB_KEYBOARD_DOWN | WB_KEYBOARD_SHIFT:
          printf("Decrease arm height\n");
          UpDownControll(Target_Height-=0.02);
          break;
        case 330:
        case WB_KEYBOARD_RIGHT | WB_KEYBOARD_SHIFT:
          printf("Close the Claws\n");
          ClawControll(Target_Width-=0.01);
        
          break;
        case 328:
        case WB_KEYBOARD_LEFT | WB_KEYBOARD_SHIFT:
          printf("Open the Claws\n");
          ClawControll(Target_Width+=0.01);
          break;
        default:
          fprintf(stderr, "Wrong keyboard input\n");
          break;
      }
    }
    pc = c;
  }

  wb_robot_cleanup();

  return 0;
}
