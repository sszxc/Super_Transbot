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
#include <webots/gps.h>
#include <webots/compass.h>
#define TIME_STEP 32

#define max(a,b) ( ((a)>(b)) ? (a):(b) )
#define min(a,b) ( ((a)>(b)) ? (b):(a) )
#define abs(a) ( ((a)<(0)) ? (0):(a))
#define MAX_WIDTH 0.2f
#define MIN_WIDTH 0.0f
// WbDeviceTag motorL;
// WbDeviceTag motorR;
WbDeviceTag forceL;
WbDeviceTag forceR;
#define MAX_HEIGHT 0.4f
#define MIN_HEIGHT 0.03f
// WbDeviceTag motorM;
#define GRIPPER_MOTOR_MAX_SPEED 0.1
#define PI 3.1415926535f
static WbDeviceTag gripper_motors[3];
static WbDeviceTag camera[2];
WbDeviceTag gps;
WbDeviceTag compass;
double gps_values[2];//gps值
double compass_angle;//罗盘角度
double initial_posture[3];//起点位姿,0为x,1为z,2为角度，每段轨迹只设置一次
double tmp_target_posture[3];//临时目标位姿，需要不断计算
double fin_target_posture[3];//最终目标位姿，
//定点
int point_index = 0;//定点编号
char* point_name[8]=
{
  "Right","Right Top","Top","Left Top",
  "Left","Left Down","Down","Right Down"
};//定点编号
double fixed_posture[8][3]=
{
{1.05,0.00,PI*2},//右
{1.05,-1.05,PI*2},//右上
{0.00,-1.05,PI/2},//上
{-1.05,-1.05,PI},//左上
{-1.05,0,PI},//左
{-1.05,0,3*PI/2},//左下
{-1.05,1.05,3*PI/2},//下
{1.05,1.05,PI*2}//右下
};

double width = 0.0; //抓手目标值
double height = 0.0;


static void step();
static void passive_wait(double sec);
static void display_helper_message();
void lift(double position);
void moveFingers(double position);
void init_all();
void caculate_tmp_target();
void set_posture(double posture[],double x,double z,double angle);
void get_gps_values(double v_gps[]);
double vector2_angle(const double v1[], const double v2[]);
void get_compass_angle(double *ret_angle);
int keyboard_control(int c);
void BasicMove(double position, double direction, double* dist_sensor);
int FindEmpty(int state);
int FindGoods(int state, WbDeviceTag camera, int goods_class);
int AimandGrasp(int state);
int ReturnandLoad(int state, double targetplace);
bool targetdist_reached(double target_posture[],double dist_threshold);
bool targetpos_reached(double target_posture[],double pos_threshold);

int main(int argc, char **argv) {
  init_all();
  //int main_state = 0;
  // 主状态机
  // 0 寻找空货架
  // 1 寻找商品
  // 2 抓取 张学超
  // 3 返回并放置

  //工单
  //装GPS
  //装罗盘
  while (true) {
    // const double force_valueL = wb_touch_sensor_get_value(forceL);
    // const double force_valueR = wb_touch_sensor_get_value(forceR);
    // if (force_valueL > 0.01||force_valueR > 0.01) {
    // printf("Collision of (%g , %g) N\n", force_valueL,force_valueR);
    // }
    // if(main_state==0){
    // main_state = FindEmpty(main_state);
    // } 
    step();
    if(targetdist_reached(tmp_target_posture,0.1))
    {

      if(targetdist_reached(fin_target_posture,0.05)&&targetpos_reached(fin_target_posture,0.05))
      {
        set_posture(initial_posture,gps_values[0],gps_values[1],compass_angle);
        //设置下一个定点位姿
        point_index += 1;
        point_index %= 8;
        set_posture(fin_target_posture,fixed_posture[point_index][0],fixed_posture[point_index][1],fixed_posture[point_index][2]);
        // printf("initial target： %.3f  %.3f  %.3f\n",initial_posture[0],initial_posture[1],initial_posture[2]);
        // printf("final target： %.3f  %.3f  %.3f\n",fin_target_posture[0],fin_target_posture[1],fin_target_posture[2]);
      }
      caculate_tmp_target(tmp_target_posture);
      base_goto_set_target(tmp_target_posture[0],tmp_target_posture[1],tmp_target_posture[2]);
    }
    printf("Target:%s\n",point_name[point_index]);
    printf("initial target： %.3f  %.3f  %.3f\n",initial_posture[0],initial_posture[1],initial_posture[2]);
    printf("tmp target： %.3f  %.3f  %.3f\n",tmp_target_posture[0],tmp_target_posture[1],tmp_target_posture[2]);
    printf("final target： %.3f  %.3f  %.3f\n\n",fin_target_posture[0],fin_target_posture[1],fin_target_posture[2]);
    base_goto_run();
    keyboard_control(wb_keyboard_get_key());
    //main_state = FindGoods(main_state, camera[1], 0);
  }

  wb_robot_cleanup();

  return 0;
}

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

  camera[0] = wb_robot_get_device("camera_top"); //相机初始化
  camera[1] = wb_robot_get_device("camera_front");
  wb_camera_enable(camera[0], TIME_STEP);
  wb_camera_recognition_enable(camera[0], TIME_STEP);
  wb_camera_enable(camera[1], TIME_STEP);
  wb_camera_recognition_enable(camera[1], TIME_STEP);
  
  //GPS初始化
  gps = wb_robot_get_device("gps_copy");
  wb_gps_enable(gps, TIME_STEP);
  //Compass初始化
  compass = wb_robot_get_device("compass_copy");
  wb_compass_enable(compass, TIME_STEP); 
  //底盘全方位移动初始化
  base_goto_init(TIME_STEP);
  //设置初始位姿
  step();
  get_gps_values(gps_values);
  get_compass_angle(&compass_angle);
  set_posture(initial_posture,gps_values[0],gps_values[1],compass_angle);
  //设置第一个定点位姿
  set_posture(fin_target_posture,fixed_posture[point_index][0],fixed_posture[point_index][1],fixed_posture[point_index][2]);
  //计算下一个临时目标;
  caculate_tmp_target(tmp_target_posture);
  //设置底盘运动目标
  base_goto_set_target(tmp_target_posture[0],tmp_target_posture[1],tmp_target_posture[2]);
  
  printf("Target:%s\n",point_name[point_index]);
  printf("initial target： %.3f  %.3f  %.3f\n",initial_posture[0],initial_posture[1],initial_posture[2]);
  printf("tmp target： %.3f  %.3f  %.3f\n",tmp_target_posture[0],tmp_target_posture[1],tmp_target_posture[2]);
  printf("final target： %.3f  %.3f  %.3f\n\n",fin_target_posture[0],fin_target_posture[1],fin_target_posture[2]);
  
  display_helper_message();
  wb_keyboard_enable(TIME_STEP);

  gripper_motors[0] = wb_robot_get_device("lift motor");
  gripper_motors[1] = wb_robot_get_device("left finger motor");
  gripper_motors[2] = wb_robot_get_device("right finger motor");


}

//细分目标位姿
double SUB = 2.0;//细分目标份数
void caculate_tmp_target(double tmp_posture[])
{
  get_gps_values(gps_values);
  get_compass_angle(&compass_angle);
  tmp_posture[0] = gps_values[0] + (fin_target_posture[0] - gps_values[0])/SUB;
  tmp_posture[1] = gps_values[1] + (fin_target_posture[1] - gps_values[1])/SUB;
  tmp_posture[2] = compass_angle + (fin_target_posture[2] - compass_angle)/(SUB*5);
}
//设置位姿
void set_posture(double posture[],double x,double z,double angle)
{
  posture[0] = x;
  posture[1] = z;
  posture[2] = angle;
}
bool targetdist_reached(double target_posture[],double dist_threshold)
{
  get_gps_values(gps_values);
  double dis = sqrt((gps_values[0]-target_posture[0]) * (gps_values[0]-target_posture[0]) + (gps_values[1]-target_posture[1]) * (gps_values[1]-target_posture[1]));
  
  // double angle = compass_angle - target_posture[2];
  if(dis <= dist_threshold) return true;
  else
  {
    printf("距离目标位置：%.3f  m\n",dis);
    return false;
  }
}
bool targetpos_reached(double target_posture[],double pos_threshold)
{
  get_compass_angle(&compass_angle);
  double angle = target_posture[2] - compass_angle;
  if(fabs(angle) <= pos_threshold) return true;
  return false;

}
//获取GPS的值
void get_gps_values(double v_gps[]){
  const double *gps_raw_values = wb_gps_get_values(gps);  
  v_gps[0] = gps_raw_values[0];
  v_gps[1] = gps_raw_values[2];
}

double vector2_angle(const double v1[], const double v2[]) {
  return atan2(v2[1], v2[0]) - atan2(v1[1], v1[0]);
}
//计算罗盘角度
void get_compass_angle(double *ret_angle){
  const double *compass_raw_values = wb_compass_get_values(compass);
  const double v_front[2] = {compass_raw_values[0], compass_raw_values[1]};
  const double v_north[2] = {1.0, 0.0};
  *ret_angle = vector2_angle(v_front, v_north) + PI;// angle E(0, 2*PI)
  printf("当前姿态：%.3f  rad\n",*ret_angle);  
}

//键盘控制基本运动
int keyboard_control(int c){
  if ((c >= 0)) { //&& c != pc) {//不要求键值变化
    switch (c)
    {
    case 'G':
    {
      get_gps_values(gps_values);
      printf("GPS device: %.3f %.3f\n", gps_values[0], gps_values[1]);
      get_compass_angle(&compass_angle);
      printf("Compass device: %.3f\n", compass_angle);
      break;
    }
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
//用GPS行驶到指定位姿 PTP 局部避障 计算一个瞬时速度'''
void BasicMove(double position, double direction, double* dist_sensor){

}
//'''寻找空货架 给四个定点GPS 摄像头看四面墙 返回货架位置和一个商品种类'''
int FindEmpty(int state){

  return state;
}
//'''给一个固定的巡逻轨 前部摄像头寻找指定商品 迹 靠近直到顶部摄像头能捕捉'''
int FindGoods(int state, WbDeviceTag camera, int goods_class){

// 下面是demo 看起来一个摄像头就够了
  int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
  printf("\n识别到 %d 个物体.\n", number_of_objects);
  const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
  for (int i = 0; i < number_of_objects; ++i)
  {
    printf("物体 %d 的类型: %s\n", i, objects[i].model);
    printf("物体 %d 的ID: %d\n", i, objects[i].id);
    printf("物体 %d 的相对位置: %lf %lf %lf\n", i, objects[i].position[0], objects[i].position[1],
          objects[i].position[2]);
    printf("物体 %d 的相对姿态: %lf %lf %lf %lf\n", i, objects[i].orientation[0], objects[i].orientation[1],
          objects[i].orientation[2], objects[i].orientation[3]);
    printf("物体的大小 %d: %lf %lf\n", i, objects[i].size[0], objects[i].size[1]);
    printf("物体 %d 在图像中的坐标: %d %d\n", i, objects[i].position_on_image[0],
          objects[i].position_on_image[1]);
    printf("物体 %d 在图像中的大小: %d %d\n", i, objects[i].size_on_image[0], objects[i].size_on_image[1]);
    for (int j = 0; j < objects[i].number_of_colors; ++j)
      printf("颜色 %d/%d: %lf %lf %lf\n", j + 1, objects[i].number_of_colors, objects[i].colors[3 * j],
            objects[i].colors[3 * j + 1], objects[i].colors[3 * j + 2]);
  }
  return state;
}
//'''前部摄像头校准并抓取'''
int AimandGrasp(int state){

  return state;
}
//'''返回货架放置货物 手动插补一下 最多插一次就够了'''
int ReturnandLoad(int state, double targetplace){

  return state;
}

