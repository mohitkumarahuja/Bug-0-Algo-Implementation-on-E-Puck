
/*

  { Project 3 }


Code submitted by:
Team members:

1.> Mohit Kumar Ahuja
2.> Marc Blanchon


Course 	: MSCV
Subject : Autonomous Robotics




Code submitted to: 

Prof. Xevi Cufí   
University of GIRONA

*/

//including important libraries
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/accelerometer.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 64 //ms
#define WHEEL_RADIUS 0.0205 //m
#define AXLE_LENGTH 0.052 //m
#define ENCODER_RESOLUTION 159.23 /* pulses per revolution  */
#define RANGE (1024 / 2)


// Global variable //

// Sensor values are defined globally because it is being used in functions as well as in main program.
double sensors_value_0; // assigning sensor values a data type.
double sensors_value_1; // assigning sensor values a data type.
double sensors_value_4; // assigning sensor values a data type.
double sensors_value_5; // assigning sensor values a data type.
double sensors_value_6; // assigning sensor values a data type.
double sensors_value_7; // assigning sensor values a data type.
double da; 
double dl_new;
double dr_new;
double old_dl;
double old_dr;
double d[3] = {0.0,0.0,0.0}; // this will be used as the {x,y,Angle} in the program
bool rot = false;
double Goal[2] = {30,-90};  // declaring Goal {x,y} coordinates in cm

// Function Defination //
void wall_follower();
void update_odometry();

int main(int argc, char *argv[]) {

  /* define variables */
  double differences;
  double store_value;
  double Angle;
  int speed_0;    // motor speed for left 
  int speed_1;    // motor speed for right
  int x = 2;
  
  wb_robot_init(); // initialize Webots //
  WbDeviceTag distance_sensor_0; //declared device tag variables 
  WbDeviceTag distance_sensor_1; //declared device tag variables 
  WbDeviceTag distance_sensor_4; //declared device tag variables 
  WbDeviceTag distance_sensor_5; //declared device tag variables 
  WbDeviceTag distance_sensor_6; //declared device tag variables 
  WbDeviceTag distance_sensor_7; //declared device tag variables 
  
  distance_sensor_0 = wb_robot_get_device("ps0"); //get distance sensors
  distance_sensor_1 = wb_robot_get_device("ps1"); //get distance sensors
  distance_sensor_4 = wb_robot_get_device("ps4"); //get distance sensors
  distance_sensor_5 = wb_robot_get_device("ps5"); //get distance sensors
  distance_sensor_6 = wb_robot_get_device("ps6"); //get distance sensors
  distance_sensor_7 = wb_robot_get_device("ps7"); //get distance sensors
  
  wb_differential_wheels_enable_encoders(TIME_STEP*4); //enabling encoders
  wb_distance_sensor_enable(distance_sensor_0,TIME_STEP*4); //enabling distance sensors
  wb_distance_sensor_enable(distance_sensor_1,TIME_STEP*4); //enabling distance sensors
  wb_distance_sensor_enable(distance_sensor_4,TIME_STEP*4); //enabling distance sensors
  wb_distance_sensor_enable(distance_sensor_5,TIME_STEP*4); //enabling distance sensors
  wb_distance_sensor_enable(distance_sensor_6,TIME_STEP*4); //enabling distance sensors
  wb_distance_sensor_enable(distance_sensor_7,TIME_STEP*4); //enabling distance sensors
  
  sensors_value_0 = wb_distance_sensor_get_value(distance_sensor_0); //getting values from those distance sensors
  sensors_value_1 = wb_distance_sensor_get_value(distance_sensor_1); //getting values from those distance sensors
  sensors_value_4 = wb_distance_sensor_get_value(distance_sensor_4); //getting values from those distance sensors
  sensors_value_5 = wb_distance_sensor_get_value(distance_sensor_5); //getting values from those distance sensors
  sensors_value_6 = wb_distance_sensor_get_value(distance_sensor_6); //getting values from those distance sensors
  sensors_value_7 = wb_distance_sensor_get_value(distance_sensor_7); //getting values from those distance sensors
  Goal[0] = Goal[0] * 0.01; // Because the calculation is in meters
  Goal[1] = Goal[1] * 0.01; // So converting it into meters
  
  /* infinite for loop */
  for (;;) {
  
    if(x == 2){  
      store_value = sqrt((Goal[0] - d[0])*(Goal[0] - d[0])+(Goal[1] - d[1])*(Goal[1] - d[1]));
      x=0;
    }
    
    differences = sqrt((Goal[0] - d[0])*(Goal[0] - d[0])+(Goal[1] - d[1])*(Goal[1] - d[1]));
    Angle = atan2(Goal[1] - d[1] , Goal[0] - d[0]);  //Finding the angle theta
    printf("\n");
    printf("value of theta: %g \n",Angle);
    update_odometry();                              //update the coordinates of {x,y} and Angle
    
    if (da<=(-Angle)  && x == 0 && rot==false){     // Move left if the goal lies on left side of the plane
      rot=true;     // set rot to 1
      speed_0=-100; // Left Wheel movement
      speed_1=100;  // Right Wheel movement
    }
    
    else  if(da>(-Angle)  && x == 0 && rot==false){ // Move Right if the goal lies on left side of the plane
      rot=true;     // set rot to 1
      speed_0=100;  // Left Wheel movement
      speed_1=-100; // Right Wheel movement
    }
    
    else { // Go straight if facing towanrds goal
      
      speed_0=250; // Left Wheel movement
      speed_1=250; // Right Wheel movement
      rot=false;   // set rot to 0
      
      // If any one of the sensors 0,1,6,7,5 detect wall ie sensor_value>500 it will start following wall
      if ((sensors_value_0>500)||(sensors_value_1>500)||(sensors_value_6>500)||(sensors_value_7>500)||(sensors_value_5>500)) { // 1
        wall_follower();    // wall_follower function will execute once and then again come back. 
        update_odometry();  // after we rotate it will automatically update the {x,y} coordinated and the angle.
      }
      
      else {
        wall_follower();   // wall_follower function will execute once and then again come back.
        update_odometry(); // after we rotate it will automatically update the {x,y} coordinated and the angle.
      }
      
      if(differences < 0.025 && differences > -0.025){              // Here is the range defined i.e if the real coordinates of  
        
        speed_0=0; //Left Wheel movement should stop                // robot will lie in between Goal coordinates when robot reaches 
        speed_1=0; //Right Wheel movement should stop               // the goal, then it should automatically stop.
        rot=true;  //rot set to 1                                   // Range is defined so as to nullify the effects of errors.
        wb_differential_wheels_set_speed(speed_0,speed_1);
        wb_robot_step(TIME_STEP*1000000);
      }   
    }
    
    wb_differential_wheels_set_speed(speed_0,speed_1);
    wb_robot_step(TIME_STEP);
  }

  return 0;
}
void update_odometry() {
  double l = wb_differential_wheels_get_left_encoder(); // Steps for left motor
  double r = wb_differential_wheels_get_right_encoder();// Steps for right motor
  double dl = l / ENCODER_RESOLUTION * WHEEL_RADIUS; // distances covered by left wheel in meter
  double dr = r / ENCODER_RESOLUTION * WHEEL_RADIUS; // distances covered by right wheel in meter
   dl_new = (dl - old_dl);   // calculating difference from last value and current value for right tyre
   dr_new = (dr - old_dr);   // calculating difference from last value and current value for left tyre
   old_dl = dl;              // Assigning current value to some variable to use old value to calculate difference for left tyre
   old_dr = dr;              // Assigning current value to some variable to use old value to calculate difference for right tyre
   da = (dr - dl) / AXLE_LENGTH;            // delta orientation
  double dist = (dr_new + dl_new)/2;        //  Mean distance covered by both tyres
  d[2] +=  (dl_new - dr_new) / AXLE_LENGTH; // Angle
  d[0] +=  dist*cos(d[2]);                  // updated x coordinate
  d[1] +=  dist*sin(d[2]);                  // updated y coordinate
  printf("Calculated change by odometry: %g rad.\n",da);
  printf("Goal(Where the Robot should stop): {%g cm, %g cm}\n",100*Goal[0],100*Goal[1]);
  printf("Distance moved in x direction: %g cm.\n",100*d[0]);
  printf("Distance moved in y direction: %g cm.\n",100*d[1]);
  
}



void wall_follower() {
  
    WbDeviceTag distance_sensor_0; // initializing distence sennsors
    WbDeviceTag distance_sensor_1; // initializing distence sennsors
    WbDeviceTag distance_sensor_4; // initializing distence sennsors
    WbDeviceTag distance_sensor_5; // initializing distence sennsors
    WbDeviceTag distance_sensor_6; // initializing distence sennsors
    WbDeviceTag distance_sensor_7; // initializing distence sennsors
    
    wb_robot_init(); // initializing robot
    distance_sensor_0 = wb_robot_get_device("ps0"); //get distance sensors
    distance_sensor_1 = wb_robot_get_device("ps1"); //get distance sensors
    distance_sensor_4 = wb_robot_get_device("ps4"); //get distance sensors
    distance_sensor_5 = wb_robot_get_device("ps5"); //get distance sensors
    distance_sensor_6 = wb_robot_get_device("ps6"); //get distance sensors
    distance_sensor_7 = wb_robot_get_device("ps7"); //get distance sensors
  
    wb_distance_sensor_enable(distance_sensor_0,TIME_STEP*4); //enabling distance sensors
    wb_distance_sensor_enable(distance_sensor_1,TIME_STEP*4); //enabling distance sensors
    wb_distance_sensor_enable(distance_sensor_4,TIME_STEP*4); //enabling distance sensors
    wb_distance_sensor_enable(distance_sensor_5,TIME_STEP*4); //enabling distance sensors
    wb_distance_sensor_enable(distance_sensor_6,TIME_STEP*4); //enabling distance sensors
    wb_distance_sensor_enable(distance_sensor_7,TIME_STEP*4); //enabling distance sensors
    
    sensors_value_0 = wb_distance_sensor_get_value(distance_sensor_0); //getting values from those distance sensors
    sensors_value_1 = wb_distance_sensor_get_value(distance_sensor_1); //getting values from those distance sensors
    sensors_value_4 = wb_distance_sensor_get_value(distance_sensor_4); //getting values from those distance sensors
    sensors_value_5 = wb_distance_sensor_get_value(distance_sensor_5); //getting values from those distance sensors
    sensors_value_6 = wb_distance_sensor_get_value(distance_sensor_6); //getting values from those distance sensors
    sensors_value_7 = wb_distance_sensor_get_value(distance_sensor_7); //getting values from those distance sensors
    
    // If sensor_value 0,1,6,7 exceeds 500 then robot should turn right
  if (((sensors_value_0>=500)||(sensors_value_1>=500)||(sensors_value_6>=600)||(sensors_value_7>=500)) && (sensors_value_5<=500)) { // 1
      wb_differential_wheels_set_speed(100,-100);
      wb_robot_step(TIME_STEP*20);
      update_odometry();// after we rotate it will automatically update the {x,y} coordinated and the angle.
      printf("1 Right Turn for wall detection\n");
    }
    
    // If value of anyone of sensors 0,1,6,7 exceeds 500 then robot should turn right
    else if (((sensors_value_0>=500)||(sensors_value_1>=500)||(sensors_value_6>=600)||(sensors_value_7>=500)) && (sensors_value_5>=500)) { // 2
      wb_differential_wheels_set_speed(100,-100);
      wb_robot_step(TIME_STEP*20);
      update_odometry();// after we rotate it will automatically update the {x,y} coordinated and the angle.
      printf("2 Right Turn\n");
    }
    
    // If value of all the sensors 0,1,6,7 reduces from 500 but sensor 5 have value greater than 500 then robot should go straight
    else if ((((sensors_value_0<=500)||(sensors_value_1<=500))&&((sensors_value_6<=500)||(sensors_value_7<=500))) && ((sensors_value_5>500)||(sensors_value_4>500))){ // 3 Straight
      printf("3 Straight\n");
      wb_differential_wheels_set_speed(200,200);
      wb_robot_step(TIME_STEP);
      update_odometry();// after we move it will automatically update the {x,y} coordinated and the angle.
    }
    
    // If value of all the sensors 0,1,6,7 reduces from 500 but sensor 5 have value from 400 till 500 then robot should turn left
    else if ((((sensors_value_0<=500)||(sensors_value_1<=500))&&((sensors_value_6<=500)||(sensors_value_7<=500))) && (500>sensors_value_5 && sensors_value_5>400)){ // 4 Left
      printf("4 Slight Left\n");
      wb_differential_wheels_set_speed(-10,40);
      wb_robot_step(TIME_STEP);
      update_odometry();// after we rotate it will automatically update the {x,y} coordinated and the angle.
    }
    
    // If value of all the sensors 0,1,6,7 reduces from 500 but sensor 5 have value from 600 till 550 then robot should turn right
    else if ((((sensors_value_0<=500)||(sensors_value_1<=500))&&((sensors_value_6<=500)||(sensors_value_7<=500))) && ((600>sensors_value_5 && sensors_value_5>550))){ // 5 Right
      printf("5 Slight Right\n");
      wb_differential_wheels_set_speed(40,-10);
      wb_robot_step(TIME_STEP);
      update_odometry();// after we rotate it will automatically update the {x,y} coordinated and the angle.
    }
    
    else {   // If none of the above condition is true then robot should move straight                                                     // 6
      printf("6 Move Forward\n");
      wb_differential_wheels_set_speed(200,200);
      wb_robot_step(TIME_STEP);
      update_odometry();// after we move it will automatically update the {x,y} coordinated and the angle.
    }
    
    printf("sensors_value_0 value: %f %f %f %f %f %f \n",sensors_value_0,sensors_value_1,sensors_value_6,sensors_value_7, sensors_value_5, sensors_value_4);
    
  }
