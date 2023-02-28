#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "crtp_commander_high_level.h"
#include "commander.h"
#include "pptraj.h"
#include "estimator_kalman.h"
#include "deck_analog.h"
#include "deck_constants.h"

// Flocking algorithm params
double alpha = 2.0;
double beta = 1.0;
double gama = 1.0;
double kappa = 0.0;
double sb = 0.3;
double sv = 0.5;
double su = 0.5;
double umax = 0.1;
double wmax = 1.5708/3;
double K1 = 0.08;
double K2 = 0.2;
double epsilon = 12.0;

uint8_t alpha_tmp;
uint8_t beta_tmp;
uint8_t kappa_tmp;
uint8_t sb_tmp;
uint8_t sv_tmp;
uint8_t K1_tmp;
uint8_t K2_tmp;;
uint8_t lmx_tmp;
uint8_t lmn_tmp;
uint8_t fh_tmp;
uint8_t wmax_tmp;
uint8_t umax_tmp;
uint8_t u_add_tmp;

// Boundary repulsion
double krep = 5.0;
double L0 = 0.5;
double rx = 0.0;
double ry = 0.0;

// Neighbourhood
double neg_xs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double neg_ys[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double neg_hs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double self_pos[] = {0.0, 0.0, 0.0};
bool neg_alive[] = {false, false, false, false, false, false, false};

// General variables
double distance = 0.0;
double ij_ang = 0.0;
double fh = 0.5;

// Heading alignment
double hx = 0.0;
double hy = 0.0;
float temp_h = 0.0; // This gets yaw
double sum_cosh = 0.0;
double sum_sinh = 0.0;

// Multi Ranger deck
float front_range = 5000;
float back_range = 5000;
float left_range = 5000;
float right_range = 5000;
double random_addition = 0.0;
double ox = 0.0;
double oy = 0.0;
double front_bias_angle = 0.0;

// Proximal force
double px = 0.0;
double py = 0.0;
double temp_px = 0.0;
double temp_py = 0.0;

// Total Force
double fx = 0.0;
double fy = 0.0;
double fx_raw = 0.0;
double fy_raw = 0.0;

// Goal force
double gx = 0.0;
double gy = 0.0;

// Linear and Angular Velocity
double u = 0.0;
double u_add = 0.05;
double w = 0.0;

// Velocity
double vx = 0.0;
double vy = 0.0;

// Light sensor
float lmx = 450;
float lmn = 50;

// Total flight time
double total_flight = 0.0;

static paramVarId_t paramIdCommanderEnHighLevel;
static paramVarId_t paramIdResetKalman;
static paramVarId_t paramIdiftakeoff;
static paramVarId_t paramIdifterminate;
static paramVarId_t paramIdifheading;
static paramVarId_t paramIdfmode;
static paramVarId_t paramIdgoalx;
static paramVarId_t paramIdgoaly;
static paramVarId_t paramIdformation;

static paramVarId_t paramIdUpdateParams;
static paramVarId_t paramIdAlpha;
static paramVarId_t paramIdBeta;
static paramVarId_t paramIdKappa;
static paramVarId_t paramIdSb;
static paramVarId_t paramIdSv;
static paramVarId_t paramIdK1;
static paramVarId_t paramIdK2;
static paramVarId_t paramIdLmx;
static paramVarId_t paramIdLmn;
static paramVarId_t paramIdFh;
static paramVarId_t paramIdWmax;
static paramVarId_t paramIdUmax;
static paramVarId_t paramIdUadd;

static void resetKalman() { paramSetInt(paramIdResetKalman, 1); }
static void enableHighlevelCommander() { paramSetInt(paramIdCommanderEnHighLevel, 1); }

// Coordinates struct
typedef struct {
  uint8_t id;
  float x;
  float y;
  float h;
  } _coords;

// Agg. data structure
typedef struct {
  uint32_t data_aggregated;
  double x;
  double y;
  double h;
  double light;
  } _data_to_agg;  

// State machine
typedef enum {
    idle,
    takingOff,
    onAir,
    land,
} State;

static State state = idle;


void p2pcallbackHandler(P2PPacket *p)
{
    // Defining wht to do when a packet is received
  _coords other_coords;
  memcpy(&other_coords, p->data, sizeof(other_coords));
  uint8_t other_id = other_coords.id;
  neg_alive[other_id - 1] = true;
  
  // Store received coordinates
  float other_X = other_coords.x;
  float other_Y = other_coords.y;
  float other_H = other_coords.h;
  neg_xs[other_id - 1] = (double)other_X;
  neg_ys[other_id - 1] = (double)other_Y;
  neg_hs[other_id - 1] = (double)other_H;
  //last_check = current_check;
}

// This function is used to send velocity commands since the original set VelCdmd does
// not have altitude control integrated and needs to be send with vz. 
// SetHoverSetpoint has altitude control integrated
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawRate)
{

  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawRate;

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->velocity_body = true;
}

int32_t aggregate_data(double x, double y, double h, double l)
{
  /* 
  Aggregation function to send relevant data in a single int.
  Example: x:5.0 y:1.0 heading(yaw):2.5 light_intensity:120
  This becomes -> 501025120
  */
  if (h < 0)
  {
    h = h + 6.28;
  }

  x = round(x * 10) / 10;
  y = round(y * 10) / 10;
  h = round(h * 10) / 10;
  l = round(l * 10) / 10;

  x = x * pow(10, 8);
  y = y * pow(10, 6);
  h = h * pow(10, 4);

  double t;
  t = x + y + h + l;

  int32_t c_data_aggregated;
  c_data_aggregated = (int32_t)t;

  return c_data_aggregated;
}

void appMain()
{
    static setpoint_t setpoint;
    static float heading_log;
    heading_log = 99;
    static uint8_t iftakeoff;
    static uint8_t ifterminate;
    static uint8_t ifheading;
    static uint8_t ifupdateParams;
    static uint8_t _fmode;
    static uint8_t _formation;
    static float _goal_x;
    static float _goal_y;
    _fmode = 2;
    _goal_x = 3.0;
    _goal_y = 2.25;
    iftakeoff = 0;
    ifterminate = 0;
    ifheading = 1;
    ifupdateParams = 0;    

    float a_read = 0.0f;
    adcInit();
    static float light_log;  
    light_log = 0;       
    static int32_t agg_data_log = 99999999;
    static uint8_t smy_id;

    // Setting Ids for logging
    logVarId_t idX = logGetVarId("stateEstimate", "x");
    logVarId_t idY = logGetVarId("stateEstimate", "y");
    logVarId_t idyaw = logGetVarId("stateEstimate", "yaw");
    logVarId_t ranger_front = logGetVarId("range", "front");
    logVarId_t ranger_back = logGetVarId("range", "back");
    logVarId_t ranger_right = logGetVarId("range", "right");
    logVarId_t ranger_left = logGetVarId("range", "left");

    static P2PPacket p_reply;
    p_reply.port=0x00;
   
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    p2pRegisterCB(p2pcallbackHandler);
    _coords self_coords;
    self_coords.id = my_id;
    smy_id = my_id;


    // Crazyflie logging groups
    //  -> Light, heading and agg data
    LOG_GROUP_START(synthLog)
    LOG_ADD_CORE(LOG_FLOAT, heading, &heading_log)
    LOG_ADD_CORE(LOG_FLOAT, light_intensity, &light_log)
    LOG_ADD_CORE(LOG_INT32, agg_data, &agg_data_log)
    LOG_ADD_CORE(LOG_UINT8, agent_id, &smy_id)
    LOG_GROUP_STOP(synthLog)

    // -> Flight modes
    PARAM_GROUP_START(fmodes)
    PARAM_ADD_CORE(PARAM_UINT8, if_takeoff, &iftakeoff)
    PARAM_ADD_CORE(PARAM_UINT8, if_terminate, &ifterminate)
    PARAM_ADD_CORE(PARAM_UINT8, if_heading, &ifheading)
    PARAM_ADD_CORE(PARAM_UINT8, fmode, &_fmode)
    PARAM_ADD_CORE(PARAM_FLOAT, goal_x, &_goal_x)
    PARAM_ADD_CORE(PARAM_FLOAT, goal_y, &_goal_y)
    PARAM_ADD_CORE(PARAM_UINT8, formation, &_formation)
    PARAM_GROUP_STOP(fmodes)

    // -> Parameters for experiments behaviours
    PARAM_GROUP_START(flockParams)
    PARAM_ADD_CORE(PARAM_UINT8, update_params, &ifupdateParams)
    PARAM_ADD_CORE(PARAM_UINT8, fh_param, &fh_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, alpha_param, &alpha_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, beta_param, &beta_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, sb_param, &sb_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, sv_param, &sv_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, kappa_param, &kappa_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, k1_param, &K1_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, k2_param, &K2_tmp)       
    PARAM_ADD_CORE(PARAM_UINT8, lmn_param, &lmn_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, lmx_param, &lmx_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, wmax_param, &wmax_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, umax_param, &umax_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, u_add_param, &u_add_tmp)
    PARAM_GROUP_STOP(flockParams)


    paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
    paramIdResetKalman = paramGetVarId("kalman", "resetEstimation");
    paramIdiftakeoff = paramGetVarId("fmodes", "if_takeoff");
    paramIdifterminate = paramGetVarId("fmodes", "if_terminate");
    paramIdfmode = paramGetVarId("fmodes", "fmode");
    paramIdifheading = paramGetVarId("fmodes", "if_heading");
    paramIdgoalx = paramGetVarId("fmodes", "goal_x");
    paramIdgoaly = paramGetVarId("fmodes", "goal_y");
    paramIdformation = paramGetVarId("fmodes", "formation");

    paramIdUpdateParams = paramGetVarId("flockParams", "update_params");
    paramIdFh = paramGetVarId("flockParams", "fh_param");
    paramIdAlpha = paramGetVarId("flockParams", "alpha_param");
    paramIdBeta = paramGetVarId("flockParams", "beta_param");
    paramIdSb = paramGetVarId("flockParams", "sb_param");
    paramIdSv = paramGetVarId("flockParams", "sv_param");    
    paramIdKappa = paramGetVarId("flockParams", "kappa_param");
    paramIdK1 = paramGetVarId("flockParams", "k1_param");
    paramIdK2 = paramGetVarId("flockParams", "k2_param"); 
    paramIdLmn = paramGetVarId("flockParams", "lmn_param");
    paramIdLmx = paramGetVarId("flockParams", "lmx_param");
    paramIdWmax = paramGetVarId("flockParams", "wmax_param");
    paramIdUmax = paramGetVarId("flockParams", "umax_param");
    paramIdUadd = paramGetVarId("flockParams", "u_add_param");

    resetKalman();
    enableHighlevelCommander();
    srand((unsigned int)xTaskGetTickCount());
    double test_rand = (double)rand()/(double)(RAND_MAX/6.28);
    DEBUG_PRINT("random is %f\n", test_rand);
    vTaskDelay(M2T(8000));

  while(1) {
    
    // Reading the lights from the onboard light sensor
    a_read = analogRead(DECK_GPIO_TX2);
    light_log = a_read;

    // Getting flag for On-the-fly parameter update
    ifupdateParams = paramGetInt(paramIdUpdateParams);

    if (ifupdateParams == 1) {
      K1 = ((double)paramGetInt(paramIdK1)) / 50;
      K2 = ((double)paramGetInt(paramIdK2)) / 50;           
      alpha = ((double)paramGetInt(paramIdAlpha)) / 50;
      beta = ((double)paramGetInt(paramIdBeta)) / 50;
      sb = ((double)paramGetInt(paramIdSb)) / 50;
      sv = ((double)paramGetInt(paramIdSv)) / 50; 
      kappa = ((double)paramGetInt(paramIdKappa)) / 50;
      fh = ((double)paramGetInt(paramIdFh)) / 10;
      lmn = ((float)paramGetInt(paramIdLmn)) * 4;
      lmx = ((float)paramGetInt(paramIdLmx)) * 4;
      umax = ((double)paramGetInt(paramIdUmax)) / 100;
      wmax = ((double)paramGetInt(paramIdWmax)) / 10 * 3.14;
      u_add = ((double)paramGetInt(paramIdUadd)) / 100;
      ifupdateParams = 0;
    }    

    if (state == idle) {

      if (iftakeoff == 0) {
        // Gets takeoff flag
        iftakeoff = paramGetInt(paramIdiftakeoff);
        vTaskDelay(M2T(100));
      }

      // Take off sequence
      if (iftakeoff == 1) {
        vTaskDelay(M2T(1000));
        crtpCommanderHighLevelTakeoff(0.5, 2.5);
        vTaskDelay(M2T(4000));
        state = takingOff;
        self_pos[2] = 0.0;
      }
    }

    // House keeping while drone is taking off
    if (state == takingOff) {
      if (crtpCommanderHighLevelIsTrajectoryFinished())
      {state = onAir;}
    }

    // Flying state
    if (state == onAir) {
      // Reseting all the vectors to be computed later
      // Proximal
      px = 0.0;
      py = 0.0;
      
      // Alignment
      hx = 0.0;
      hy = 0.0;

      // Boundary
      rx = 0.0;
      ry = 0.0;

      // Obstacle avoidance
      ox = 0.0;
      oy = 0.0;

      // Update of current state
      self_coords.x = logGetFloat(idX);
      self_coords.y = logGetFloat(idY);
      temp_h = logGetFloat(idyaw);
      self_coords.h = (double)temp_h * 0.0174532;
      heading_log = self_coords.h;
      self_pos[0] = (double)self_coords.x;
      self_pos[1] = (double)self_coords.y;
      self_pos[2] = (double)self_coords.h;

      // light cap
      if (light_log < lmn) {light_log=lmn;}
      if (light_log >lmx) {light_log=lmx;}      
      
      /* 
      Fmodes are use to define specific missions. 
      For this set of experiments, fmode==2 is the one used
      */
      if (_fmode == 1) {
        // Flocking mode
        kappa = 0.0;
        u_add = 0.05;
      }
      
      else if (_fmode == 2) {
        // Gradient following mode
        // Equation (2)
        su = sb + pow((light_log-lmn)/(lmx - lmn), 0.1)*sv;
        u_add = 0.05;  
        kappa = 0.0;  
      }

      else if (_fmode == 3) {
        // Go to goal mode
        _goal_x = paramGetFloat(paramIdgoalx);
        _goal_y = paramGetFloat(paramIdgoaly);
        gx = (double)_goal_x - self_pos[0];
        gy = (double)_goal_y - self_pos[1];
        gx = gx / (sqrt(gx*gx + gy*gy));
        gy = gy / (sqrt(gx*gx + gy*gy));
        u_add = 0.05;       
      }  

      sum_cosh = cos(self_pos[2]);
      sum_sinh = sin(self_pos[2]);

      for (int i = 0; i < 7; i++)
      {
        if ( neg_alive[i] && (neg_xs[i] > 0.0) && ((i+1) != my_id) )
        {
          // Computation of interagent distance and bearings
          distance = sqrt( ((self_pos[0] - neg_xs[i])*(self_pos[0] - neg_xs[i])) + ((self_pos[1] - neg_ys[i])*(self_pos[1] - neg_ys[i])) );
          ij_ang = atan2((neg_ys[i] - self_pos[1]), (neg_xs[i] - self_pos[0]));
          
          // Equation (1) Computing proximal control 
          temp_px = ((-epsilon * (( 2.*( pow(su, 4.) / pow(distance, 5.) ) ) - ( pow(su, 2.) / pow(distance, 3.) ) ) ) * cos(ij_ang) );
          temp_py = ((-epsilon * (( 2.*( pow(su, 4.) / pow(distance, 5.) ) ) - ( pow(su, 2.) / pow(distance, 3.) ) ) ) * sin(ij_ang) );

          px = px + temp_px;
          py = py + temp_py;

          sum_cosh += cos(neg_hs[i]);
          sum_sinh += sin(neg_hs[i]);
        }
      }

      // Equation (3) Alignment control vector
      hx = sum_cosh / (sqrt((sum_cosh*sum_cosh) + (sum_sinh*sum_sinh)));
      hy = sum_sinh / (sqrt((sum_cosh*sum_cosh) + (sum_sinh*sum_sinh)));

      // Equation (4) Boundary repulsion vector 
      if (self_pos[1]<0.5)
      {
        rx += krep * ( (1./fabs(self_pos[1])) - (1./L0) ) * ( cos(1.5708)/pow(fabs(self_pos[1]),3.) );
        ry += krep * ( (1./fabs(self_pos[1])) - (1./L0) ) * ( sin(1.5708)/pow(fabs(self_pos[1]),3.) );
      }
      if (self_pos[0]>6.5)
      {
        rx += krep * ( (1./(fabs(7.0-self_pos[0]))) - (1./L0) ) * ( cos(3.14)/pow((fabs(7.0-self_pos[0])),3.) );
        ry += krep * ( (1./(fabs(7.0-self_pos[0]))) - (1./L0) ) * ( sin(3.14)/pow((fabs(7.0-self_pos[0])),3.) );
      }
      if (self_pos[1]>4.25)
      {
        rx += krep * ( (1./(fabs(4.75-self_pos[1]))) - (1./L0) ) * ( cos(-1.5708)/pow((fabs(4.75-self_pos[1])),3.) );
        ry += krep * ( (1./(fabs(4.75-self_pos[1]))) - (1./L0) ) * ( sin(-1.5708)/pow((fabs(4.75-self_pos[1])),3.) );
      }
      if (self_pos[0]<0.5)
      {
        rx += krep * ( (1./(fabs(self_pos[0]))) - (1./L0) ) * ( cos(0.0)/pow((fabs(self_pos[0])),3.) );
        ry += krep * ( (1./(fabs(self_pos[0]))) - (1./L0) ) * ( sin(0.0)/pow((fabs(self_pos[0])),3.) );
      }

      // Getting sensor measurements from Multiranger deck and defining offsets 
      front_range = logGetFloat(ranger_front) - 150;
      back_range  = logGetFloat(ranger_back) - 150;
      right_range = logGetFloat(ranger_right) - 150;
      left_range  = logGetFloat(ranger_left) - 150;
      if (front_range<0) {front_range = 50;}    
      if (back_range<0) {back_range = 50;}  
      if (right_range<0) {right_range = 50;}
      if (left_range<0) {left_range = 50;}    

      // Equations (5), (6), (7) Obstacle avoidance vector
      //FRONT
      if (front_range < 300) 
      {
        random_addition = -0.5 + ((double)rand()/((double)RAND_MAX));  
        hx = 0.0;
        hy = 0.0;

        // Selecting the bias to break symmetry in favour of the swarm position
        front_bias_angle = atan2(py, px) - self_pos[2];

        if (front_bias_angle > 0)
        {
          // Equation (7) We add two vector forces here
          // This is one vector force. Pointing backwards (3.14 rads or 180 deg)
          ox += pow(2.5/((double)front_range/1000),2.) * cos(self_pos[2] + 3.14 + random_addition) * 0.5;
          oy += pow(2.5/((double)front_range/1000),2.) * sin(self_pos[2] + 3.14 + random_addition) * 0.5;

          // This is the second vector force. Pointing right (1.57 rads or 90 deg)
          ox += pow(2.5/((double)front_range/1000),2.) * cos(self_pos[2] + 1.57 + random_addition);
          oy += pow(2.5/((double)front_range/1000),2.) * sin(self_pos[2] + 1.57 + random_addition);
        }
        else
        {
          ox += pow(2.5/((double)front_range/1000),2.) * cos(self_pos[2] + 3.14 + random_addition) * 0.5;
          oy += pow(2.5/((double)front_range/1000),2.) * sin(self_pos[2] + 3.14 + random_addition) * 0.5;          
          ox += pow(2.5/((double)front_range/1000),2.) * cos(self_pos[2] + -1.57 + random_addition);
          oy += pow(2.5/((double)front_range/1000),2.) * sin(self_pos[2] + -1.57 + random_addition);  
        }      
      }

      // BACK
      if (back_range < 100) 
      {
        random_addition = -0.5 + ((double)rand()/((double)RAND_MAX));  
        // Equation (7) In this case only one vector is added
        ox += pow(2.5/((double)back_range/1000),2.) * cos(self_pos[2] + 0.0 + random_addition);
        oy += pow(2.5/((double)back_range/1000),2.) * sin(self_pos[2] + 0.0 + random_addition);
        hx = 0.0;
        hy = 0.0;        
      }

      // LEFT
      if (left_range < 300) 
      {
        random_addition = -0.5 + ((double)rand()/((double)RAND_MAX));  
        ox += pow(2.5/((double)left_range/1000),2.) * cos(self_pos[2] + -1.57 + random_addition);
        oy += pow(2.5/((double)left_range/1000),2.) * sin(self_pos[2] + -1.57 + random_addition);
        hx = 0.0;
        hy = 0.0;        
      }

      // RIGHT
      if (right_range < 300) 
      {
        random_addition = -0.5 + ((double)rand()/((double)RAND_MAX));  
        ox += pow(2.5/((double)right_range/1000),2.) * cos(self_pos[2] + 1.57 + random_addition);
        oy += pow(2.5/((double)right_range/1000),2.) * sin(self_pos[2] + 1.57 + random_addition);
        hx = 0.0;
        hy = 0.0;        
      }         

      // Equation (8) Total force vector
      fx_raw = alpha * px + beta * hx + gama * rx + kappa * gx + 2*ox;
      fy_raw = alpha * py + beta * hy + gama * ry + kappa * gy + 2*oy;

      fx = sqrt(fx_raw*fx_raw + fy_raw*fy_raw) * cos(atan2(fy_raw, fx_raw) - self_pos[2]);
      fy = sqrt(fx_raw*fx_raw + fy_raw*fy_raw) * sin(atan2(fy_raw, fx_raw) - self_pos[2]);

      // Equation (9) Linear and Angular speeds
      u = K1 * fx + u_add;
      w = K2 * fy;

      if (u>umax) {u = umax;}
      else if (u<0) {u = 0.0;}

      if (w>wmax) {w = wmax;}
      else if (w<-wmax) {w = -wmax;}  
     
     // Linear speed (u) is defined as the velocity in X axis
      vx = (float)u;
      vy = 0.0;
     
      w = w / 0.0174532;

      // Sending control commands to low level Controllers
      setHoverSetpoint(&setpoint, vx, vy, (float)fh, w);
      commanderSetSetpoint(&setpoint, 3);  

      // Communications stuff
      memcpy(p_reply.data, &self_coords, sizeof(self_coords));
      p_reply.size = sizeof(self_coords)+1;
      radiolinkSendP2PPacketBroadcast(&p_reply);
      ifterminate = paramGetInt(paramIdifterminate);
      _fmode = paramGetInt(paramIdfmode);
      ifheading = paramGetInt(paramIdifheading);
      _formation = paramGetInt(paramIdformation);

      // Data aggregation for next logging 
      agg_data_log = aggregate_data(self_pos[0], self_pos[1], self_pos[2], (double)light_log);

      vTaskDelay(M2T(50));

      // Setting a total flight time
      total_flight += 50;    
      if (total_flight > 400000 || ifterminate == 1) {
        state = land;
      }
    }

    // Our defined land sequence. 
    if(state == land) {
      crtpCommanderHighLevelLand(0.03, 3.0); 
      vTaskDelay(M2T(2000));
      return;
    }
  }
}
