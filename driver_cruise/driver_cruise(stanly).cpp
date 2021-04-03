/*
     WARNING !

     DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
//#include <windows.h>
#endif

#include "driver_cruise.h"
#include <cstdio>
//#include <fstream>
#include <ostream>

#define PI 3.141592653589793238462643383279

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float *cmdAcc, float *cmdBrake, float *cmdSteer, int *cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_cruise(tModInfo *modInfo)
{
    memset(modInfo, 0, 10 * sizeof(tModInfo));
    modInfo[0].name = "driver_cruise";               // name of the module (short).
    modInfo[0].desc = "user module for CyberCruise"; // Description of the module (can be long).
    modInfo[0].fctInit = InitFuncPt;                 // Init function.
    modInfo[0].gfId = 0;
    modInfo[0].index = 0;
    return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
    tUserItf *itf = (tUserItf *)pt;
    itf->userDriverGetParam = userDriverGetParam;
    itf->userDriverSetParam = userDriverSetParam;
    return 0;
}

/*
     WARNING!
     DO NOT MODIFY CODES ABOVE!
*/

static float _midline[200][2];                           //
int mid;                                                 //
static float _yaw, _yawrate, _speed, _acc, _width, _rpm; //
static int _gearbox, counter = 0, accCounter = 0;
double horizonXRate;

bool parameterSet = false; //
void PIDParamSetter();     //

//******************************************************//
typedef struct Circle //
{                     //
    double r;         //
    int sign;         //
} circle;             //

//********************PID parameters*************************//
double kp_s; //kp for speed							     //
double ki_s; //ki for speed							     //
double kd_s; //kd for speed							     //
double kp_d; //kp for direction						     //
double ki_d; //ki for direction					    	 //
double kd_d; //kd for direction						     //
// Direction Control Variables						         //
double D_err;         //direction error					             //
double D_errDiff = 0; //direction difference(Differentiation) //
double D_errSum = 0;  //sum of direction error(Integration)      //
// Speed Control Variables//
double curSpeedErr;      //speed error
double speedErrDiff = 0; //speed difference(Differentiation)
double speedErrSum = 0;  //sum of speed error(Integration)
double tmpSpeedErr = 0;
double expectedSpeed; //
                      //
circle c;             //
int startPoint;       //
int delta = 10;       //
//***********************************************************//

//*******************Other parameters*******************//
const int topGear = 7; //
double tmp;            //
bool flag = true;      //
double offset = 0; //
double Tmp = 0;
int count = 0;
bool isDirt = 0;
double midLineOffset = 0;
double stanley = 0;
//******************************************************//

//******************************Helping Functions*******************************//
// Function updateGear:															//
//		Update Gear automatically according to the current speed.				//
//		Implemented as Schmitt trigger.											//
void updateGear(int *cmdGear); //
// Function constrain:															//
//		Given input, outputs value with boundaries.								//
double constrain(double lowerBoundary, double upperBoundary, double input); //
// Function getR:																//
//		Given three points ahead, outputs a struct circle.						//
//		{radius:[1,500], sign{-1:left,1:right}									//
circle getR(float x1, float y1, float x2, float y2, float x3, float y3); //
float getV(float r);

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm)
{
    /* write your own code here */

    for (int i = 0; i < 200; ++i)
        _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
    _yaw = yaw;
    _yawrate = yawrate;
    _speed = speed;
    _acc = acc;
    _width = width;
    _rpm = rpm;
    _gearbox = gearbox;
}

float getV(float r)
{
    if (r > 150)
        return 185 + 130 / (1.0 + exp((200 - r) / 15));
    else
        return 90 + 95 / (1.0 + exp((100.0 - r) / 15));
}

static void userDriverSetParam(float *cmdAcc, float *cmdBrake, float *cmdSteer, int *cmdGear)
{
    if (parameterSet == false) // Initialization Part
    {
        PIDParamSetter();
    }
    else
    {
        //根据加速到30km/h的时间不同区分公路与土路
        if (_speed < 30)
        {
            *cmdAcc = 1;
            *cmdBrake = 0;
            *cmdGear = 1;
            accCounter++;
        }

        if (accCounter < 45)
        {
            isDirt = 0;
        }
        else
            isDirt = 1;

        //公路
        if (!isDirt)
        {
            if (_speed < 250)
                startPoint = _speed * 0.305;
            else
                startPoint = _speed * 0.5;
            c = getR(_midline[startPoint][0], _midline[startPoint][1], _midline[startPoint + delta][0], _midline[startPoint + delta][1], _midline[startPoint + 5 + 1 * delta][0], _midline[startPoint + 5 + 1 * delta][1]);
            if (c.r <= 50)
            {
                //expectedSpeed = constrain(100, 120, c.r * c.r * (-0.046) + c.r * 5.3 - 50.66);
                //expectedSpeed = constrain(c.r + 60, 120, c.r * c.r * (-0.046) + c.r * 5.3 - 50.66);
                expectedSpeed = constrain(c.r + 80, 130, c.r * c.r * (-0.046) + c.r * 6.3 - 50.66);
            }
            else
            {
                expectedSpeed = constrain(130, 300, c.r * 1.6);
            }

            if (_speed > 90)
            {
                mid = _speed / 4;
            }
            else
                mid = 20;

            curSpeedErr = expectedSpeed - _speed;
            speedErrSum = 0.1 * speedErrSum + curSpeedErr;
            if (curSpeedErr > 0) //lack speed
            {
                if (abs(*cmdSteer) < 0.6) //-1.0 <= *cmdSteer <=  1.0; when *cmdSteer is small
                {
                    //printf("*cmdSteer small\t");
                    *cmdAcc = constrain(0.0, 1.0, kp_s * curSpeedErr + ki_s * speedErrSum + offset);
                    *cmdBrake = 0;
                }
                else if (abs(*cmdSteer) > 0.70) //when *cmdSteer is large
                {
                    //printf("*cmdSteer large\t");
                    *cmdAcc = 0.005 + offset;
                    *cmdBrake = 0;
                }
                else //when *cmdSteer is in the middle ( 0.6 to 0.7 )
                {
                    //printf("*cmdSteer middle\t");
                    *cmdAcc = 0.11 + offset;
                    *cmdBrake = 0;
                }
            }
            else if (curSpeedErr < 0)		//overspeed
            {
                *cmdBrake = constrain(0.0, 0.8, -kp_s * curSpeedErr / 5 - offset / 3);
                *cmdAcc = 0;
            }

            updateGear(cmdGear);

            // Direction Control
            //set the param of PID controller
            kp_d = 10;
            ki_d = 0;
            kd_d = 40;

            //std::fstream file;
            //FILE *fp;

            D_err = -atan2(_midline[mid][0], _midline[mid][1]); //only track the aiming point on the middle line

            //the differential and integral operation
            D_errDiff = D_err - Tmp;
            D_errSum = D_errSum + D_err;
            Tmp = D_err;

            //set the error and get the cmdSteer
            if (_midline[0][0] < 0)
                midLineOffset = sqrt(_midline[0][0] * _midline[0][0] + _midline[0][1] * _midline[0][1]);
            else
                midLineOffset = -sqrt(_midline[0][0] * _midline[0][0] + _midline[0][1] * _midline[0][1]);

            stanley = _yaw + atan(250 * midLineOffset / _speed);
            *cmdSteer = constrain(-1.0, 1.0, stanley + kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);
            //*cmdSteer = constrain(-1.0, 1.0, kp_d * D_err + 11 / c.r + 2 * _yaw);

            //print some useful info on the terminal
            // printf("公路");
            printf("D_err : %f \n", D_err);
            printf("cmdSteer %f \n", *cmdSteer);
        }

        //土路
        else
        {
            if (_speed < 250)
                startPoint = _speed * 0.305;
            else
                startPoint = _speed * 0.5;
            c = getR(_midline[startPoint][0], _midline[startPoint][1], _midline[startPoint + delta][0], _midline[startPoint + delta][1], _midline[startPoint + 5 + 1 * delta][0], _midline[startPoint + 5 + 1 * delta][1]);
            if (c.r <= 50)
            {
                //expectedSpeed = constrain(100, 120, c.r * c.r * (-0.046) + c.r * 5.3 - 50.66);
                //expectedSpeed = constrain(c.r + 60, 120, c.r * c.r * (-0.046) + c.r * 5.3 - 50.66);
                expectedSpeed = constrain(90, 120, c.r * c.r * (-0.046) + c.r * 5.3 - 50.66);
            }
            //else if(c.r <= 60)
            //{
            //    expectedSpeed = constrain(c.r + 65, 120, c.r * c.r * (-0.046) + c.r * 5.3 - 50.66);
            //}
            //else if (c.r <= 70)
            //{
            //    expectedSpeed = constrain(c.r + 70, 120, c.r * c.r * (-0.046) + c.r * 5.3 - 50.66);
            //}
            else
            {
                expectedSpeed = constrain(120, 300, c.r * 1.4);
            }
            if (_speed > 120)
            {
                mid = _speed / 3;
            }
            else
                mid = 40;

            curSpeedErr = expectedSpeed - _speed;
            speedErrSum = 0.1 * speedErrSum + curSpeedErr;
            if (curSpeedErr > 0) //lack speed
            {
                if (abs(*cmdSteer) < 0.6) //-1.0 <= *cmdSteer <=  1.0; when *cmdSteer is small
                {
                    //printf("*cmdSteer small\t");
                    *cmdAcc = constrain(0.0, 1.0, kp_s * curSpeedErr + ki_s * speedErrSum + offset);
                    *cmdBrake = 0;
                }
                else if (abs(*cmdSteer) > 0.70) //when *cmdSteer is large
                {
                    //printf("*cmdSteer large\t");
                    *cmdAcc = 0.005 + offset;
                    *cmdBrake = 0;
                }
                else //when *cmdSteer is in the middle ( 0.6 to 0.7 )
                {
                    //printf("*cmdSteer middle\t");
                    *cmdAcc = 0.11 + offset;
                    *cmdBrake = 0;
                }
            }
            else if (curSpeedErr < 0) //overspeed
            {
                *cmdBrake = constrain(0.0, 0.8, -kp_s * curSpeedErr / 5 - offset / 3);
                *cmdAcc = 0;
            }

            updateGear(cmdGear);

            // Direction Control
            //set the param of PID controller
            kp_d = 10;
            ki_d = 0;
            kd_d = 50;

            //std::fstream file;
            //FILE *fp;

            D_err = -atan2(_midline[mid][0], _midline[mid][1]); //only track the aiming point on the middle line

            //the differential and integral operation
            D_errDiff = D_err - Tmp;
            D_errSum = D_errSum + D_err;
            Tmp = D_err;

            //set the error and get the cmdSteer
            if (_midline[0][0] < 0)
                midLineOffset = sqrt(_midline[0][0] * _midline[0][0] + _midline[0][1] * _midline[0][1]);
            else
                midLineOffset = -sqrt(_midline[0][0] * _midline[0][0] + _midline[0][1] * _midline[0][1]);

            stanley =  0.1 * _yaw + atan(250 * midLineOffset / _speed);
            *cmdSteer = constrain(-1.0, 1.0, stanley + kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff );
            // *cmdSteer = constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff + 3 / c.r + 0.1 * _yaw);
            //*cmdSteer = constrain(-1.0, 1.0, kp_d * D_err + 11 / c.r + 2 * _yaw);

            //print some useful info on the terminal
            // printf("土路");
            printf("D_err : %f \n", D_err);
            printf("cmdSteer %f \n", *cmdSteer);
        }
    }
}

void PIDParamSetter()
{
    kp_s = 0.07;
    ki_s = 0.01;
    kd_s = 0.02;
    kp_d = 10;
    ki_d = 0.01;
    kd_d = 60;
    parameterSet = true;
}

void updateGear(int *cmdGear)
{
    if (_gearbox == 1)
    {
        if (_speed >= 60 && topGear > 1)
        {
            *cmdGear = 2;
        }
        else
        {
            *cmdGear = 1;
        }
    }
    else if (_gearbox == 2)
    {
        if (_speed <= 45)
        {
            *cmdGear = 1;
        }
        else if (_speed >= 105 && topGear > 2)
        {
            *cmdGear = 3;
        }
        else
        {
            *cmdGear = 2;
        }
    }
    else if (_gearbox == 3)
    {
        if (_speed <= 90)
        {
            *cmdGear = 2;
        }
        else if (_speed >= 145 && topGear > 3)
        {
            *cmdGear = 4;
        }
        else
        {
            *cmdGear = 3;
        }
    }
    else if (_gearbox == 4)
    {
        if (_speed <= 131)
        {
            *cmdGear = 3;
        }
        else if (_speed >= 187 && topGear > 4)
        {
            *cmdGear = 5;
        }
        else
        {
            *cmdGear = 4;
        }
    }
    else if (_gearbox == 5)
    {
        if (_speed <= 173)
        {
            *cmdGear = 4;
        }
        else if (_speed >= 234 && topGear > 5)
        {
            *cmdGear = 6;
        }
        else
        {
            *cmdGear = 5;
        }
    }
    else if (_gearbox == 6)
    {
        if (_speed <= 219)
        {
            *cmdGear = 5;
        }
        else if (_speed >= 264 && topGear > 6)
        {
            *cmdGear = 7;
        }
        else
        {
            *cmdGear = 6;
        }
    }
    else if (_gearbox == 7)
    {
        if (_speed <= 252)
        {
            *cmdGear = 6;
        }
        else
        {
            *cmdGear = 7;
        }
    }
    else
    {
        *cmdGear = 1;
    }
}

double constrain(double lowerBoundary, double upperBoundary, double input)
{
    if (input > upperBoundary)
        return upperBoundary;
    else if (input < lowerBoundary)
        return lowerBoundary;
    else
        return input;
}

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
    double a, b, c, d, e, f;
    double r, x, y;

    a = 2 * (x2 - x1);
    b = 2 * (y2 - y1);
    c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
    d = 2 * (x3 - x2);
    e = 2 * (y3 - y2);
    f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;
    x = (b * f - e * c) / (b * d - e * a);
    y = (d * c - a * f) / (b * d - e * a);
    r = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
    x = constrain(-1000.0, 1000.0, x);
    y = constrain(-1000.0, 1000.0, y);
    r = constrain(1.0, 500.0, r);
    int sign = (x > 0) ? 1 : -1;
    circle tmp = {r, sign};
    return tmp;
}