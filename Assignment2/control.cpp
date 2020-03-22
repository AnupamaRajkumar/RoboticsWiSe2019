// controlDLL.cpp : Defines the entry point for the DLL application.
//
#include "servo.h"
#include "param.h"
#include "control.h"
//#include "UiAgent.h"
#include "PrVector.h"
#include "PrMatrix.h"
#include "Utils.h" // misc. utility functions, such as toRad, toDeg, etc.
#include <math.h>
#include <algorithm>
#include "forwardkinematics.hpp"

using std::min;
using std::max;

//structure for cubic splines
struct CubicSpline
{
	double t0, tf;
	double a0[3], a1[3], a2[3], a3[3];
};

CubicSpline spline;

struct CircleProps
{
	double rad, ang, centerX, centerY;
};

CircleProps cProps;

struct TimeComps
{
	float  startT, tb, th, tf, td;
};

TimeComps tComp;


/*Function to compute the total duration, tf, of the trajectory
 Using the approach of parabolic split joint*/
double computeTf(GlobalVariables& gv)
{
	PrVector3 time = PrVector3(0,0,0);
	float timeMax = 0;
	for(int i = 0; i < gv.dof; i++)
	{
	        time[i] = gv.curTime+2*(gv.dqmax[i]/gv.ddqmax[i])+((gv.qd[i]-gv.q[i])/gv.dqmax[i]);
		/*Find the max time amongst all the joints*/
		if(time[i] > timeMax)
		{
		  timeMax = time[i];		
		}
	}
	return timeMax;
}

void PrintDebug(GlobalVariables& gv);

// *******************************************************************
// Initialization functions
// *******************************************************************

void InitControl(GlobalVariables& gv) 
{
   // This code runs before the first servo loop
}

void PreprocessControl(GlobalVariables& gv)
{
   // This code runs on every servo loop, just before the control law
   
    float m1, m2, m3;
    float l1, l2, l3;
    float r1, r2, r3;
    float g;
    PrVector3 tauM1 = PrVector3(0,0,0);
    PrVector3 tauM2 = PrVector3(0,0,0);
    PrVector3 tauM3 = PrVector3(0,0,0);
    PrVector3 tauM  = PrVector3(0,0,0);

  // 
    if ((gv.dof == 3) || (gv.dof == 6)) {

        //get the correct joint angles depending on the current mode:
        double q1,q2,q3;
        if (gv.dof == 3) {
            q1 = gv.q[0];
            q2 = gv.q[1];
            q3 = gv.q[2];
        } else if (gv.dof == 6) {
            q1 = gv.q[1];
            q2 = gv.q[2];
            q3 = gv.q[4];
        }

        PrVector3 g123 = PrVector3(0,0,0); //Variable that holds the torque exerted by gravity for each joint

        //Compute g123 here!  
	r1 = R2;
        r2 = 0.189738;
        r3 = R6;

	l1 = L2;
	l2 = L3;
	l3 = L6;

	m1 = M2;
	m2 = M3+M4+M5;
	m3 = M6;

	g = GRAVITY;
        //force exterted by joint 1
	tauM1[0] = (m1*g*r1*cos(q1));
	tauM1[1] = 0;
	tauM1[2] = 0;

	//force exerted by joint 2
	tauM2[0] = ((m2*g*l1*cos(q1))+(m2*g*r2*sin(q1+q2)));
	tauM2[1] = (m2*g*r2*sin(q1+q2));
	tauM2[2] = 0;

	//force exerted by joint 3
	tauM3[0] = ((m3*g*l1*cos(q1))+(m3*g*l2*sin(q1+q2))+(m3*g*r3*sin(q1+q2+q3)));
	tauM3[1] = (m3*g*l2*sin(q1+q2))+(m3*g*r3*sin(q1+q2+q3));
	tauM3[2] = (m3*g*r3*sin(q1+q2+q3));

	//adding tauM1, tauM2 and tauM3 for gravity vector
	tauM[0] = tauM1[0]+tauM2[0]+tauM3[0];
	tauM[1] = tauM1[1]+tauM2[1]+tauM3[1];
	tauM[2] = tauM1[2]+tauM2[2]+tauM3[2];
	
        //gravity vector
	g123 = PrVector3(tauM[0], tauM[1], tauM[2]);
         

        //maps the torques to the right joint indices depending on the current mode:
        if (gv.dof == 3) {
            gv.G[0] = g123[0];
            gv.G[1] = g123[1];
            gv.G[2] = g123[2];
        } else if (gv.dof == 6) {
            gv.G[1] = g123[0];
            gv.G[2] = g123[1];
            gv.G[4] = g123[2];
        }

//        printVariable(g123, "g123");
    } else {
        gv.G = PrVector(gv.G.size());
    } 

      
}

void PostprocessControl(GlobalVariables& gv) 
{
   // This code runs on every servo loop, just after the control law
}

void initFloatControl(GlobalVariables& gv) 
{
    // Control Initialization Code Here
}

void initOpenControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initJholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjmoveControl(GlobalVariables& gv) 
{
	/*int val = 0;
	for(val = 0; val < gv.dof; val++){
		gv.kp[val] = 100;	
	}*/
	
}

void initJmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initJgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNjtrackControl(GlobalVariables& gv) 
{
	/*initializing joint time*/
	spline.t0 = gv.curTime;
	/*Computing total duration*/
	spline.tf = computeTf(gv);
	
	/*computing splines*/
	for(int i=0;i<gv.dof;i++)
	{
		spline.a0[i] = gv.q[i];
		spline.a1[i] = 0.;
		spline.a2[i] = (3*(gv.qd[i]-gv.q[i]))/pow((spline.tf-gv.curTime),2);
		spline.a3[i] = -(2*(gv.qd[i]-gv.q[i]))/pow((spline.tf-gv.curTime),3);
	}
	
}

void initJtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNxtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initXtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNholdControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initHoldControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initNgotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initGotoControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initNtrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initTrackControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initPfmoveControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
} 

void initLineControl(GlobalVariables& gv) 
{
	// Control Initialization Code Here
}

void initProj1Control(GlobalVariables& gv) 
{
	/*Initializing the joints to (10.2, 52.2, -62.4)*/	
	gv.qd[0] = (10.2 * M_PI)/180;
	gv.qd[1] = (52.2 * M_PI)/180;
	gv.qd[2] = (-62.4 * M_PI)/180;
	/*Calculating splines*/
	initNjtrackControl(gv);
}

void initProj2Control(GlobalVariables& gv) 
{
	/*Initializing the circle properties*/	
	cProps.rad 	= 0.2;		//Radius of the given circle
	cProps.ang	= (2*M_PI)/5;	//Angular Velocity 
	cProps.centerX	= 0.6;		//X coordinate of the center
	cProps.centerY	= 0.3;		//Y coordinate of the center
	tComp.startT	= gv.curTime;	//Current time
}

void initProj3Control(GlobalVariables& gv) 
{
	tComp.startT	= gv.curTime;	//Current time	
	tComp.tb 	= 5.;		//first portion of blend
	tComp.tf 	= 20.;		//final time of blend
	tComp.th 	= 10.;		//time at highest point
}


// *******************************************************************
// Control laws
// *******************************************************************

void noControl(GlobalVariables& gv)
{
}

void floatControl(GlobalVariables& gv)
{   
	/*int i;
	for (i = 0; i < gv.dof; i++){
		gv.tau[i] = gv.G[i];
	}*/
	gv.tau = gv.G;	
	
	//PrintDebug(gv);
}

void openControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void njholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void jholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void njmoveControl(GlobalVariables& gv)
{
   /*implement P controller*/
   gv.tau = gv.kp*(gv.qd-gv.q);
}

void jmoveControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void njgotoControl(GlobalVariables& gv) 
{	
   gv.tau = gv.kp*(gv.qd-gv.q)+gv.G;
}

void jgotoControl(GlobalVariables& gv) 
{
  
   gv.tau = gv.kp*(gv.qd-gv.q)-(gv.kv*gv.dq)+gv.G;

}

/*Function to plot the trajectory*/
void njtrackControl(GlobalVariables& gv) 
{
  
      PrVector3 actVel;
      float timeDiff = 0;
      
      timeDiff = (gv.curTime-spline.t0); 
      /*Implement the trajectory with PD control until total duration of the
	trajectory. Otherwise implement float control*/
      if(gv.curTime <= (spline.tf)){
      for(int i = 0; i < gv.dof; i++)
	{
	  /*implementing the trajectory*/	
	  gv.qd[i] = spline.a0[i] + spline.a1[i]*timeDiff + spline.a2[i]*timeDiff*timeDiff + spline.a3[i]*timeDiff*timeDiff*timeDiff;
	  /*calculating actual velocity*/
	  actVel[i] = 2*spline.a2[i]*timeDiff + 3*spline.a3[i]*timeDiff*timeDiff;
	  /*Implementing the PD control*/
	  gv.tau[i] = -gv.kp[i]*((gv.q[i]-gv.qd[i]))-(gv.kv[i]*(gv.dq[i]-actVel[i]))+gv.G[i];
        }
      }
      else
      {
	/*Implementing float control*/
	 floatControl(gv);
      }
}

void jtrackControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void nxtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void xtrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void nholdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void holdControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void ngotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void gotoControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void ntrackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void trackControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void pfmoveControl(GlobalVariables& gv) 
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void lineControl(GlobalVariables& gv)
{
   floatControl(gv);  // Remove this line when you implement this controller
}

void proj1Control(GlobalVariables& gv) 
{
	njtrackControl(gv);
}

void proj2Control(GlobalVariables& gv) 
{
	/*Applying time correction so that the increasing current time can be compensated with the start time*/
	float timeDiff = (gv.curTime - tComp.startT);
	/*Equation of the desired Position on the trajectory*/							
	gv.xd = PrVector3(-cos((cProps.ang)*timeDiff)*(cProps.rad) + cProps.centerX,-sin((cProps.ang)*timeDiff)*(cProps.rad) + cProps.centerY,double (0));
	/*Equation of desired Velocity on the trajectory which is a derivative of gv.xd*/	
	gv.dxd = PrVector3(sin((cProps.ang)*timeDiff)*(cProps.rad)*(cProps.ang),-cos((cProps.ang)*timeDiff)*(cProps.rad)*(cProps.ang),double (0));		
	/*Calculating force*/
	PrVector F = -gv.kp*(gv.x-gv.xd)-gv.kv*(gv.dx-gv.dxd);	
	/*Applying inverse kinematics*/			
	gv.tau = gv.Jtranspose*F + gv.G;						
}

void proj3Control(GlobalVariables& gv) 
{
	float t = gv.curTime;
	float timeDiff = 0.;
	float tp = tComp.tf-tComp.tb;
	/*given: max angular acceleration*/
	double maxAcc = (2.*M_PI)/25.;
	float angVel;
	float angPos;
	timeDiff = t-tComp.startT;
	
	if(timeDiff < tComp.tb)
	{		
		//cout<<"In first blend\n";		
		angVel = maxAcc*(timeDiff);	
		angPos = 0.5*maxAcc*(timeDiff)*(timeDiff);

	}
	if((timeDiff >= tComp.tb )&&(timeDiff < tp ))
	{	
		//cout<<"In second blend\n";			
		angVel = (2 * M_PI/5);
		angPos = (timeDiff*angVel) - M_PI;
	
	}
	if((timeDiff >= tp)&&( timeDiff < tComp.tf))
	{
		//cout<<"In third blend\n";
		angVel = (4 * M_PI/10) - (maxAcc* (timeDiff-tComp.tf));
		angPos =  (6 * M_PI) - (maxAcc*(tComp.tf-timeDiff)*(tComp.tf-timeDiff));	
	}
	
	/*Equation of the desired Position on the trajectory*/	
	gv.xd[0]	= (float)(0.6+0.2*cos(angPos));
	gv.xd[1]	= (float)(0.3-sin(angPos));
	gv.xd[2]	= 0.;
	//cout<<"gv.xd[0]:"<<gv.xd[0]<<"gv.xd[1]:"<<gv.xd[1]<<"gv.xd[2]:"<<gv.xd[2]<<"\n";
	/*Equation of desired Velocity on the trajectory which is a derivative of gv.xd*/
	gv.dxd[0]	= -0.2*angVel*sin(angPos);
	gv.dxd[1]	= -0.2*angVel*cos(angPos);
	gv.dxd[2]	= 0.;
	//cout<<"gv.dxd[0]:"<<gv.dxd[0]<<"gv.dxd[1]:"<<gv.dxd[1]<<"gv.dxd[2]:"<<gv.dxd[2]<<"\n";
	/*Calculating force*/
	PrVector F = -gv.kp*(gv.x-gv.xd)-gv.kv*(gv.dx-gv.dxd);	
	/*Applying inverse kinematics*/			
	gv.tau = gv.Jtranspose*F + gv.G;	
	
}

// *******************************************************************
// Debug function
// *******************************************************************

void PrintDebug(GlobalVariables& gv)
{
   // Replace this code with any debug information you'd like to get
   // when you type "pdebug" at the prompt.f
   printf( "This sample code prints the torque and mass\n" );
   gv.tau.display( "tau" );
   gv.A.display( "A" );
}

#ifdef WIN32
// *******************************************************************
// XPrintf(): Replacement for printf() which calls ui->VDisplay()
// whenever the ui object is available.  See utility/XPrintf.h.
// *******************************************************************

int XPrintf( const char* fmt, ... )
{
  int returnValue;
  va_list argptr;
  va_start( argptr, fmt );

  returnValue = vprintf( fmt, argptr );

  va_end( argptr );
  return returnValue;
}
#endif //#ifdef WIN32

/********************************************************

END OF DEFAULT STUDENT FILE 

ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS 

*******************************************************/
