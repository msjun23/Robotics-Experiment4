#include "stdafx.h"
#include "ODE.h"
#include "SystemMemory.h"
#include "DataType.h"

#ifndef DRAWSTUFF_TEXTURE_PATH
#define DRAWSTUFF_TEXTURE_PATH "./../ode-0.13/drawstuff/textures"
#endif

#define GRAVITY 9.81
#define MAX_JOINT_NUM 3

#define DEG2RAD 0.0174533
#define RAD2DEG 57.2958

#define _X 0
#define _Y 1
#define _Z 2

#define _MASS 0
#define _DIR 1
#define _RADIUS 2
#define _LENGTH 3

dsFunctions g_Fn;

static dWorldID g_World;
static dSpaceID g_Space;
static dJointGroupID g_Contactgroup;

Object g_oObj[MAX_JOINT_NUM + 1];
static dJointID g_oJoint[MAX_JOINT_NUM + 1];

double g_tar_q[MAX_JOINT_NUM] = { 0.0, 0.0 };
double g_cur_q[MAX_JOINT_NUM] = { 0.0, 0.0 };

// Body & Joint array
dJointID	g_Joint[MAX_JOINT_NUM];
Object		g_Link[MAX_JOINT_NUM];
dGeomID		g_Ground;

void InitDrawStuff() {

	g_Fn.version = DS_VERSION;
	g_Fn.start = &StartDrawStuff;
	g_Fn.step = &SimLoopDrawStuff;
	g_Fn.command = &CommandDrawStuff;
	g_Fn.stop = StopDrawStuff;
	g_Fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
}


void InitODE() {

	dInitODE();
	g_World = dWorldCreate();
	g_Space = dHashSpaceCreate(0);
	g_Contactgroup = dJointGroupCreate(0);
	dWorldSetGravity(g_World, 0, 0, -GRAVITY);
	dWorldSetCFM(g_World, 1e-5);
	//dWorldSetAutoDisableFlag(g_World, 1);
}



void RunODE(size_t width, size_t height) {

	InitDrawStuff();
	InitODE();

	InitRobot();

	// run simulation
	dsSimulationLoop(0, 0, width, height, &g_Fn);
}



void ReleaseODE() {

	dJointGroupDestroy(g_Contactgroup);
	dSpaceDestroy(g_Space);
	dWorldDestroy(g_World);
	dCloseODE();
}



void StartDrawStuff() {

	//TO DO

}


void SimLoopDrawStuff(int pause)
{
	//////////////////////////////////////////////////////////////////////////
	// To Do
	DataType_t joint_data;
	GET_SYSTEM_MEMORY("JointData", joint_data);

	g_tar_q[0] = joint_data.Q_tar[0];
	g_tar_q[1] = joint_data.Q_tar[1];
	joint_data.Q_cur[0] = g_cur_q[0];
	joint_data.Q_cur[1] = g_cur_q[1];

	SET_SYSTEM_MEMORY("JointData", joint_data);

	if (g_tar_q[0] >= 360.0 * DEG2RAD) g_tar_q[0] -= 360.0 * DEG2RAD;
	if (g_tar_q[0] <= -360.0 * DEG2RAD) g_tar_q[0] += 360.0 * DEG2RAD;

	//////////////////////////////////////////////////////////////////////////
	// To Do HW4: 2-DOF Manipulator

	PControl();

	float full_color = 255.0;

	dsSetColor(109 / full_color, 227 / full_color, 191 / full_color);			// RGB
	dsDrawCapsuleD(dBodyGetPosition(g_Link[0].body), dBodyGetRotation(g_Link[0].body), 0.25, 0.145);		// Pos, Rotation, length, radius

	dsSetColor(152 / full_color, 240 / full_color, 213 / full_color);			// RGB
	dsDrawCapsuleD(dBodyGetPosition(g_Link[1].body), dBodyGetRotation(g_Link[1].body), 1.0, 0.135);		// Pos, Rotation, length, radius

	dsSetColor(198 / full_color, 250 / full_color, 231 / full_color);			// RGB
	dsDrawCapsuleD(dBodyGetPosition(g_Link[2].body), dBodyGetRotation(g_Link[2].body), 0.5, 0.125);	// Pos, Rotation, length, radius

	double dt = 0.01;
	dWorldStep(g_World, dt);

	//////////////////////////////////////////////////////////////////////////
}



void CommandDrawStuff(int cmd) {

	//TO DO

}



void StopDrawStuff() {

	//TO DO

}


void InitRobot()
{
	//////////////////////////////////////////////////////////////////////////
	// Create Body
	dMass mass;
	dReal pos[3][3] = { { 0.0, 0.0, 0.125 },
						{ 0.0, 0.0, 0.75 },
						{ 0.0, 0.0, 1.5 } };				// x, y, z
	dReal prop[3][4] = { { 0.5, 3, 0.145, 0.25},
						 { 1.0, 3, 0.135, 1.0},
						 { 0.5, 3, 0.125, 0.5} };			// mass, direction, radius, length

	for (int i = 0; i < MAX_JOINT_NUM; i++) {				// 2 Joints, 3 Links
		g_Link[i].body = dBodyCreate(g_World);				// Create i`th body & Return ID
		dBodySetPosition(g_Link[i].body, pos[i][_X],
			pos[i][_Y],
			pos[i][_Z]);									// body ID, x, y, z
		dMassSetZero(&mass);								// Initialize mass
		dMassSetCapsuleTotal(&mass, prop[i][_MASS],
			prop[i][_DIR],
			prop[i][_RADIUS],
			prop[i][_LENGTH]);								// mass, direction, radius, length
		dBodySetMass(g_Link[i].body, &mass);				// Set body mass
		g_Link[i].geom = dCreateCapsule(g_Space,
			prop[i][_RADIUS],
			prop[i][_LENGTH]);								// radius, length
		dGeomSetBody(g_Link[i].geom, g_Link[i].body);		// Create geometry
	}

	// Create Joint
	g_Joint[0] = dJointCreateFixed(g_World, 0);				// fixed joint at world
	dJointAttach(g_Joint[0], 0, g_Link[0].body);			// joint ID, prev body(0 = ground), next body
	dJointSetFixed(g_Joint[0]);								// Set fixed joint

	for (int i = 1; i < MAX_JOINT_NUM; i++) {
		g_Joint[i] = dJointCreateHinge(g_World, 0);			// body(i - 1) to body(i) hinge joint
		dJointAttach(g_Joint[i], g_Link[i - 1].body,
			g_Link[i].body);								// joint ID, prev body(i - 1), current body
		dJointSetHingeAnchor(g_Joint[i], pos[i][_X],
			pos[i][_Y],
			pos[i][_Z] - prop[i][_LENGTH] / 2);				// joint pos
		dJointSetHingeAxis(g_Joint[i], 1, 0, 0);			// joint rotation dir
	}

	//////////////////////////////////////////////////////////////////////////
}

void PControl()
{
	//////////////////////////////////////////////////////////////////////////
	// To Do HW4: 2-DOF Manipulator

	// The index of each array matches the index of the joint, 
	// and index 0 is a fixed joint to the earth.
	double theta_cur[] = { 0, dJointGetHingeAngle(g_Joint[1]),
							  dJointGetHingeAngle(g_Joint[2]) };	// Current Angle of each joint
	double theta_ref[] = { 0, g_tar_q[0], 
							  g_tar_q[1] };							// Reference Angle of each joint
	double theta_err[MAX_JOINT_NUM] = { 0, };						// Error of each joint angle

	g_cur_q[0] = theta_cur[1];
	g_cur_q[1] = theta_cur[2];

	for (int i = 1; i < MAX_JOINT_NUM; i++) {
		theta_err[i] = theta_ref[i] - theta_cur[i];					// error = reference - current
	}

	double Kp = 0.5;
	double vel_ref[MAX_JOINT_NUM];									// Velocity reference for control angle
	for (int i = 1; i < MAX_JOINT_NUM; i++) {
		vel_ref[i] = Kp * theta_err[i];								// P Contorller

		dJointSetHingeParam(g_Joint[i], dParamVel, vel_ref[i]);		// Set hinge parameter: velocity
		dJointSetHingeParam(g_Joint[i], dParamFMax, 100);			// Set hinge parameter: Max force
	}

	//////////////////////////////////////////////////////////////////////////
}
