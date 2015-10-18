/****************************************************************
 *
 * Copyright (c) 2013
 *
 * European Space Technology and Research Center
 * ESTEC - European Space Agency
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: ROBS
 * stack name: MotionControl
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Martin Azkarate, email:martin.azkarate@esa.int
 * Supervised by: Pantelis Poulakis, email:pantelis.poulakis@esa.int
 *
 * Date of creation: Jul 2013
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

/*
 * LocomotionControl.cpp
 *
 *  Created on: Jul 1, 2013
 *      Author: lrm
 */

/** Math only use in the cpp **/
#include <math.h>

#include "LocomotionControl.h"


//-----------------------------------------------
LocomotionControl::LocomotionControl()
{
	m_DrivingMode = STOPPED_WHEELS;
}

LocomotionControl::~LocomotionControl()
{
	//nothing to delete
}

void LocomotionControl::setRoverParams(generic_rover_param p)
{
	//* Generic Rover Manoeuvre parameters
	double MyInitParam[] = {
	p.WheelNumber,				// number of wheels

	p.IsWalkingWheel[0],p.IsWalkingWheel[1],p.IsWalkingWheel[2],
	p.IsWalkingWheel[3],p.IsWalkingWheel[4],p.IsWalkingWheel[5], 			// position of walking wheels (1 -> walk, 0 -> no walk)
	p.IsSteeringWheel[0],p.IsSteeringWheel[1],p.IsSteeringWheel[2],
	p.IsSteeringWheel[3],p.IsSteeringWheel[4],p.IsSteeringWheel[5],			// position of steering wheels (1 -> steer, 0 -> no steer)
	p.IsDrivingWheel[0],p.IsDrivingWheel[1],p.IsDrivingWheel[2],
	p.IsDrivingWheel[3],p.IsDrivingWheel[4],p.IsDrivingWheel[5],			// position of driving wheels (1 -> drive, 0 -> no drive)

	p.WheelRadius[0],p.WheelRadius[1],p.WheelRadius[2],
	p.WheelRadius[3],p.WheelRadius[4],p.WheelRadius[5],						// radius of the wheels

	0,														// indicate type of coordinates for wheel: 0 CARTESIAN, 1 POLAR, 2 BOTH
	p.WheelCoordCart[0],p.WheelCoordCart[1],p.WheelCoordCart[2],
	p.WheelCoordCart[3],p.WheelCoordCart[4],p.WheelCoordCart[5],
	p.WheelCoordCart[6],p.WheelCoordCart[7],p.WheelCoordCart[8],			// wheel Cartesian coordinate FLx, FLy, FRx, FRy, CLx, CLy,
	p.WheelCoordCart[9],p.WheelCoordCart[10],p.WheelCoordCart[11],			// wheel Cartesian coordinate CRx, CRy, RLx, RLy, RRx, RRy

	p.LegLengthV[0],p.LegLengthV[1],p.LegLengthV[2],
	p.LegLengthV[3],p.LegLengthV[4],p.LegLengthV[5],						// leg length

	0,														// indicate type of coordinates for walk: 0 CARTESIAN, 1 POLAR, 2 BOTH
	p.WalkCoordCart[0],p.WalkCoordCart[1],p.WalkCoordCart[2],
	p.WalkCoordCart[3],p.WalkCoordCart[4],p.WalkCoordCart[5],
	p.WalkCoordCart[6],p.WalkCoordCart[7],p.WalkCoordCart[8],				// walk Cartesian coordinate FLx, FLy, FRx, FRy, CLx, CLy,
	p.WalkCoordCart[9],p.WalkCoordCart[10],p.WalkCoordCart[11],				// walk Cartesian coordinate CRx, CRy, RLx, RLy, RRx, RRy

	};

	m_iNumWheels=p.WheelNumber;
	m_dWheelRadius=p.WheelRadius[0];



	/*
	//! Generic Rover Manoeuvre parameters
	double MyInitParam[] = {
	6, 									// number of wheels

	1,1,1,1,1,1,						// position of walking wheels (1 -> walk, 0 -> no walk)
	1,1,0,0,1,1,						// position of steering wheels (1 -> steer, 0 -> no steer)
	1,1,1,1,1,1,						// position of driving wheels (1 -> drive, 0 -> no drive)

	.075,.075,.075,.075,.075,.075,		// radius of the wheels

	0,									// indicate type of coordinates for wheel: 0 CARTESIAN, 1 POLAR, 2 BOTH
	.265,.305,							// wheel Cartesian coordinate FL
	.265,-.305,							// FR
	0,.305,								// CL
	0,-.305,							// CR
	-.265,.305,							// RL
	-.265,-.305,						// RR

	.120,.120,.120,.120,.120,.120,		// leg length

	0,									// indicate type of coordinates for walk: 0 CARTESIAN, 1 POLAR, 2 BOTH
	.265,.305,							// wheel Cartesian coordinate FL
	.265,-.305,							// FR
	0,.305,								// CL
	0,-.305,							// CR
	-.265,.305,							// RL
	-.265,-.305,						// RR

	};
	*/

	RoverInit( &MyRover, MyInitParam, sizeof(MyInitParam) / sizeof(MyInitParam[0]) );

}

bool LocomotionControl::setDrivingMode(PltfDrivingMode mode)
{
	PltfDrivingMode newDrivingMode = mode;

	switch (newDrivingMode)
	{
		case STOPPED_WHEELS:
			commands[COMMAND_WHEEL_DRIVE_GROUP].vel=0;
			commands[COMMAND_WHEEL_DRIVE_GROUP].mode=MODE_SPEED;
			break;
		case STRAIGHT_LINE:
			commands[COMMAND_WHEEL_DRIVE_GROUP].vel=0;
			commands[COMMAND_WHEEL_DRIVE_GROUP].mode=MODE_SPEED;
			
			commands[COMMAND_WHEEL_STEER_FL].pos=0;
			commands[COMMAND_WHEEL_STEER_FL].vel=0;
			commands[COMMAND_WHEEL_STEER_FL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_STEER_FR].pos=0;
			commands[COMMAND_WHEEL_STEER_FR].vel=0;
			commands[COMMAND_WHEEL_STEER_FR].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_STEER_BL].pos=0;
			commands[COMMAND_WHEEL_STEER_BL].vel=0;
			commands[COMMAND_WHEEL_STEER_BL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_STEER_BR].pos=0;
			commands[COMMAND_WHEEL_STEER_BR].vel=0;
			commands[COMMAND_WHEEL_STEER_BR].mode=MODE_POSITION;

			commands[COMMAND_WHEEL_WALK_FL].pos=0;
			commands[COMMAND_WHEEL_WALK_FL].vel=0;
			commands[COMMAND_WHEEL_WALK_FL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_FR].pos=0;
			commands[COMMAND_WHEEL_WALK_FR].vel=0;
			commands[COMMAND_WHEEL_WALK_FR].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_CL].pos=0;
			commands[COMMAND_WHEEL_WALK_CL].vel=0;
			commands[COMMAND_WHEEL_WALK_CL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_CR].pos=0;
			commands[COMMAND_WHEEL_WALK_CR].vel=0;
			commands[COMMAND_WHEEL_WALK_CR].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_BL].pos=0;
			commands[COMMAND_WHEEL_WALK_BL].vel=0;
			commands[COMMAND_WHEEL_WALK_BL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_BR].pos=0;
			commands[COMMAND_WHEEL_WALK_BR].vel=0;
			commands[COMMAND_WHEEL_WALK_BR].mode=MODE_POSITION;
			break;
		case ACKERMAN:
			commands[COMMAND_WHEEL_DRIVE_GROUP].vel=0;
			commands[COMMAND_WHEEL_DRIVE_GROUP].mode=MODE_SPEED;
			
			commands[COMMAND_WHEEL_WALK_FL].pos=0;
			commands[COMMAND_WHEEL_WALK_FL].vel=0;
			commands[COMMAND_WHEEL_WALK_FL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_FR].pos=0;
			commands[COMMAND_WHEEL_WALK_FR].vel=0;
			commands[COMMAND_WHEEL_WALK_FR].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_CL].pos=0;
			commands[COMMAND_WHEEL_WALK_CL].vel=0;
			commands[COMMAND_WHEEL_WALK_CL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_CR].pos=0;
			commands[COMMAND_WHEEL_WALK_CR].vel=0;
			commands[COMMAND_WHEEL_WALK_CR].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_BL].pos=0;
			commands[COMMAND_WHEEL_WALK_BL].vel=0;
			commands[COMMAND_WHEEL_WALK_BL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_BR].pos=0;
			commands[COMMAND_WHEEL_WALK_BR].vel=0;
			commands[COMMAND_WHEEL_WALK_BR].mode=MODE_POSITION;
			break;
		case SPOT_TURN:
			commands[COMMAND_WHEEL_DRIVE_GROUP].vel=0;
			commands[COMMAND_WHEEL_DRIVE_GROUP].mode=MODE_SPEED;
			
			commands[COMMAND_WHEEL_WALK_FL].pos=0;
			commands[COMMAND_WHEEL_WALK_FL].vel=0;
			commands[COMMAND_WHEEL_WALK_FL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_FR].pos=0;
			commands[COMMAND_WHEEL_WALK_FR].vel=0;
			commands[COMMAND_WHEEL_WALK_FR].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_CL].pos=0;
			commands[COMMAND_WHEEL_WALK_CL].vel=0;
			commands[COMMAND_WHEEL_WALK_CL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_CR].pos=0;
			commands[COMMAND_WHEEL_WALK_CR].vel=0;
			commands[COMMAND_WHEEL_WALK_CR].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_BL].pos=0;
			commands[COMMAND_WHEEL_WALK_BL].vel=0;
			commands[COMMAND_WHEEL_WALK_BL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_BR].pos=0;
			commands[COMMAND_WHEEL_WALK_BR].vel=0;
			commands[COMMAND_WHEEL_WALK_BR].mode=MODE_POSITION;
			break;
		case SKID_TURN:
			commands[COMMAND_WHEEL_DRIVE_GROUP].vel=0;
			commands[COMMAND_WHEEL_DRIVE_GROUP].mode=MODE_SPEED;
			
			commands[COMMAND_WHEEL_STEER_FL].pos=0;
			commands[COMMAND_WHEEL_STEER_FL].vel=0;
			commands[COMMAND_WHEEL_STEER_FL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_STEER_FR].pos=0;
			commands[COMMAND_WHEEL_STEER_FR].vel=0;
			commands[COMMAND_WHEEL_STEER_FR].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_STEER_BL].pos=0;
			commands[COMMAND_WHEEL_STEER_BL].vel=0;
			commands[COMMAND_WHEEL_STEER_BL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_STEER_BR].pos=0;
			commands[COMMAND_WHEEL_STEER_BR].vel=0;
			commands[COMMAND_WHEEL_STEER_BR].mode=MODE_POSITION;

			commands[COMMAND_WHEEL_WALK_FL].pos=0;
			commands[COMMAND_WHEEL_WALK_FL].vel=0;
			commands[COMMAND_WHEEL_WALK_FL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_FR].pos=0;
			commands[COMMAND_WHEEL_WALK_FR].vel=0;
			commands[COMMAND_WHEEL_WALK_FR].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_CL].pos=0;
			commands[COMMAND_WHEEL_WALK_CL].vel=0;
			commands[COMMAND_WHEEL_WALK_CL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_CR].pos=0;
			commands[COMMAND_WHEEL_WALK_CR].vel=0;
			commands[COMMAND_WHEEL_WALK_CR].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_BL].pos=0;
			commands[COMMAND_WHEEL_WALK_BL].vel=0;
			commands[COMMAND_WHEEL_WALK_BL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_WALK_BR].pos=0;
			commands[COMMAND_WHEEL_WALK_BR].vel=0;
			commands[COMMAND_WHEEL_WALK_BR].mode=MODE_POSITION;
			break;
		case WHEEL_WALKING:
			commands[COMMAND_WHEEL_DRIVE_GROUP].vel=0;
			commands[COMMAND_WHEEL_DRIVE_GROUP].mode=MODE_SPEED;
		
			commands[COMMAND_WHEEL_STEER_FL].pos=0;
			commands[COMMAND_WHEEL_STEER_FL].vel=0;
			commands[COMMAND_WHEEL_STEER_FL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_STEER_FR].pos=0;
			commands[COMMAND_WHEEL_STEER_FR].vel=0;
			commands[COMMAND_WHEEL_STEER_FR].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_STEER_BL].pos=0;
			commands[COMMAND_WHEEL_STEER_BL].vel=0;
			commands[COMMAND_WHEEL_STEER_BL].mode=MODE_POSITION;
			commands[COMMAND_WHEEL_STEER_BR].pos=0;
			commands[COMMAND_WHEEL_STEER_BR].vel=0;
			commands[COMMAND_WHEEL_STEER_BR].mode=MODE_POSITION;
			break;
		case DIRECT_DRIVE:
			commands[COMMAND_WHEEL_DRIVE_GROUP].vel=0;
			commands[COMMAND_WHEEL_DRIVE_GROUP].mode=MODE_SPEED;
			break;
	}

	m_DrivingMode = newDrivingMode;
	std::cout << "Driving mode set to: " << m_DrivingMode << std::endl;
	return true;
}

void LocomotionControl::pltfDriveStraightVelocity(double dVelocity)
{
	if (m_DrivingMode!=STRAIGHT_LINE){
		std::cout << "Trying to drive straight without being in straight line mode. Exiting without driving..." << std::endl;
		return;
	}

	double dVelRadS = dVelocity/m_dWheelRadius;
        commands[COMMAND_WHEEL_DRIVE_GROUP].vel=dVelRadS;
	commands[COMMAND_WHEEL_DRIVE_GROUP].mode=MODE_SPEED;
}

void LocomotionControl::pltfDriveGenericAckerman(double dVelocity, double *dRotationCenter, double *dPointToControl)
{
	if (m_DrivingMode!=ACKERMAN){
		std::cout << "Trying to drive Ackerman without being in Ackerman mode. Exiting without driving..." << std::endl;
		return;
	}

	if(GenericAckermann( &MyRover,
		dVelocity,
		dRotationCenter,
		dPointToControl,
		m_dWheelSteering,
		m_dWheelVelocity ))
	{
		std::cout << "Error in GenericAckerman function. Exiting without driving..." << std::endl;
		return;
	}

	commands[COMMAND_WHEEL_STEER_FL].pos=m_dWheelSteering[0];
	commands[COMMAND_WHEEL_STEER_FL].vel=0;
	commands[COMMAND_WHEEL_STEER_FL].mode=MODE_POSITION;

	commands[COMMAND_WHEEL_STEER_FR].pos=m_dWheelSteering[1];
	commands[COMMAND_WHEEL_STEER_FR].vel=0;
	commands[COMMAND_WHEEL_STEER_FR].mode=MODE_POSITION;

	commands[COMMAND_WHEEL_STEER_BL].pos=m_dWheelSteering[4];
	commands[COMMAND_WHEEL_STEER_BL].vel=0;
	commands[COMMAND_WHEEL_STEER_BL].mode=MODE_POSITION;

	commands[COMMAND_WHEEL_STEER_BR].pos=m_dWheelSteering[5];
	commands[COMMAND_WHEEL_STEER_BR].vel=0;
	commands[COMMAND_WHEEL_STEER_BR].mode=MODE_POSITION;

	for (int i=0; i<m_iNumWheels;i++)
	{
		commands[COMMAND_WHEEL_DRIVE_FL+i].vel=m_dWheelVelocity[i];
		commands[COMMAND_WHEEL_DRIVE_FL+i].mode=MODE_SPEED;
	}
}

void LocomotionControl::pltfDriveSpotTurn(double dAngularVelocity)
{
	if (m_DrivingMode!=SPOT_TURN){
		std::cout << "Trying to drive Spot Turn without being in Spot Turn mode. Exiting without driving..." << std::endl;
		return;
	}

	if (SpotTurn( &MyRover,
		dAngularVelocity,
		m_dWheelSteering,
		m_dWheelVelocity ))
	{
		std::cout << "Error in SpotTurn function. Exiting without driving..." << std::endl;
		return;
	}

	commands[COMMAND_WHEEL_STEER_FL].pos=m_dWheelSteering[0]-M_PI;
	commands[COMMAND_WHEEL_STEER_FL].vel=0;
	commands[COMMAND_WHEEL_STEER_FL].mode=MODE_POSITION;

	commands[COMMAND_WHEEL_STEER_FR].pos=m_dWheelSteering[1];
	commands[COMMAND_WHEEL_STEER_FR].vel=0;
	commands[COMMAND_WHEEL_STEER_FR].mode=MODE_POSITION;

	commands[COMMAND_WHEEL_STEER_BL].pos=m_dWheelSteering[4]+M_PI;
	commands[COMMAND_WHEEL_STEER_BL].vel=0;
	commands[COMMAND_WHEEL_STEER_BL].mode=MODE_POSITION;

	commands[COMMAND_WHEEL_STEER_BR].pos=m_dWheelSteering[5];
	commands[COMMAND_WHEEL_STEER_BR].vel=0;
	commands[COMMAND_WHEEL_STEER_BR].mode=MODE_POSITION;

	commands[COMMAND_WHEEL_DRIVE_FL].vel=-m_dWheelVelocity[0];
	commands[COMMAND_WHEEL_DRIVE_FL].mode=MODE_SPEED;
	commands[COMMAND_WHEEL_DRIVE_FR].vel=m_dWheelVelocity[1];
	commands[COMMAND_WHEEL_DRIVE_FR].mode=MODE_SPEED;
	commands[COMMAND_WHEEL_DRIVE_CL].vel=-m_dWheelVelocity[2];
	commands[COMMAND_WHEEL_DRIVE_CL].mode=MODE_SPEED;
	commands[COMMAND_WHEEL_DRIVE_CR].vel=m_dWheelVelocity[3];
	commands[COMMAND_WHEEL_DRIVE_CR].mode=MODE_SPEED;
	commands[COMMAND_WHEEL_DRIVE_BL].vel=-m_dWheelVelocity[4];
	commands[COMMAND_WHEEL_DRIVE_BL].mode=MODE_SPEED;
	commands[COMMAND_WHEEL_DRIVE_BR].vel=m_dWheelVelocity[5];
	commands[COMMAND_WHEEL_DRIVE_BR].mode=MODE_SPEED;

}

void LocomotionControl::pltfDriveSkidTurn(double dVelocity, double dRadiusOfCurvature, int iIsStraightLine)
{
	if (m_DrivingMode!=SKID_TURN){
		std::cout << "Trying to drive Skid Turn without being in Skid Turn mode. Exiting without driving..." << std::endl;
		return;
	}

	if (SkidTurn(
		&MyRover,
		dVelocity,
		dRadiusOfCurvature,
		iIsStraightLine,
		m_dWheelVelocity ))
	{
		std::cout << "Error in SkidTurn function. Exiting without driving..." << std::endl;
		return;
	}

	for (int i=0; i<m_iNumWheels;i++)
	{
		commands[COMMAND_WHEEL_DRIVE_FL+i].vel=m_dWheelVelocity[i];
		commands[COMMAND_WHEEL_DRIVE_FL+i].mode=MODE_SPEED;
	}
}

void LocomotionControl::pltfDriveWheelWalk(double *dStepLength, int iGait)
{
	if (m_DrivingMode!=WHEEL_WALKING){
		std::cout << "Trying to drive Wheel Walk without being in Wheel Walking mode. Exiting without driving..." << std::endl;
		return;
	}

	if (WheelWalk( &MyRover,
		dStepLength,
		iGait,
		m_dWalkAngleRad,
		m_dWheelAngleRad
		))
	{
		std::cout << "Error in WheelWalk function. Exiting without driving..." << std::endl;
		return;
	}

	std::cout << "WheelWalk function not implemented yet in Generic Manoeuvre Library. Exiting without driving..." << std::endl;

	/* Commented for safety until a validated code for wheel walking capabilities is implemented in the Generic Manoeuvre Library
	for (int i=0; i<m_iNumWheels;i++)
	{
		nodePositionSetPointRad(CANNODE_WHEEL_DRIVE_FL+i,m_dWheelAngleRad[i],0);
		nodePositionSetPointRad(CANNODE_WHEEL_WALK_FL+i,m_dWalkAngleRad[i],0);
	}
	nodeCommandSetPoint(CANNODE_WHEEL_DRIVE_GROUP);
	nodeCommandSetPoint(CANNODE_WHEEL_WALK_GROUP);
	*/
}

void LocomotionControl::pltfBemaDeploy(double angle)
{
        for (int i=0;i<m_iNumWheels;i++)
        {
            commands[COMMAND_WHEEL_WALK_FL+i].pos=angle;
            commands[COMMAND_WHEEL_WALK_FL+i].mode=MODE_POSITION;
        }
}

void LocomotionControl::pltfWalkingDeploy(double angle)
{
        for (int i=0;i<2;i++)
        {
            commands[COMMAND_WHEEL_WALK_FL+i].pos=angle;
            commands[COMMAND_WHEEL_WALK_FL+i].mode=MODE_POSITION;
        }
}

//-----------------------------------------------
void LocomotionControl::directWheelDriveVelocityDegS(int iWheel, double dVelocity)
{
	if (m_DrivingMode!=DIRECT_DRIVE){
		std::cout << "Trying to Direct Drive without being in Direct Drive mode. Exiting without driving..." << std::endl;
		return;
	}
	double dRadS = dVelocity*M_PI/180;
	commands[iWheel].vel=dRadS;
	commands[iWheel].mode=MODE_SPEED;
}

//-----------------------------------------------
void LocomotionControl::directWheelSteerAngleDeg(int iWheel, double dAngle)
{
	if (m_DrivingMode!=DIRECT_DRIVE){
		std::cout << "Trying to Direct Drive without being in Direct Drive mode. Exiting without driving..." << std::endl;
		return;
	}
	double dRad = dAngle*M_PI/180;
	commands[iWheel].pos=dRad;
	commands[iWheel].vel=0;
	commands[iWheel].mode=MODE_POSITION;
}

//-----------------------------------------------
void LocomotionControl::directWheelWalkJointAngleDeg(int iJoint, double dAngle)
{
	if (m_DrivingMode!=DIRECT_DRIVE){
		std::cout << "Trying to Direct Drive without being in Direct Drive mode. Exiting without driving..." << std::endl;
		return;
	}
	double dRad = dAngle*M_PI/180;
	commands[iJoint].pos=dRad;
	commands[iJoint].vel=0;
	commands[iJoint].mode=MODE_POSITION;

}


