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
 * LocomotionControl.h
 *
 *  Created on: Jul 1, 2013
 *      Author: lrm
 */

#ifndef LOCOMOTIONCONTROL_H_
#define LOCOMOTIONCONTROL_H_

//* general includes ---------------------------------------------
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>


//* other shared library includes --------------------------------
#include <generic_rover_manoeuvre/GenericRoverManoeuvre.h>
#include "LocomotionControlTypes.h"

/**
 * List of platform driving modes. So far, this are the modes supported by the Generic Manoeuvre Library.
 */
enum PltfDrivingMode
{
	STOPPED_WHEELS,
	STRAIGHT_LINE,
	ACKERMAN,
	SPOT_TURN,
	SKID_TURN,
	WHEEL_WALKING,
	DIRECT_DRIVE
};

/**
 * List of all can nodes in the platform.
 * Include groups of nodes in order to create a Drive class object to address them altogether.
 */
enum MotorCommandNode
{
	COMMAND_WHEEL_DRIVE_FL
	,COMMAND_WHEEL_DRIVE_FR
	,COMMAND_WHEEL_DRIVE_CL
	,COMMAND_WHEEL_DRIVE_CR
	,COMMAND_WHEEL_DRIVE_BL
	,COMMAND_WHEEL_DRIVE_BR
	,COMMAND_WHEEL_STEER_FL
	,COMMAND_WHEEL_STEER_FR
	,COMMAND_WHEEL_STEER_BL
	,COMMAND_WHEEL_STEER_BR
	,COMMAND_WHEEL_WALK_FL
	,COMMAND_WHEEL_WALK_FR
	,COMMAND_WHEEL_WALK_CL
	,COMMAND_WHEEL_WALK_CR
	,COMMAND_WHEEL_WALK_BL
	,COMMAND_WHEEL_WALK_BR
	,COMMAND_WHEEL_DRIVE_GROUP
	,COMMAND_WHEEL_STEER_GROUP
	,COMMAND_WHEEL_WALK_GROUP
};

/**
 * List of platform driving modes. So far, this are the modes supported by the Generic Manoeuvre Library.
 */
enum MotorControlMode
{
	MODE_POSITION,
	MODE_SPEED,
	UNSET_COMMAND
};


struct MotorCommand{
	double pos;
	double vel;
	int mode;
};



class LocomotionControl
{
public:

	/**
	 * Default constructor.
	 */
	LocomotionControl();

	/**
	 * Default destructor.
	 */
	~LocomotionControl();

	/**
	 * Set rover parameters that are used by the Generic Rover Manoeuvre Library. Structure defined in LocomotionControlTypes.h
	 * @param parameters Structure containing rover parameters.
	 */
	void setRoverParams(generic_rover_param parameters);

	/**
	 * Handles the transition in between different driving modes of the rover platform.
	 * Makes sure that the necessary initial conditions for driving in a given mode are met.
	 * @param Mode from the enum list PltfDrivingMode to be set
	 * @return True if mode was set and initial conditions were met.
	 */
	bool setDrivingMode(PltfDrivingMode mode);

	/**
	 * **** Generic Manoeuvre Library ****
	 * Drives the rover platform in straight linear motion (forwards or backwards depending on the velocity sign).
	 * @param dVelocity Is the velocity in m/s at which the rover shall be commanded
	 */
	void pltfDriveStraightVelocity(double dVelocity);

	/**
	 * **** Generic Manoeuvre Library ****
	 * Drives the rover platform to perform a Generic Ackerman manoeuvre (forwards or backwards depending on the velocity sign).
	 * @param dVelocity Is the velocity in m/s at which the rover shall be commanded
	 * @param dRotationCenter is the Center of Rotation of the Generic Ackerman manoeuvre. {x,y} position values
	 * @param dPointToControl is the Point to Control of the Generic Ackerman manoeuvre. {x,y} position values
	 */
	void pltfDriveGenericAckerman(double dVelocity, double *dRotationCenter, double *dPointToControl);

	/**
	 * **** Generic Manoeuvre Library ****
	 * Drives the rover platform to perform a Spot Turn (left or right depending on the velocity sign).
	 * @param dVelocity Is the velocity in radian/s at which the rover shall be commanded
	 */
	void pltfDriveSpotTurn(double dAngularVelocity);

	/**
	 * **** Generic Manoeuvre Library ****
	 * Drives the rover platform to perform a Skid Turn (left or right depending on the sign of the radius of curvature).
	 * @param dVelocity Is the velocity in cm/s at which the rover shall be commanded
	 * @param dRadiusOfCurvature Is the Radius of Curvature of the turning motion
	 * @param iIsStraightLine determines if a straight line motion is desired
	 */
	void pltfDriveSkidTurn(double dVelocity, double dRadiusOfCurvature, int iIsStraightLine);

	/**
	 * **** Generic Manoeuvre Library ****
	 * Drives the rover platform to perform a Wheel Walking manoeuvre. **** under development ****
	 * @param dStepLength
	 * @param iGait is the wheel walk gait to be used for the manoeuvre
	 */
	void pltfDriveWheelWalk(double *dStepLength, int iGait);

        /**
         * **** EGRESS TESTS ADDITIONAL METHODS ****
         * @param angle is the angle at which the deployment/walking motors are set
         */
        void pltfBemaDeploy(double angle);
        void pltfWalkingDeploy(double angle);

	/**
	 * Direct Drive mode motion commands. Use these functions in direct drive control mode only. They control one single motor at a time. Use for drive testing.
	 * @param iWheel or iJoint selects the CAN node motor to be commanded.
	 * @param dVelocity or dAngle is the command for the motor.
	 */
	void directWheelDriveVelocityDegS(int iWheel, double dVelocity);
	void directWheelSteerAngleDeg(int iWheel, double dAngle);
	void directWheelWalkJointAngleDeg(int iJoint, double dAngle);

	PltfDrivingMode m_DrivingMode;							/**< Stores the current driving mode of the rover platform. Default: STOPPED_WHEELS */

	//* Generic Manoeuvre Library variables
	ROVER_PARAM MyRover;									/**< Rover parameters structure */
	double m_dWheelVelocity[NUM_WHEEL_ROVER_MAX];					/**< Wheel driving velocity commands */
	double m_dWheelSteering[NUM_WHEEL_ROVER_MAX];					/**< Wheel steering position commands */
	double m_dWalkAngleRad[NUM_WHEEL_ROVER_MAX];						/**< Walking angle commands */
	double m_dWheelAngleRad[NUM_WHEEL_ROVER_MAX];					/**< Wheel driving position commands (wheel walking manoeuvre) */
	double m_dWheelRadius;
	int m_iNumWheels;

	std::vector<MotorCommand> commands;

};


#endif /* LOCOMOTIONCONTROL_H_ */
