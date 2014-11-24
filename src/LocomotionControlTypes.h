/*
 * LocomotionControlTypes.h
 *
 *  Created on: Jul 16, 2013
 *      Author: lrm
 */

#ifndef LOCOMOTIONCONTROLTYPES_H_
#define LOCOMOTIONCONTROLTYPES_H_

#define NUM_WHEEL_ROVER 6


struct generic_rover_param
{
	//! number of wheels of the rover
	int WheelNumber;

	//! position of walking wheels (1 -> walk, 0 -> no walk)
	int IsWalkingWheel[NUM_WHEEL_ROVER];
	//! position of steering wheels (1 -> steer, 0 -> no steer)
	int IsSteeringWheel[NUM_WHEEL_ROVER];
	//! position of driving wheels (1 -> drive, 0 -> no drive)
	int IsDrivingWheel[NUM_WHEEL_ROVER];

	//! radius of each wheel of the rover, in [m]
	double WheelRadius[NUM_WHEEL_ROVER];
	//! wheel Cartesian coordinates in 2D, in [m] (X [0] and Y [1])  in the rover frame
	double WheelCoordCart[2*NUM_WHEEL_ROVER];
	//! wheel polar coordinates, in [m] and [rad] (D [0] and ALPHA [1]) in the rover frame
	//double WheelCoordPol[2*NUM_WHEEL_ROVER];
	//! difference of height (Z axis) in [m] between wheel centre and walking joint centre
	double LegLengthV[NUM_WHEEL_ROVER];
	//! cartesian coordinates of the walking joints, in [m] (X [0] and Y [1])  in the rover frame
	double WalkCoordCart[2*NUM_WHEEL_ROVER];
	//! polar coordinates of the walking joints, in [m] and [rad] (D [0] and ALPHA [1]) in the rover frame
	//double WalkCoordPol[2*NUM_WHEEL_ROVER];
};


#endif /* LOCOMOTIONCONTROLTYPES_H_ */
