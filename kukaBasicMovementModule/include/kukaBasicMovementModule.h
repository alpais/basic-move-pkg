/*
 * Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author:  Lucia Pais
 * email:   lucia.pais@epfl.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef kukaBasicMovementModule_H_
#define kukaBasicMovementModule_H_

#include "RobotLib/RobotInterface.h"
#include "RobotLib/WorldObject.h"
#include "RobotLib/KinematicChain.h"
#include "MathLib/IKGroupSolver.h"
#include "KUKARobotModel/LWRRobot.h"
#include "MotionGenerators/CDDynamics.h"

#define MODE_NONE  0
#define MODE_HOME  1
#define MODE_REACH 2


class kukaBasicMovementModule : public RobotInterface
{
protected:
// ===================================================
// ========  Define Your Robot ====== ================
// ===================================================
    LWRRobot* mLWRRobot;
    Robot::ControlMode ctrlmode;

    RevoluteJointSensorGroup    mSensorsGroup;
    RevoluteJointActuatorGroup  mActuatorsGroup;
    CDDynamics *genCart; 		// moving in cartesian space
    CDDynamics *genJoint;		// moving in joint space
    CDDynamics *mCDJointFilter; 	// smoother for joint space commands
    Matrix4 mHTARGETinABS;
    Vector3 currentTarget;

// ===================================================
// ========  Inverse Kinematics Stuff ================
// ===================================================
   	KinematicChain  mKinChain;
   	IKGroupSolver   mIKSolver;
   	IndicesVector   mJointMapping;
   	Vector		mJointVelLimits[2];
   	Vector          vCurrJoint;

// ===================================================
// ========  Target Object Variables  ================
// ===================================================
	WorldObject*    oTargetObj;             // the object we're trying to reach
	Matrix4		mTargetFrame;		// the target reference frame
	Matrix4         mTargetRelativeFrame;   // the frame relative to the target
	Vector 		vTarget;		// the target as it should be set for CDDynamics (x, y, z, wx, wy, wz)

// ===================================================
// ========  Robot Position Variables ================
// ===================================================
    int nEndEffectorId;

	Vector3 mRobotEEAbsolutePos;			// Robot's position in absolute coordinates
	Matrix3 mRobotEEAbsoluteOrient;			// Robot's orientation in absolute coordinates
	Matrix4 mRobotEEAbsoluteFrame;
	Vector3 vRobotEEPosInTargetFrame;  		// Robot's position in the target frame
	Vector3 vRobotEEOrientInTargetFrame;		// Robot's orientation in the target frame
	Matrix4	mFullEEInTargetFrame;			// Robot's full EE position and orientation in target frame

	//Vector3 vPosInTargetFrame
	Vector3	vPositionError3;				// Position error
	Vector3 vPosErrMult;					// Multiplying factors for position error
	Vector3	vOrientationError3;				// Orientation error
	Vector3 vOrientErrMult;					// Multiplying factors for orientation error


	REALTYPE reachingThreshold;				// threshold for reaching the target

	bool    bSync;						// True if the current robot joint state should be sinked in CDDynamics
	bool	bSyncCart;					// True if the current robot cartesian state should be sinked in CDDynamics
	int 	mState;						// the state index for the commands to their real time loop corresponding behavior

// ===================================================

public:
            kukaBasicMovementModule();
    virtual ~kukaBasicMovementModule();
  
    void initializeVariables();
    void setTargetToObject();


    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
};



#endif 
