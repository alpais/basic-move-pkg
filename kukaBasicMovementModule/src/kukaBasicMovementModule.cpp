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

#include "kukaBasicMovementModule.h"

double module_dt = 0.002;
double fri_dt = 0.002;


kukaBasicMovementModule::kukaBasicMovementModule()
:RobotInterface(){
}
kukaBasicMovementModule::~kukaBasicMovementModule(){
}

RobotInterface::Status kukaBasicMovementModule::RobotInit(){
	// ===================================================
	// ========  Initialize Robot and CTRL mode ==========
	// ===================================================
	if (mRobot->IsSimulationMode())
		mLWRRobot = new LWRRobot;
	else
		mLWRRobot = (LWRRobot*) mRobot;

	ctrlmode = Robot::CTRLMODE_POSITION;
	mSensorsGroup.SetSensorsList(mRobot->GetSensors());
	mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());

	// ===================================================
	// ========  Add Console Commands ====================
	// ===================================================
	AddConsoleCommand("home");
	AddConsoleCommand("go");

	nEndEffectorId = mRobot->GetLinkIndex("EndEffector");
	cout << "The end effector id is " << nEndEffectorId << endl;
	if (nEndEffectorId == -1)
		GetConsole()->Print("ERROR: End effector not found");

	// ===================================================
	// ========  Initialize Inverse Kinematics ===========
	// ===================================================
	mKinChain.SetRobot(mRobot);
	mKinChain.Create(0, 0, nEndEffectorId);
	mJointMapping = mKinChain.GetJointMapping();
	mIKSolver.SetSizes(mKinChain.GetDOFCount());
	mIKSolver.AddSolverItem(6); 				// One solver with 6 constraints (x,y,z, wx, wy, wz)
	mIKSolver.SetVerbose(false); 				// No comments
	mIKSolver.SetThresholds(0.0001, 0.0001); 		// Singularities thresholds
	mIKSolver.Enable(true, 0); 				// Enable first solver
	mIKSolver.SetDofsIndices(mJointMapping, 0); 		// Joint maps for first solver
	mJointVelLimits[0].Resize(mKinChain.GetDOFCount());
	mJointVelLimits[1].Resize(mKinChain.GetDOFCount());
	mJointVelLimits[0] = DEG2RAD(-60.0);
	mJointVelLimits[1] = DEG2RAD( 60.0);
	mIKSolver.SetLimits(mJointVelLimits[0], mJointVelLimits[1]);
	vCurrJoint.Resize(mRobot-> GetDOFCount());

	initializeVariables();

    return STATUS_OK;
}
RobotInterface::Status kukaBasicMovementModule::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status kukaBasicMovementModule::RobotStart(){

	Vector3 tmppos;Matrix3 tmporient;
	((LWRRobot*)mRobot)->GetMeasuredCartPose(tmppos, tmporient);
	tmppos.Print();
	tmporient.Print();

    return STATUS_OK;
}    
RobotInterface::Status kukaBasicMovementModule::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status kukaBasicMovementModule::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status kukaBasicMovementModule::RobotUpdateCore(){
	if (mRobot->GetControlMode() != ctrlmode) {
		mRobot->SetControlMode(ctrlmode);
		bSync = true;
	}
	// ===================================================
	// ========  Initialize                    ===========
	// ===================================================
	if (bSync) {
		mSensorsGroup.ReadSensors();
		vCurrJoint = mSensorsGroup.GetJointAngles();
		mActuatorsGroup.SetJointAngles(vCurrJoint);
		mActuatorsGroup.WriteActuators();
		mSensorsGroup.SetJointAngles(vCurrJoint);
		mSensorsGroup.WriteSensors();

		// Set state for the CDDynamics joint filter
		mCDJointFilter->SetState(vCurrJoint);

		// Set State for the CDDynamics in Joint Space
		Vector robotDOF;
		robotDOF.Resize(mRobot->GetDOFCount());

		for (int i = 0; i < sizeof(robotDOF); i++)
			robotDOF[i] = vCurrJoint[mJointMapping[i]];
		genJoint->SetState(robotDOF);

		// Set State for CDDynamics in Cartesian Space
		// we actually do this when we assign a new target, no need to do it here then

		bSync = false;
		return STATUS_OK;
	}

	switch (mState) {
	case MODE_NONE:
		break;
	case MODE_HOME:		// Moves the robot in a home position using a joint position controller
	{
		Vector  vJointCommand;
		vJointCommand.Resize(mRobot->GetDOFCount());
		vJointCommand[mJointMapping[0]] = DEG2RAD(0);
		vJointCommand[mJointMapping[1]] = DEG2RAD(10);
		vJointCommand[mJointMapping[2]] = DEG2RAD(0);
		vJointCommand[mJointMapping[3]] = DEG2RAD(-90);
		vJointCommand[mJointMapping[4]] = DEG2RAD(0);
		vJointCommand[mJointMapping[5]] = DEG2RAD(45);
		vJointCommand[mJointMapping[6]] = DEG2RAD(45);

		genJoint->SetTarget(vJointCommand);
		genJoint->Update();
		genJoint->GetState(vJointCommand);

		mActuatorsGroup.SetJointAngles(vJointCommand);
		mActuatorsGroup.WriteActuators();

		break;
	}
	case MODE_REACH:	
	// Moves the robot to a target in Cartesian space using a joint position controller 
	// (if we are in simulation mode), or a cartesian position controller 
	// (if the program runs on the real robot)
	{
		if (bSyncCart == true){

			Vector    x_state;
			x_state.Resize(6, false);

			Vector3 tmppos;Matrix3 tmporient;
			if(mRobot->IsSimulationMode())
			{
				x_state.InsertSubVector(0, mRobot->GetReferenceFrame(nEndEffectorId).GetHMatrix().GetTranslation(), 0, 3);
				x_state.InsertSubVector(3, mRobot->GetReferenceFrame(nEndEffectorId).GetHMatrix().GetOrientation().GetExactRotationAxis(), 0, 3);
			}
			else
			{
/*				Vector cartStiffness;
				cartStiffness.Resize(6);
				cartStiffness.SetSubVector(0, Vector3(1200, 1200, 1200));
				cartStiffness.SetSubVector(3, Vector3(700, 700, 700));
				mLWRRobot->SetCartStiffness(cartStiffness);

*/				((LWRRobot*)mRobot)->GetMeasuredCartPose(tmppos, tmporient);
				x_state(0) = tmppos(0);
				x_state(1) = tmppos(1);
				x_state(2) = tmppos(2);
				x_state.SetSubVector(3, tmporient.GetExactRotationAxis());

				mSensorsGroup.ReadSensors();
				vCurrJoint = mSensorsGroup.GetJointAngles();
				mActuatorsGroup.SetJointAngles(vCurrJoint);
				mActuatorsGroup.WriteActuators();
			}
			genCart->SetState(x_state);
			bSyncCart = false;
		}

		Matrix4 mHABSOBJinABS = oTargetObj->GetReferenceFrame().GetHMatrix();

//		mHABSOBJinABS.Mult(mTargetRelativeFrame, mHTARGETinABS);
		mHTARGETinABS.Identity();
		mHTARGETinABS.SetTranslation(mHABSOBJinABS.GetOrientation()*mTargetRelativeFrame.GetTranslation()
				+ mHABSOBJinABS.GetTranslation());

		mHTARGETinABS.SetOrientation(mHABSOBJinABS.GetOrientation()*mTargetRelativeFrame.GetOrientation());

		vTarget.SetSubVector(0, mHTARGETinABS.GetTranslation());
		vTarget.SetSubVector(3, mHTARGETinABS.GetOrientation().GetExactRotationAxis());
		genCart->SetTarget(vTarget);
		genCart->Update();
		genCart->GetState(vTarget);

		mRobotEEAbsolutePos.Set(Vector3(vTarget[0], vTarget[1], vTarget[2]));
		mRobotEEAbsoluteOrient = Matrix3::SRotationV(Vector3(vTarget[3], vTarget[4], vTarget[5]));

		// Test to see if it works with the cartesian impedance
		//mLWRRobot->SetCartCommand(mRobotEEAbsolutePos, mRobotEEAbsoluteOrient);

		// Doing it the old way in joint angles

		Vector	mCartesianTarget;
		mCartesianTarget.Resize(6);
		mCartesianTarget.Zero();

		if(mRobot->IsSimulationMode())
			vPositionError3 = mHTARGETinABS.GetTranslation() - mRobot->GetReferenceFrame(nEndEffectorId).GetOrigin();
		else
		{
			Matrix3 tmporient;Vector3 tmppos;
			mLWRRobot->GetMeasuredCartPose(tmppos, tmporient);
			vPositionError3 = mHTARGETinABS.GetTranslation() - tmppos;
		}
		mCartesianTarget(0) = vPositionError3(0) * vPosErrMult(0);
		mCartesianTarget(1) = vPositionError3(1) * vPosErrMult(1);
		mCartesianTarget(2) = vPositionError3(2) * vPosErrMult(2);

		if(mRobot->IsSimulationMode())
		{
		vOrientationError3 = ((Vector3) (mRobot->GetReferenceFrame(nEndEffectorId).GetOrient().GetColumn(2))).
				Cross(mHTARGETinABS.GetOrientation().GetColumn(2))
						   + ((Vector3) (mRobot->GetReferenceFrame(nEndEffectorId).GetOrient().GetColumn(1))).
						   Cross(mHTARGETinABS.GetOrientation().GetColumn(1));
		}
		else
		{
			Matrix3 tmporient;Vector3 tmppos;
			mLWRRobot->GetMeasuredCartPose(tmppos, tmporient);
			vOrientationError3 = ((Vector3)(tmporient.GetColumn(2))).
					Cross(mHTARGETinABS.GetOrientation().GetColumn(2)) +
					((Vector3) (tmporient.GetColumn(1))).Cross(mHTARGETinABS.GetOrientation().GetColumn(1));
		}
		mCartesianTarget(3) = vOrientationError3(0) * vOrientErrMult(0);
		mCartesianTarget(4) = vOrientationError3(1) * vOrientErrMult(1);
		mCartesianTarget(5) = vOrientationError3(2) * vOrientErrMult(2);
		
		double err=0;
		err = (vPositionError3. Norm() + vOrientationError3.Norm())/2.0;


		if (err > reachingThreshold) { // It means we are not there yet

		// ===============   DO IK ======================================

		mSensorsGroup.ReadSensors();
		vCurrJoint = mSensorsGroup.GetJointAngles();

		mKinChain.Update();
		mIKSolver.SetJacobian(mKinChain.GetJacobian(), 0);
		mIKSolver.SetTarget(SharedVector(mCartesianTarget), 0);
		mIKSolver.Solve();
		Vector ikout;
		ikout.Resize(mKinChain.GetDOFCount());
		ikout = mIKSolver.GetOutput();

		Vector  vJointTarget;
		vJointTarget.Resize(mRobot->GetDOFCount());
		for (int i = 0; i < ikout.Size(); i++)
			vJointTarget(mJointMapping[i]) = vCurrJoint(mJointMapping[i]) + ikout(i);

		Vector vJointCommand;
		vJointCommand.Resize(mRobot->GetDOFCount());

		mCDJointFilter->SetTarget(vJointTarget);
		mCDJointFilter->Update(fri_dt);
		mCDJointFilter->GetState(vJointCommand);

		mActuatorsGroup.SetJointAngles(vJointCommand);
		mActuatorsGroup.WriteActuators();
		}
		else{

			cout << "    >>> Target Reached with errors: " << endl;
			vPositionError3.Print();
			vOrientationError3.Print();
			cout<<"Error: "<<err <<endl;
			Matrix3 tmporient;Vector3 tmppos;
			if(mRobot->IsSimulationMode())
			{
				tmppos =  mRobot->GetReferenceFrame(nEndEffectorId).GetHMatrix().GetTranslation();
				tmporient =  mRobot->GetReferenceFrame(nEndEffectorId).GetHMatrix().GetOrientation();
			}
			else
			{
				mLWRRobot->GetMeasuredCartPose(tmppos, tmporient);
				tmppos.Print();

				mHABSOBJinABS.Print();
				mTargetRelativeFrame.Print();
				mHTARGETinABS.Print();


			}
			if(mRobot->IsSimulationMode())
			{
				currentTarget = mHABSOBJinABS.GetOrientation().Transpose().Mult(tmppos) - mHABSOBJinABS.GetTranslation();
				currentTarget.Print();
			}
		}
		
	}
	default:
		break;
	}

    return STATUS_OK;
}
int kukaBasicMovementModule::RespondToConsoleCommand(const string cmd, const vector<string> &args){
	if (cmd == "home") {
		mState = MODE_HOME;
		bSync = true;
	} else if (cmd == "go") {
		mState = MODE_REACH;
		setTargetToObject();
		cout << "Reaching the target" << endl;
		bSync = true;
	}
    return 0;
}

void kukaBasicMovementModule::initializeVariables(){

	// ===================================================
	// ========  Initialize CDDynamics         ===========
	// ===================================================
	
	// ! ! ! In your case you should initialize the function that will generate your trajectory
	
	genCart = new CDDynamics(6, module_dt, 1);  // Generates trajectory in cartesian space x, y, z, wx, wy, wz
	Vector vel_lim_cart(6);
	vel_lim_cart = DEG2RAD(60);
	genCart->SetVelocityLimits(vel_lim_cart);

	genJoint = new CDDynamics(mRobot->GetDOFCount(), module_dt, 1); // Generates trajectory in joint space
	Vector vel_lim_joint(mRobot->GetDOFCount());
	vel_lim_joint = DEG2RAD(60);
	genJoint->SetVelocityLimits(vel_lim_joint);

	mCDJointFilter = new CDDynamics(mRobot->GetDOFCount(), fri_dt, 1); // filter in jointspace
	Vector vel_lim_joints(mRobot->GetDOFCount());
	vel_lim_joints = DEG2RAD(60);
	mCDJointFilter->SetVelocityLimits(vel_lim_joints);

	//Reaching threshold
	reachingThreshold = 0.01;

	mTargetRelativeFrame = Matrix4::IDENTITY;

	mRobotEEAbsolutePos.Zero();
	mRobotEEAbsoluteOrient = Matrix3::IDENTITY;

	vRobotEEPosInTargetFrame.Zero();
	vRobotEEOrientInTargetFrame.Zero();
	mFullEEInTargetFrame = Matrix4::IDENTITY;

	vPositionError3.Zero();
	vOrientationError3.Zero();

	vTarget.Resize(6);
	vTarget.Zero();

	bSync = false;
	bSyncCart = false;

}

void kukaBasicMovementModule::setTargetToObject() {
	oTargetObj = GetWorld()->Find("wrist");
	if (!oTargetObj) GetConsole()->Print("ERROR: Target object not found");

	Matrix3         mTargetRelativeOrient;  // the target orientation relative to the object we're trying to reach
	Vector3         vTargetRelativePos;     // the target position relative to the object we're trying to reach
	
	// Setting a target relative to the object
	mTargetRelativeOrient.SetRow(Vector3(-1, 0, 0), 0);
	mTargetRelativeOrient.SetRow(Vector3(0, 1, 0), 1);
	mTargetRelativeOrient.SetRow(Vector3(0, 0, -1), 2);
	vTargetRelativePos.Set(-0.10, -0.12, 0.30);

	mTargetRelativeFrame.SetOrientation(mTargetRelativeOrient);
	mTargetRelativeFrame.SetTranslation((vTargetRelativePos));

	vPosErrMult.Set(1, 1, 0.5);
	vOrientErrMult.Set(2, 2, 2); // >> higher values, Orientation will converge first, position second

	genCart->SetWn(1);

	bSyncCart = true;
	bSync = true;
}

extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    kukaBasicMovementModule* create(){return new kukaBasicMovementModule();}
    void destroy(kukaBasicMovementModule* module){delete module;}
}

