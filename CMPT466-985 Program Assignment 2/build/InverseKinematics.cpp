/***************************************************************************************
* Student Name:
* Student Number: 
****************************************************************************************/

#include "InverseKinematics.h"


float degree2rad = 3.141592658f / 180.0f;

//IK workflow
void InverseKinematics::IK() {

	//check if the endeffector and the target are close enough
	Vector3 endPos = Vector3(endEffector->GlobalPos.x, endEffector->GlobalPos.y, endEffector->GlobalPos.z);
	Vector3 tarPos = Vector3(target->x, target->y, target->z);

	int i = 0;
	while ((endPos - tarPos).length() > threshold && i < iterNum)
	{
		if (mode == 0) 
		{
			CCDMode();
		}
		else if (mode == 1)
		{
			JacobianMode();
		}
		endPos = Vector3(endEffector->GlobalPos.x, endEffector->GlobalPos.y, endEffector->GlobalPos.z);
		i++;
	}
}

//CCD Mode IK for right arm
void InverseKinematics::CCDMode()
{
	//add your code here
	//hint: Do forward kinematics when the endeffector's global position need to be updated
	//with newly computed quaternions. 
	
	//I.e., use forwardKinematcis->forwardKinematicsComputation(base) 
	//whenever you need to update endeffector's global position.

}

//Entire Right Arm Jacobian IK
void InverseKinematics::JacobianMode()
{
	//add your code here
}



