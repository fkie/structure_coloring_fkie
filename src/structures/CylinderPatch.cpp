/*
 * CylinderPatch.cpp
 *
 *  Created on: Dec 6, 2010
 *      Author: oehler
 */

#include <structureColoring/structures/CylinderPatch.h>
//#include <structureColoring/RotationHelper.h>

//void CylinderPatch::computeTransformation(){
//	Vec3 zAxis(0.f, 0.f, 1.f);
//	calculateRotation(mRotAxis, mRotAngle, mAxisDirection, zAxis);
//	mTransformRot = Eigen::AngleAxisf(mRotAngle, mRotAxis);
//	mTransformTrans= -mPointOnAxis;
//	mReTransformRot = Eigen::AngleAxisf(-mRotAngle, mRotAxis);
//}

/*****************************************************************************/

//void CylinderPatch::setTransformation(const float rotationAngle, const Vec3& rotationAxis){
//	mTransformRot = Eigen::AngleAxisf(rotationAngle, rotationAxis);
//	mReTransformRot = Eigen::AngleAxisf(-rotationAngle, rotationAxis);
//	mTransformTrans = -mPointOnAxis;
//}

/*****************************************************************************/

void CylinderPatch::computeOrientedBoundingBox(){
	//push BBVertices(bounding box vertices)
	mBBVertices.push_back(Vec3(mAxisMin, -mRadius, -mRadius));
	mBBVertices.push_back(Vec3(mAxisMin, mRadius, -mRadius));
	mBBVertices.push_back(Vec3(mAxisMax, -mRadius, -mRadius));
	mBBVertices.push_back(Vec3(mAxisMax, mRadius, -mRadius));
	mBBVertices.push_back(Vec3(mAxisMax, -mRadius, mRadius));
	mBBVertices.push_back(Vec3(mAxisMax, mRadius, mRadius));
	mBBVertices.push_back(Vec3(mAxisMin, -mRadius, mRadius));
	mBBVertices.push_back(Vec3(mAxisMin, mRadius, mRadius));

	//transform to robot coordinate system
	for(unsigned int i=0; i<mBBVertices.size();i++){
		Vec3 tmpPoint;
		transformToXYZCoords(tmpPoint, mBBVertices[i]);
		mBBVertices[i] = tmpPoint;
	}
}
