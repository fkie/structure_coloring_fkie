/*
 * Copyright (c) 2013, Fraunhofer FKIE
 *
 * Authors: Bastian Gaspers
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of the Fraunhofer FKIE nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * This file is part of the StructureColoring ROS package.
 *
 * The StructureColoring ROS package is free software:
 * you can redistribute it and/or modify it under the terms of the
 * GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The StructureColoring ROS package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with The StructureColoring ROS package.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _StructureGL_h_
#define _StructureGL_h_

#include <QtOpenGL/QGLWidget>
#include <QtCore/QTimer>
#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>
#include <QtGui/QApplication>
#include <QtCore/QMutex>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <vector>
#include <time.h>
#include <ctime>
#include <opencv/highgui.h>
#include <structureColoring/structures/PlanePatch.h>
#include <structureColoring/structures/CylinderPatch.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>

class StructureGL: public QGLWidget {
Q_OBJECT
public:
	typedef Eigen::Vector3f Vec3;
	typedef PlanePatch::PlanePatches PlanePatches;
	typedef CylinderPatch::CylinderPatches CylinderPatches;
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef boost::shared_ptr<PointCloud> PointCloudPtr;
	typedef GLuint TexID;
	typedef std::vector<TexID> TexIDs;

	/**
	 * \brief If vec does not point to the viewport, it is flipped.
	 * \param vec Input and output vector to be flipped.
	 * \param viewport Input viewport position.
	 */
	static void flipToViewport(Vec3& vec, const Vec3& viewport);

	StructureGL(QWidget* parent = 0, const QGLWidget* shareWidget = 0, Qt::WindowFlags f = 0);
	StructureGL(QGLContext* context, QWidget* parent = 0, const QGLWidget* shareWidget = 0, Qt::WindowFlags f = 0);
	StructureGL(const QGLFormat& format, QWidget* parent = 0, const QGLWidget* shareWidget = 0, Qt::WindowFlags f = 0);
//	~StructureGL();

	void initializeGL();
	void initLight();
	void resizeGL(int width, int height);

	void keyPressEvent(QKeyEvent*);
	void keyReleaseEvent(QKeyEvent*);
	void wheelEvent(QWheelEvent*);
	void mouseMoveEvent(QMouseEvent*);
	void mousePressEvent(QMouseEvent*);
	void mouseReleaseEvent(QMouseEvent*);
	void setPerspective(double fovy, double aspekt, double zNear, double zFar);

	void paintGL();
	void paintCoordinateSystemGL();
	void paintPointsGL();
	void paintUnsegPointsGL();
	void paintPlanes();
	void paintBMPlanes();
	void paintHeightMapPlanes();
	void paintCylinders();

	void setPointCloud(PointCloudPtr pc){
		if (mPointCloudMutex.tryLock(100)){
			mPointCloud = pc;
			mPointCloudMutex.unlock();
		}
	}

	void setUnsegmentedPointCloud(PointCloudPtr pc){
		if (mPointCloudMutex.tryLock(100)){
			mUnsegPointCloud = pc;
			mPointCloudMutex.unlock();
		}
	}

	void setPlanePatches(const PlanePatches& pp){
		if (mPlanePatchesMutex.tryLock(100)){
			mPlanePatches = pp;
			resetAllPlaneTextures();
			mPlaneTextureIDs.reserve(pp.size());
			mPlaneAlphaIDs.reserve(pp.size());
			mHeightMapIDs.reserve(pp.size());
			mNormalMapIDs.reserve(pp.size());
			mPlanePatchesMutex.unlock();
			update();
		}
	}

	void setCylinderPatches(const CylinderPatches& cp){
		if (mCylinderPatchesMutex.tryLock(100)){
			mCylinderPatches = cp;
			//TODO Do texture things.
			mCylinderPatchesMutex.unlock();
			update();
		}
	}

private Q_SLOTS:
	void moveTimeout();

private:
	typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3> > Points;

	class Viewpoint {
	public:
		Viewpoint() :
			mX(0.f), mY(0.f), mZ(0.f), mYaw(0.f), mPitch(0.f), mVelForward(0.f), mVelUp(0.f), mVelSide(0.f), mVelTurn(0.f) {
		}
		void setXYZ(float x, float y, float z) {
			mX = x;
			mY = y;
			mZ = z;
		}
		void setYaw(float yaw) {
			mYaw = yaw;
		}
		void setPitch(float pitch) {
			mPitch = pitch;
		}
		void translate(float forward, float up, float sideways);
		void rotate(float yaw, float pitch);
		void setVelocity(float forwardVel, float upVel, float sidewaysVel, float turnVel) {
			mVelForward = forwardVel;
			mVelSide = sidewaysVel;
			mVelTurn = turnVel;
			mVelUp = upVel;
		}
		void setForwardVelocity(float forwardVel) {
			mVelForward = forwardVel;
		}
		void setSidewaysVelocity(float sidewaysVel) {
			mVelSide = sidewaysVel;
		}
		void setTurnVelocity(float turnVel) {
			mVelTurn = turnVel;
		}
		void setUpVelocity(float upVel) {
			mVelUp = upVel;
		}
		void applyGL();
		void applyVelocity(float dt = 1.f);
		float getYaw() {
			return mYaw;
		}
		float getPitch() {
			return mPitch;
		}
		float getX() {
			return mX;
		}
		float getY() {
			return mY;
		}
		float getZ() {
			return mZ;
		}
	private:
		float mX, mY, mZ, mYaw, mPitch;
		float mVelForward, mVelUp, mVelSide, mVelTurn;
	};

	static void packTo01(Vec3& vec){
		vec += Vec3(1.f, 1.f, 1.f);
		vec *= 0.5f;
	}

	void initialize();
	void getColorByIndex(float *rgb, unsigned int index, unsigned int total);
	void generateNormalisationCubeMap();
	void resetAllPlaneTextures();
	void resetTextures(TexIDs& textureIDs);
	void removedMarkedTextures();

	double mProjM[16], mModelM[16];
	int mViewP[4];
	GLfloat mLightPos0[4];

	bool mMouseView;
	float mTranslationSpeed, mRotationSpeed;
	float mYawStart, mPitchStart;
	QPoint mStartPos;
	Viewpoint mViewpoint;

	std::clock_t mLastTick;
	QTimer mMoveTimer;

	bool mForward, mReverse, mLeft, mRight, mUp, mDown, mTurnLeft, mTurnRight;


//	GLUquadricObj *mQuadric;

	PointCloudPtr mPointCloud;
	PointCloudPtr mUnsegPointCloud;
	QMutex mPointCloudMutex;

	PlanePatches mPlanePatches;
	QMutex mPlanePatchesMutex;

	CylinderPatches mCylinderPatches;
	QMutex mCylinderPatchesMutex;

	QMutex mPlaneTextureMutex;
	TexIDs mPlaneTextureIDs;
	TexIDs mPlaneAlphaIDs;
	TexIDs mHeightMapIDs;
	TexIDs mNormalMapIDs;
	TexIDs mTexIDsToRemove;
	TexID mNormalisationCubeMap;

	bool mShowPoints;
	bool mShowPlanes;
	bool mShowCylinders;
	bool mLight;
	bool mBumpMaps;
	bool mHeightMaps;
	bool mShowUnsegmentedPoints;
};


#endif /*_StructureGL_h_*/
