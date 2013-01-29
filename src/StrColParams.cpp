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

#include <structureColoring/StrColParams.h>
#include <math.h>
#include <stdio.h>

StrColParams::StrColParams() {
	init();
}

StrColParams::~StrColParams() {
	// TODO Auto-generated destructor stub
}

void StrColParams::init(){
	//for detailed parameter description look at the member variable declaration
	mMaxSACDistanceThreshold = 0.05f;//0.05f;
	mMinSACDistanceThreshold = 0.02f;//0.02f;
	mSACOctreeFactor = 0.1f;
	mSACOutlierRatioThreshold = 0.6f;
//	mGridCellSize = 0.02f;//0.02 for kinect
	mAngleEps = M_PI / 10.f;//depends on mPhi_resolution
	mAngleEpsOnMinOctreeRes = mAngleEps;
	mMinPointsInCC = 200;
	mMinNodesInCC = 3;
	mTexelSizeFactor = 2.f;
	mDilateIterations = 0;
	mPhi_resolution = 25;
	mRho_max = 20.0f;
	mMaxHTDistanceThreshold = 0.2f; // 0.2
	mHTOctreeBinSizeFactor = 2.f; // 2.f
	mHTDistanceDeviationFactor = 1.f;
	mMinOctreeResolution = 0.005f;//0.005 for ABW//0.01 for kinect
	mSqDistFactor = 0.002f;
	mMinPointsForNormal = 15;
	mCurvThreshold = 1.f / 10.f;//1.f / 12.f;
	mCurv2Threshold = 0.2f;
	mPrincipalVarFactor = 0.f;//0.5f;
	mMinOctreeNodes = mMinNodesInCC;
	mMinNodesInCylinder = 20;
	mTriesOnEachDepth = 25;
	mPostProcessing = 3;//7?
	mMergePlanesSimilarityFactor = 1.f;
	mNodeToBBDistance = 0.2f;
	mDebugSteps = -1;
	mConnectionNeighbors = 2;
	mNodeSegmentedRatio = 1.f; // should be smaller but near 1.f
	mNormalDistanceWeight = 0.1f;
	mMinRadiusFactor = 0.5f;
	mMaxRadiusFactor = 10.f;
	mMinCylinderRadius = 0.025f;
	mMaxCylinderRadius = 1.0f;
	mCylinderBins = 30;
	mCylinderPairNeighbors = 2;
	mMidPointBinSizeFactor = 0.25f;
	mRadiusDevFactor = 1.5f;
	mOccupiedRatio = 0.2f;
	mCylinderHeightDev = 0.2f;

	mPclSACmaxIter = 10000;
	mOnlyDepth = 0;
	mVerbose = false;
	mPGM = false;
	mKinect = false;
	mLaser = false;
	mABW = false;
	mPerceptron = false;
	mLoadPCD = false;
	mPCDnoRGB = false;
	mTextures = false;
	mCylinder = false;
	mPclSAC = false;
	mNoRansacStep = false;
	mWriteRawPic = false;
	mRawPicFilename = "rawPic";
	mLaserTopic = "";
	mRawPicCounter = 0;
	mRuntimeFilename = "elapsed.time";
}

void StrColParams::parseParamFile(std::string filename)
{
	FILE* file = fopen(filename.c_str(), "r");
	if (file == NULL) {
		printf("cannot open paramfile, using defaults\n");
		return;
	}
	if (!fscanf(file, "%f", &mMaxHTDistanceThreshold)) {
		printf("could not read param mMaxHTDistanceThreshold\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%f", &mHTOctreeBinSizeFactor)) {
		printf("could not read param mHTOctreeBinSizeFactor\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%f", &mHTDistanceDeviationFactor)) {
		printf("could not read param mHTDistanceDeviationFactor\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%f", &mMaxSACDistanceThreshold)) {
		printf("could not read param mMaxSACDistanceThreshold\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%f", &mMinSACDistanceThreshold)) {
		printf("could not read param mMinSACDistanceThreshold\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%f", &mSACOctreeFactor)) {
		printf("could not read param mSACOctreeFactor\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%f", &mSACOutlierRatioThreshold)) {
		printf("could not read param mSACOutlierRatioThreshold\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%f", &mMinOctreeResolution)) {
		printf("could not read param mMinOctreeResolution\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%f", &mSqDistFactor)) {
		printf("could not read param mSqDistFactor\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%d", &mPhi_resolution)) {
		printf("could not read param mPhi_resolution\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%d", &mConnectionNeighbors)) {
		printf("could not read param mConnectionNeighbors\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%f", &mAngleEps)) {
		printf("could not read param mAngleEps\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%f", &mRadiusDevFactor)) {
		printf("could not read param mRadiusDevFactor\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%d", &mTriesOnEachDepth)) {
		printf("could not read param mTriesOnEachDepth\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%f", &mOccupiedRatio)) {
		printf("could not read param mOccupiedRatio\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	if (!fscanf(file, "%f", &mNormalDistanceWeight)) {
		printf("could not read param mNormalDistanceWeight\n paramfile has wrong format, using defaults\n");
		fclose(file);
		return;
	}
	fclose(file);
}
