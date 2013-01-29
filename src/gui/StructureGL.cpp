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

#include <structureColoring/gui/StructureGL.h>
#include <GL/glu.h>

/*****************************************************************************/

void StructureGL::flipToViewport(Vec3& vec, const Vec3& viewport) {
	if (viewport.dot(vec) > 0) {
		vec *= -1.f;
	}
}

/*****************************************************************************/

StructureGL::StructureGL(QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f) :
	QGLWidget(parent, shareWidget, f), mForward(false), mReverse(false), mLeft(false), mRight(false), mUp(false), mDown(
			false), mTurnLeft(false), mTurnRight(false) {
	initialize();
}

/*****************************************************************************/

StructureGL::StructureGL(QGLContext* context, QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f) :
	QGLWidget(context, parent, shareWidget, f), mForward(false), mReverse(false), mLeft(false), mRight(false), mUp(false),
			mDown(false), mTurnLeft(false), mTurnRight(false) {
	initialize();
}

/*****************************************************************************/

StructureGL::StructureGL(const QGLFormat& format, QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f) :
	QGLWidget(format, parent, shareWidget, f), mForward(false), mReverse(false), mLeft(false), mRight(false), mUp(false),
			mDown(false), mTurnLeft(false), mTurnRight(false) {
	initialize();
}

/*****************************************************************************/

//StructureGL::~StructureGL() {
//	gluDeleteQuadric(mQuadric);
//}

/*****************************************************************************/

void StructureGL::initialize() {
	connect(&mMoveTimer, SIGNAL(timeout()), this, SLOT(moveTimeout()));
	mMoveTimer.setSingleShot(false);
	mTranslationSpeed = 1.f;
	mRotationSpeed = M_PI_2;
	mMouseView = false;
	mShowPoints = false;
	mShowPlanes = true;
	mShowCylinders = true;
	mPointCloud = PointCloudPtr(new PointCloud());
	mUnsegPointCloud = PointCloudPtr(new PointCloud());
	mLight = true;
	mBumpMaps = false;
	mHeightMaps = false;
	mShowUnsegmentedPoints = true;
}

/*****************************************************************************/

void StructureGL::initializeGL() {
	glClearColor(1.f, 1.f, 1.f, 0.f);
	glClearDepth(1.f);
	glDisable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glShadeModel(GL_SMOOTH);
	glPointSize(2.f);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	glDisable(GL_BLEND);
	glEnable(GL_TEXTURE_2D);
	//	mQuadric = gluNewQuadric();
	//	gluQuadricNormals(mQuadric, GLU_SMOOTH);
	//	gluQuadricTexture(mQuadric, GL_TRUE);
	//	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	//	glEnable(GL_COLOR_MATERIAL);
    glGenTextures(1, &mNormalisationCubeMap);
    glBindTexture(GL_TEXTURE_CUBE_MAP, mNormalisationCubeMap);
    generateNormalisationCubeMap();
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	initLight();
}

/*****************************************************************************/

void StructureGL::initLight() {
	GLfloat ambient_matrl[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat diffuse_matrl[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_matrl);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse_matrl);
	GLfloat lightAmbient[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat lightDiffuse[] = { 1.f, 1.f, 1.f, 1.f };
	mLightPos0[0] = 0.f;
	mLightPos0[1] = -2.0f;
	mLightPos0[2] = 0.f;
	mLightPos0[3] = 1.f;
//	GLfloat linearAttenuation[] = { 0.3f };
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, mLightPos0);
//	glLightfv(GL_LIGHT0, GL_LINEAR_ATTENUATION, linearAttenuation);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHT0);
	GLfloat ambient_lightModel[] = { 0.25f, 0.25f, 0.25f, 1.0f };
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient_lightModel);

}

/*****************************************************************************/

void StructureGL::resizeGL(int width, int height) {
	glViewport(0, 0, width, height);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	setPerspective(45., 1. * width / height, 0.01, 327.68);
	glGetDoublev(GL_PROJECTION_MATRIX, mProjM);
	glGetDoublev(GL_MODELVIEW_MATRIX, mModelM);
	glGetIntegerv(GL_VIEWPORT, mViewP);
	update();
}

/*****************************************************************************/

void StructureGL::paintCoordinateSystemGL() {
	glBegin(GL_LINES);
	glColor3f(1.0, 0, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(-0.1, 0, 0);
	glColor3f(0, 1.0, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0.1, 0);
	glColor3f(0, 0, 1.0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, -0.1);
	glEnd();
}

/*****************************************************************************/

void StructureGL::paintPlanes() {
	if (mPlanePatchesMutex.tryLock(10)) {
		if (mPlanePatches.size() > 0) {
			unsigned int pi = 0;
			glEnable(GL_ALPHA_TEST);
			glAlphaFunc(GL_GREATER, 0.5);
			for (PlanePatches::const_iterator pit = mPlanePatches.begin(); pit != mPlanePatches.end(); ++pit) {
				glColor4f(1.f, 1.f, 1.f, 1.f);
				const PlanePatch& pp = **pit;
				if (pi >= mPlaneTextureIDs.size()) {
					TexID texID = 0;
					glGenTextures(1, &texID);
					mPlaneTextureIDs.push_back(texID);
					glBindTexture(GL_TEXTURE_2D, texID);
					const cv::Mat& texMap = pp.getTextureMap();
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);

					gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA, texMap.cols, texMap.rows, GL_RGBA, GL_FLOAT,
							texMap.data);
				}
				Points vertices = pp.getBRVertices();
				Vec3 ppNormal = pp.getPlane3D().getPlaneNormal();
				Vec3 vp(-mViewpoint.getZ(), -mViewpoint.getX(), mViewpoint.getY());
				flipToViewport(ppNormal, vp);
				glActiveTexture(GL_TEXTURE0);
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, mPlaneTextureIDs[pi]);
				glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
				glBegin(GL_QUADS);
				glNormal3f(-ppNormal(1), ppNormal(2), -ppNormal(0));
				glTexCoord2f(0, 0);
				glVertex3f(-vertices[0](1), vertices[0](2), -vertices[0](0));
				glTexCoord2f(0, pp.getTextureHeightRatio());
				glVertex3f(-vertices[1](1), vertices[1](2), -vertices[1](0));
				glTexCoord2f(pp.getTextureWidthRatio(), pp.getTextureHeightRatio());
				glVertex3f(-vertices[2](1), vertices[2](2), -vertices[2](0));
				glTexCoord2f(pp.getTextureWidthRatio(), 0);
				glVertex3f(-vertices[3](1), vertices[3](2), -vertices[3](0));
				glEnd();
				++pi;
			}
			glDisable(GL_ALPHA_TEST);
			glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			glDisable(GL_TEXTURE_2D);
		}
		mPlanePatchesMutex.unlock();
	}
}

/*****************************************************************************/

void StructureGL::paintBMPlanes() {
	const GLfloat& light_x = mLightPos0[0];
	const GLfloat& light_y = mLightPos0[1];
	const GLfloat& light_z = mLightPos0[2];
	if (mPlanePatchesMutex.tryLock(10)) {
		if (mPlanePatches.size() > 0) {
			unsigned int pi = 0;
			glEnable(GL_ALPHA_TEST);
			glAlphaFunc(GL_GREATER, 0.5);
			for (PlanePatches::const_iterator pit = mPlanePatches.begin(); pit != mPlanePatches.end(); ++pit) {
				const PlanePatch& pp = **pit;
				glColor4f(1.f, 1.f, 1.f, 1.f);
				if (pi >= mPlaneTextureIDs.size()) {
					TexID texID = 0;
					glGenTextures(1, &texID);
					mPlaneTextureIDs.push_back(texID);
					glBindTexture(GL_TEXTURE_2D, texID);
					const cv::Mat& texMap = pp.getTextureMap();
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
					gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA, texMap.cols, texMap.rows, GL_RGBA, GL_FLOAT, texMap.data);
				}
				if (pi >= mNormalMapIDs.size()) {
					TexID texID = 0;
					glGenTextures(1, &texID);
					mNormalMapIDs.push_back(texID);
					glBindTexture(GL_TEXTURE_2D, texID);
					const cv::Mat& texMap = pp.getNormalMap();
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
					gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, texMap.cols, texMap.rows, GL_RGB, GL_FLOAT, texMap.data);
				}
				Points vertices = pp.getBRVertices();
				Vec3 ppNormal = pp.getPlane3D().getPlaneNormal();
				Vec3 vp(-mViewpoint.getZ(), -mViewpoint.getX(), mViewpoint.getY());
				flipToViewport(ppNormal, vp);

				// Set The First Texture Unit To Normalize Our Vector From The Surface To The Light.
				// Set The Texture Environment Of The First Texture Unit To Replace It With The
				// Sampled Value Of The Normalization Cube Map.
				glActiveTexture(GL_TEXTURE0);
				glEnable(GL_TEXTURE_CUBE_MAP);
				glBindTexture(GL_TEXTURE_CUBE_MAP, mNormalisationCubeMap);
				glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE);
				glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_REPLACE) ;
				glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_RGB, GL_TEXTURE) ;

				// Set The Second Unit To The Bump Map.
				// Set The Texture Environment Of The Second Texture Unit To Perform A Dot3
				// Operation With The Value Of The Previous Texture Unit (The Normalized
				// Vector Form The Surface To The Light) And The Sampled Texture Value (The
				// Normalized Normal Vector Of Our Bump Map).
				glActiveTexture(GL_TEXTURE1);
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, mNormalMapIDs[pi]);
				glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_COMBINE);
				glTexEnvi(GL_TEXTURE_ENV, GL_COMBINE_RGB, GL_DOT3_RGB);
				glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE0_RGB, GL_PREVIOUS);
				glTexEnvi(GL_TEXTURE_ENV, GL_SOURCE1_RGB, GL_TEXTURE);

				// Set The Third Texture Unit To Our Texture.
				// Set The Texture Environment Of The Third Texture Unit To Modulate
				// (Multiply) The Result Of Our Dot3 Operation With The Texture Value.
				glActiveTexture(GL_TEXTURE2);
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, mPlaneTextureIDs[pi]);
				glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

				GLfloat vertex_to_light_x, vertex_to_light_y, vertex_to_light_z;
				glBegin(GL_QUADS);
				for(unsigned int i = 0; i < 4; i++){
					const float& current_vertex_x = -vertices[i](1);
					const float& current_vertex_y = vertices[i](2);
					const float& current_vertex_z = -vertices[i](0);
					float current_texcoord_s = 0;
					if (i > 1) current_texcoord_s += pp.getTextureWidthRatio();
					float current_texcoord_t = 0;
					if (i == 1 || i == 2) current_texcoord_t += pp.getTextureHeightRatio();
					vertex_to_light_x = current_vertex_x - light_x;
					vertex_to_light_y = current_vertex_y - light_y;
					vertex_to_light_z = current_vertex_z - light_z;

					// Passing The vector_to_light Values To Texture Unit 0.
					// Remember The First Texture Unit Is Our Normalization Cube Map
					// So This Vector Will Be Normalized For Dot 3 Bump Mapping.
					glMultiTexCoord3f(GL_TEXTURE0, vertex_to_light_x, vertex_to_light_y, vertex_to_light_z);
					// Passing The Simple Texture Coordinates To Texture Unit 1 And 2.
					glMultiTexCoord2f(GL_TEXTURE1, current_texcoord_s, current_texcoord_t);
					glMultiTexCoord2f(GL_TEXTURE2, current_texcoord_s, current_texcoord_t);
					glVertex3f(current_vertex_x, current_vertex_y, current_vertex_z);
				}
				glEnd();
				++pi;
			}
		}
		glDisable( GL_ALPHA_TEST);
		glActiveTexture( GL_TEXTURE2 );
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glDisable( GL_TEXTURE_2D);
		glActiveTexture( GL_TEXTURE1 );
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glDisable(GL_TEXTURE_2D);
		glActiveTexture( GL_TEXTURE0 );
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glDisable(GL_TEXTURE_CUBE_MAP);
		mPlanePatchesMutex.unlock();
	}
}

/*****************************************************************************/

void StructureGL::paintHeightMapPlanes() {
	if (mPlanePatchesMutex.tryLock(10)) {
		if (mPlanePatches.size() > 0) {
			unsigned int pi = 0;
			glEnable(GL_ALPHA_TEST);
			glAlphaFunc(GL_GREATER, 0.5);
			for (PlanePatches::const_iterator pit = mPlanePatches.begin(); pit != mPlanePatches.end(); ++pit) {
				const PlanePatch& pp = **pit;
				glColor4f(1.f, 1.f, 1.f, 1.f);
				if (pi >= mPlaneAlphaIDs.size()) {
					TexID texID = 0;
					glGenTextures(1, &texID);
					mPlaneAlphaIDs.push_back(texID);
					glBindTexture(GL_TEXTURE_2D, texID);
					const cv::Mat& texMap = pp.getTextureMap();
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
					gluBuild2DMipmaps(GL_TEXTURE_2D, GL_ALPHA, texMap.cols, texMap.rows, GL_RGBA, GL_FLOAT, texMap.data);
				}
				if (pi >= mHeightMapIDs.size()) {
					TexID texID = 0;
					glGenTextures(1, &texID);
					mHeightMapIDs.push_back(texID);
					glBindTexture(GL_TEXTURE_2D, texID);
					const cv::Mat& texMap = pp.getHeightMap();
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
					gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA, texMap.cols, texMap.rows, GL_LUMINANCE, GL_FLOAT, texMap.data);
				}
				Points vertices = pp.getBRVertices();
				Vec3 ppNormal = pp.getPlane3D().getPlaneNormal();
				Vec3 vp(-mViewpoint.getZ(), -mViewpoint.getX(), mViewpoint.getY());
				flipToViewport(ppNormal, vp);

				glActiveTexture(GL_TEXTURE0);
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, mHeightMapIDs[pi]);
				glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

				glActiveTexture(GL_TEXTURE1);
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, mPlaneAlphaIDs[pi]);
				glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

				glBegin(GL_QUADS);
				glNormal3f(-ppNormal(1), ppNormal(2), -ppNormal(0));
				glMultiTexCoord2f(GL_TEXTURE0, 0, 0);
				glMultiTexCoord2f(GL_TEXTURE1, 0, 0);
				glVertex3f(-vertices[0](1), vertices[0](2), -vertices[0](0));
				glMultiTexCoord2f(GL_TEXTURE0, 0, pp.getTextureHeightRatio());
				glMultiTexCoord2f(GL_TEXTURE1, 0, pp.getTextureHeightRatio());
				glVertex3f(-vertices[1](1), vertices[1](2), -vertices[1](0));
				glMultiTexCoord2f(GL_TEXTURE0, pp.getTextureWidthRatio(), pp.getTextureHeightRatio());
				glMultiTexCoord2f(GL_TEXTURE1, pp.getTextureWidthRatio(), pp.getTextureHeightRatio());
				glVertex3f(-vertices[2](1), vertices[2](2), -vertices[2](0));
				glMultiTexCoord2f(GL_TEXTURE0, pp.getTextureWidthRatio(), 0);
				glMultiTexCoord2f(GL_TEXTURE1, pp.getTextureWidthRatio(), 0);
				glVertex3f(-vertices[3](1), vertices[3](2), -vertices[3](0));
				glEnd();
				++pi;
			}
		}
		glDisable(GL_ALPHA_TEST);
		glActiveTexture(GL_TEXTURE1);
		glDisable(GL_TEXTURE_2D);
		glActiveTexture(GL_TEXTURE0);
		glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glDisable(GL_TEXTURE_2D);
		mPlanePatchesMutex.unlock();
	}
}

/*****************************************************************************/

void StructureGL::paintCylinders(){
	if(mCylinderPatchesMutex.tryLock(10)){
		float rgb[3] = {0.f, 0.f, 1.f};
		unsigned int ci=0;
		for(CylinderPatches::const_iterator cit = mCylinderPatches.begin(); cit != mCylinderPatches.end(); ++cit){
			getColorByIndex(rgb, ci, mCylinderPatches.size());
			const CylinderPatch& cp = **cit;
			glColor3f(rgb[1],rgb[0],rgb[2]);
			glDisable(GL_TEXTURE_2D);
//			if(mShowPoints){
//				std::vector<Vec3> points = cp.getPoints();
//				glBegin(GL_POINTS);
//				for(unsigned int i=0; i<points.size(); i++){
//					Vec3 p = points[i];
//					glVertex3f(-p(1), p(2), -p(0));
//				}
//				glEnd();
//			}
//			glColor3f(rgb[1],rgb[2],rgb[0]);
//			if(mBB){
				const Points& vertices3 = cp.getBBVertices();
				glBegin(GL_QUAD_STRIP);
				glVertex3f(-vertices3[0](1),vertices3[0](2),-vertices3[0](0));
				glVertex3f(-vertices3[1](1),vertices3[1](2),-vertices3[1](0));
				glVertex3f(-vertices3[2](1),vertices3[2](2),-vertices3[2](0));
				glVertex3f(-vertices3[3](1),vertices3[3](2),-vertices3[3](0));
				glVertex3f(-vertices3[4](1),vertices3[4](2),-vertices3[4](0));
				glVertex3f(-vertices3[5](1),vertices3[5](2),-vertices3[5](0));
				glVertex3f(-vertices3[6](1),vertices3[6](2),-vertices3[6](0));
				glVertex3f(-vertices3[7](1),vertices3[7](2),-vertices3[7](0));
				glEnd();
				glBegin(GL_QUAD_STRIP);
				glVertex3f(-vertices3[4](1),vertices3[4](2),-vertices3[4](0));
				glVertex3f(-vertices3[2](1),vertices3[2](2),-vertices3[2](0));
				glVertex3f(-vertices3[6](1),vertices3[6](2),-vertices3[6](0));
				glVertex3f(-vertices3[0](1),vertices3[0](2),-vertices3[0](0));
				glVertex3f(-vertices3[7](1),vertices3[7](2),-vertices3[7](0));
				glVertex3f(-vertices3[1](1),vertices3[1](2),-vertices3[1](0));
				glVertex3f(-vertices3[5](1),vertices3[5](2),-vertices3[5](0));
				glVertex3f(-vertices3[3](1),vertices3[3](2),-vertices3[3](0));
				glEnd();
//			} else if(mShowAlphaMap){
//				glColor4f(rgb[1],rgb[2],rgb[0],1.f);
//				glEnable(GL_ALPHA_TEST);
//				glAlphaFunc(GL_GREATER, 0.5);
//				if (ci >= mCylinderTextureIDs.size()){
//					unsigned int texID = 0;
//					glEnable(GL_TEXTURE_2D);
//					glGenTextures(1, &texID);
//					mCylinderTextureIDs.push_back(texID);
//					glBindTexture(GL_TEXTURE_2D, texID);
//					cv::Mat alphaMap = cp.getTextureMap();
//					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//					glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//					glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
//					glTexImage2D(GL_TEXTURE_2D, //target
//						0, //Mipmap level
//						GL_ALPHA, //internal format
//						alphaMap.cols, //width
//						alphaMap.rows, //height
//						0, //border
//						GL_ALPHA, //format of pixeldata
//						GL_UNSIGNED_BYTE, //type of pixeldata
//						alphaMap.data //pixeldata
//					);
//				}
//				glEnable(GL_TEXTURE_2D);
//				glBindTexture(GL_TEXTURE_2D, mCylinderTextureIDs[ci]);
//				glPushMatrix();
//				Vec3 bp = cp.getTopPoint();
//				glTranslatef(-bp(1), bp(2), -bp(0));
//				Vec3 rotAxis = cp.getRotationAxis();
//				double rotAngle = (cp.getRotationAngle()/ M_PI) * 180;
//				glRotatef(rotAngle, -rotAxis(1), rotAxis(2), -rotAxis(0));
//				gluCylinder(mQuadric, cp.getRadius(), cp.getRadius(), cp.getHeight(),16,16);
//				glPopMatrix();
//				glDisable(GL_ALPHA_TEST);
//				glDisable(GL_TEXTURE_2D);
//			} else {
//				glPushMatrix();
//				Vec3 bp = cp.getTopPoint();
//				glTranslatef(-bp(1), bp(2), -bp(0));
//				Vec3 rotAxis = cp.getRotationAxis();
//				double rotAngle = (cp.getRotationAngle()/ M_PI) * 180;
//				glRotatef(rotAngle, -rotAxis(1), rotAxis(2), -rotAxis(0));
//				gluCylinder(mQuadric, cp.getRadius(), cp.getRadius(), cp.getHeight(),16,16);
//				glPopMatrix();
//			}
		}
		mCylinderPatchesMutex.unlock();
	}
}

/*****************************************************************************/

void StructureGL::paintPointsGL() {
	if (mPointCloudMutex.tryLock(50)) {
		if (mPointCloud != NULL) {
			glBegin(GL_POINTS);
			for (unsigned int i = 0; i < mPointCloud->points.size(); i++) {
				PointT p = mPointCloud->points[i];
				int rgb = *reinterpret_cast<int*> (&p.rgb);
				glColor3ub((rgb >> 16) & 0xff, (rgb >> 8) & 0xff, rgb & 0xff);
				glVertex3f(-p.y, p.z, -p.x);
			}
			glEnd();
		}
		mPointCloudMutex.unlock();
	}
}

/*****************************************************************************/

void StructureGL::paintUnsegPointsGL() {
	if (mPointCloudMutex.tryLock(50)) {
		if (mUnsegPointCloud != NULL) {
			glBegin(GL_POINTS);
			for (unsigned int i = 0; i < mUnsegPointCloud->points.size(); i++) {
				PointT p = mUnsegPointCloud->points[i];
				int rgb = *reinterpret_cast<int*> (&p.rgb);
				glColor3ub((rgb >> 16) & 0xff, (rgb >> 8) & 0xff, rgb & 0xff);
				glVertex3f(-p.y, p.z, -p.x);
			}
			glEnd();
		}
		mPointCloudMutex.unlock();
	}
}

/*****************************************************************************/

void StructureGL::paintGL() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	mViewpoint.applyGL();
	glGetDoublev(GL_MODELVIEW_MATRIX, mModelM);
	if (mLight)
		glEnable(GL_LIGHTING);
	else
		glDisable(GL_LIGHTING);

	glLightfv(GL_LIGHT0, GL_POSITION, mLightPos0);

	paintCoordinateSystemGL();
	if (mShowPoints)
		paintPointsGL();
	if (mShowUnsegmentedPoints)
		paintUnsegPointsGL();
	if (mShowPlanes) {
		if (mBumpMaps)
			paintBMPlanes();
		else if (mHeightMaps)
			paintHeightMapPlanes();
		else
			paintPlanes();
	}
	if (mShowCylinders) paintCylinders();

	GLenum errCode;
	const GLubyte *errString;
	if ((errCode = glGetError()) != GL_NO_ERROR) {
	    errString = gluErrorString(errCode);
	   std::cerr << "OpenGL Error (paintGL): " << errString << std::endl;
	   exit(1);
	}

	if (!mTexIDsToRemove.empty()){
		removedMarkedTextures();
	}
}

/*****************************************************************************/

void StructureGL::moveTimeout() {
	std::clock_t tick = clock();
	float dt = (float) (mLastTick - tick) / (float) CLOCKS_PER_SEC;
	float forward = 0.f, up = 0.f, side = 0.f, turn = 0.f;
	if (mForward)
		forward -= 1.f;
	if (mReverse)
		forward += 1.f;
	if (mLeft)
		side += 1.f;
	if (mRight)
		side -= 1.f;
	if (mUp)
		up -= 1.f;
	if (mDown)
		up += 1.f;
	if (mTurnLeft)
		turn -= 1.f;
	if (mTurnRight)
		turn += 1.f;
	forward *= mTranslationSpeed;
	up *= mTranslationSpeed;
	side *= mTranslationSpeed;
	turn *= mRotationSpeed;
	mViewpoint.setVelocity(forward, up, side, turn);
	mViewpoint.applyVelocity(dt);
	mLastTick = tick;
	updateGL();
}

/*****************************************************************************/

void StructureGL::setPerspective(double fovy, double aspect, double zNear, double zFar) {
	gluPerspective(fovy, aspect, zNear, zFar);
}

/*****************************************************************************/

void StructureGL::keyPressEvent(QKeyEvent* event) {
	if (!QApplication::keyboardModifiers()) {
		switch (event->key()) {
		case Qt::Key_Space:
			mUp = true;
			mDown = false;
			event->accept();
			break;
		case Qt::Key_A:
			mLeft = true;
			mRight = false;
			event->accept();
			break;
		case Qt::Key_B:
			mBumpMaps = !mBumpMaps;
			mHeightMaps = false;
			resetTextures(mPlaneTextureIDs);
			event->accept();
			update();
			break;
		case Qt::Key_C:
			mDown = true;
			mUp = false;
			event->accept();
			break;
		case Qt::Key_D:
			mRight = true;
			mLeft = false;
			event->accept();
			break;
		case Qt::Key_H:
			mHeightMaps = !mHeightMaps;
			mBumpMaps = false;
			resetTextures(mPlaneTextureIDs);
			event->accept();
			update();
			break;
		case Qt::Key_L:
			mLight = !mLight;
			event->accept();
			update();
			break;
		case Qt::Key_P:
			if (mShowPoints){
				mShowPoints = false;
				mShowPlanes = true;
			} else if (mShowPlanes){
				mShowPlanes = false;
				mShowPoints = true;
			} else {
				mShowPoints = true;
			}
			event->accept();
			update();
			break;
		case Qt::Key_S:
			mReverse = true;
			mForward = false;
			event->accept();
			break;
		case Qt::Key_U:
			mShowUnsegmentedPoints = !mShowUnsegmentedPoints;
			event->accept();
			break;
		case Qt::Key_W:
			mForward = true;
			mReverse = false;
			event->accept();
			break;
		default:
			event->ignore();
			break;
		}
	} else
		event->ignore();
	if ((mForward || mReverse || mLeft || mRight || mUp || mDown || mTurnLeft || mTurnRight) && !mMoveTimer.isActive()) {
		mLastTick = clock();
		mMoveTimer.start(0);
	}
}

/*****************************************************************************/

void StructureGL::keyReleaseEvent(QKeyEvent* event) {
	if (!event->isAutoRepeat()) {
		switch (event->key()) {
		case Qt::Key_W:
			mForward = false;
			mReverse = false;
			event->accept();
			break;
		case Qt::Key_S:
			mReverse = false;
			mForward = false;
			event->accept();
			break;
		case Qt::Key_A:
			mLeft = false;
			mRight = false;
			event->accept();
			break;
		case Qt::Key_D:
			mRight = false;
			mLeft = false;
			event->accept();
			break;
		case Qt::Key_Space:
			mUp = false;
			mDown = false;
			event->accept();
			break;
		case Qt::Key_C:
			mDown = false;
			mUp = false;
			event->accept();
			break;
		default:
			event->ignore();
			break;
		}
	} else
		event->ignore();
	if (!(mForward || mReverse || mLeft || mRight || mUp || mDown || mTurnLeft || mTurnRight) && mMoveTimer.isActive()) {
		mMoveTimer.stop();
	}
}

/*****************************************************************************/

void StructureGL::mousePressEvent(QMouseEvent* event) {
	if (event->button() == Qt::RightButton) {
		mMouseView = true;
		mStartPos = event->pos();
		mYawStart = mViewpoint.getYaw();
		mPitchStart = mViewpoint.getPitch();
		event->accept();
	} else
		event->ignore();
}

/*****************************************************************************/

void StructureGL::mouseReleaseEvent(QMouseEvent* event) {
	if (event->button() == Qt::RightButton) {
		mMouseView = false;
		event->accept();
	} else
		event->ignore();
}

/*****************************************************************************/

void StructureGL::mouseMoveEvent(QMouseEvent* event) {
	if (mMouseView) {
		static const float LOOK_SENSITIVITY = -0.01f;
		static const float FREE_TURN_SENSITIVITY = -0.01f;

		float turn = FREE_TURN_SENSITIVITY * (event->pos().x() - mStartPos.x());
		float tilt = LOOK_SENSITIVITY * (event->pos().y() - mStartPos.y());
		mViewpoint.setYaw(mYawStart + turn);
		mViewpoint.setPitch(mPitchStart + tilt);
		updateGL();
		event->accept();
	}
	if (event->buttons() & Qt::LeftButton)
		event->ignore();
}

/*****************************************************************************/

void StructureGL::wheelEvent(QWheelEvent* event) {
	static const float WHEEL_SENSITIVITY = 0.01f;
	mViewpoint.translate(0.0f, event->delta() * WHEEL_SENSITIVITY * mTranslationSpeed / 15.0f, 0.0f);
	updateGL();
}

/*****************************************************************************/

void StructureGL::getColorByIndex(float *rgb, unsigned int index, unsigned int total) {
	float t = static_cast<float> (index) / static_cast<float> (total);
	// set rgb values
	if (t == 1.0) {
		rgb[0] = 1.0f;
		rgb[1] = 0.0f;
		rgb[2] = 0.0f;
	} else if (t < 1.0 && t >= 0.83333) {
		rgb[0] = 1.0f;
		rgb[1] = 1.0f - (t - 0.83333) / 0.16666;
		rgb[2] = 0.0f;
	} else if (t < 0.83333 && t >= 0.66666) {
		rgb[0] = (t - 0.66666) / 0.16666;
		rgb[1] = 1.0f;
		rgb[2] = 0.0f;
	} else if (t < 0.66666 && t >= 0.5) {
		rgb[0] = 0.0f;
		rgb[1] = 1.0f;
		rgb[2] = 1.0f - (t - 0.5) / 0.16666;
	} else if (t < 0.5 && t >= 0.33333) {
		rgb[0] = 0.0f;
		rgb[1] = 1.0f - (t - 0.33333) / 0.16666;
		rgb[2] = 1.0f;
	} else if (t < 0.33333 && t >= 0.16666) {
		rgb[0] = (t - 0.16666) / 0.16666;
		rgb[1] = 0.0f;
		rgb[2] = 1.0f;
	} else {
		rgb[0] = 1.0f;
		rgb[1] = 0.0f;
		rgb[2] = 1.0f - t / 0.16666;
	}
}

/*****************************************************************************/

void StructureGL::generateNormalisationCubeMap() {

	//First we create space to hold the data for a single face. Each face is 32x32, and we need to store the R, G and B components of the color at each point.

	unsigned char * data = new unsigned char[32 * 32 * 3];
	if (!data) {
//		printf("Unable to allocate memory for texture data for cube map\n");
		return;
	}
	//Declare some useful variables.
	int size = 32;
	float offset = 0.5f;
	float halfSize = 16.0f;
	Vec3 tempVector;
	unsigned char * bytePtr;
	//We will do this next part once for each face. I will show how it is done for the positive x face. The other faces are very similar. Look in the source for details.

	//positive x
	bytePtr = data;
	//Loop through the pixels in the face.
	for (int j = 0; j < size; j++) {
		for (int i = 0; i < size; i++) {
			//Calculate the vector from the center of the cube to this texel.
			tempVector.x() = halfSize;
			tempVector.y() = - (j + offset - halfSize);
			tempVector.z() = - (i + offset - halfSize);
			//We normalize this vector, pack it to [0, 1] so it can be stored in a color, and save this into the texture data.
			tempVector.normalize();
			packTo01(tempVector);
			bytePtr[0] = (unsigned char) (tempVector.x() * 255);
			bytePtr[1] = (unsigned char) (tempVector.y() * 255);
			bytePtr[2] = (unsigned char) (tempVector.z() * 255);
			bytePtr += 3;
		}
	}
	//Now we upload this face of the cube to OpenGL. We tell it that this is the positive x face of the current cube map, and where to find the data.
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, GL_RGBA8, size, size, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

	//positive y
	bytePtr = data;
	//Loop through the pixels in the face.
	for (int j = 0; j < size; j++) {
		for (int i = 0; i < size; i++) {
			//Calculate the vector from the center of the cube to this texel.
			tempVector.x() = (i + offset - halfSize);
			tempVector.y() = halfSize;
			tempVector.z() = (j + offset - halfSize);
			//We normalize this vector, pack it to [0, 1] so it can be stored in a color, and save this into the texture data.
			tempVector.normalize();
			packTo01(tempVector);
			bytePtr[0] = (unsigned char) (tempVector.x() * 255);
			bytePtr[1] = (unsigned char) (tempVector.y() * 255);
			bytePtr[2] = (unsigned char) (tempVector.z() * 255);
			bytePtr += 3;
		}
	}
	//Now we upload this face of the cube to OpenGL. We tell it that this is the positive x face of the current cube map, and where to find the data.
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, GL_RGBA8, size, size, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

	//positive z
	bytePtr = data;
	//Loop through the pixels in the face.
	for (int j = 0; j < size; j++) {
		for (int i = 0; i < size; i++) {
			//Calculate the vector from the center of the cube to this texel.
			tempVector.x() = (i + offset - halfSize);
			tempVector.y() = - (j + offset - halfSize);
			tempVector.z() = halfSize;
			//We normalize this vector, pack it to [0, 1] so it can be stored in a color, and save this into the texture data.
			tempVector.normalize();
			packTo01(tempVector);
			bytePtr[0] = (unsigned char) (tempVector.x() * 255);
			bytePtr[1] = (unsigned char) (tempVector.y() * 255);
			bytePtr[2] = (unsigned char) (tempVector.z() * 255);
			bytePtr += 3;
		}
	}
	//Now we upload this face of the cube to OpenGL. We tell it that this is the positive x face of the current cube map, and where to find the data.
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, GL_RGBA8, size, size, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

	//negative x
	bytePtr = data;
	//Loop through the pixels in the face.
	for (int j = 0; j < size; j++) {
		for (int i = 0; i < size; i++) {
			//Calculate the vector from the center of the cube to this texel.
			tempVector.x() = - halfSize;
			tempVector.y() = - (j + offset - halfSize);
			tempVector.z() = (i + offset - halfSize);
			//We normalize this vector, pack it to [0, 1] so it can be stored in a color, and save this into the texture data.
			tempVector.normalize();
			packTo01(tempVector);
			bytePtr[0] = (unsigned char) (tempVector.x() * 255);
			bytePtr[1] = (unsigned char) (tempVector.y() * 255);
			bytePtr[2] = (unsigned char) (tempVector.z() * 255);
			bytePtr += 3;
		}
	}
	//Now we upload this face of the cube to OpenGL. We tell it that this is the positive x face of the current cube map, and where to find the data.
	glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, GL_RGBA8, size, size, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

	//negative y
	bytePtr = data;
	//Loop through the pixels in the face.
	for (int j = 0; j < size; j++) {
		for (int i = 0; i < size; i++) {
			//Calculate the vector from the center of the cube to this texel.
			tempVector.x() = (i + offset - halfSize);
			tempVector.y() = - halfSize;
			tempVector.z() = - (j + offset - halfSize);
			//We normalize this vector, pack it to [0, 1] so it can be stored in a color, and save this into the texture data.
			tempVector.normalize();
			packTo01(tempVector);
			bytePtr[0] = (unsigned char) (tempVector.x() * 255);
			bytePtr[1] = (unsigned char) (tempVector.y() * 255);
			bytePtr[2] = (unsigned char) (tempVector.z() * 255);
			bytePtr += 3;
		}
	}
	//Now we upload this face of the cube to OpenGL. We tell it that this is the positive x face of the current cube map, and where to find the data.
	glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, GL_RGBA8, size, size, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

	//negative z
	bytePtr = data;
	//Loop through the pixels in the face.
	for (int j = 0; j < size; j++) {
		for (int i = 0; i < size; i++) {
			//Calculate the vector from the center of the cube to this texel.
			tempVector.x() = - (i + offset - halfSize);
			tempVector.y() = - (j + offset - halfSize);
			tempVector.z() = - halfSize;
			//We normalize this vector, pack it to [0, 1] so it can be stored in a color, and save this into the texture data.
			tempVector.normalize();
			packTo01(tempVector);
			bytePtr[0] = (unsigned char) (tempVector.x() * 255);
			bytePtr[1] = (unsigned char) (tempVector.y() * 255);
			bytePtr[2] = (unsigned char) (tempVector.z() * 255);
			bytePtr += 3;
		}
	}
	//Now we upload this face of the cube to OpenGL. We tell it that this is the positive x face of the current cube map, and where to find the data.
	glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, GL_RGBA8, size, size, 0, GL_RGB, GL_UNSIGNED_BYTE, data);

	//After repeating this for each face of the cube, we are done. Don't forget to delete the temporary data storage.
	delete[] data;

	return;
}

/*****************************************************************************/
void StructureGL::resetAllPlaneTextures(){
	resetTextures(mPlaneTextureIDs);
	resetTextures(mNormalMapIDs);
	resetTextures(mHeightMapIDs);
	resetTextures(mPlaneAlphaIDs);
}

/*****************************************************************************/

void StructureGL::resetTextures(TexIDs& textureIDs){
	if(mPlaneTextureMutex.tryLock()){
		for (TexIDs::const_iterator tex_it = textureIDs.begin(); tex_it != textureIDs.end(); ++tex_it){
			mTexIDsToRemove.push_back(*tex_it);
		}
		textureIDs.clear();
		mPlaneTextureMutex.unlock();
	}
}

/*****************************************************************************/

void StructureGL::removedMarkedTextures(){
	if(mPlaneTextureMutex.tryLock(10)){
		for(TexIDs::const_iterator tex_it = mTexIDsToRemove.begin(); tex_it != mTexIDsToRemove.end(); ++tex_it){
			glDeleteTextures(1, &*tex_it);
		}
		mTexIDsToRemove.clear();
		mPlaneTextureMutex.unlock();
	}
}

/****************** VIEWPOINT ********************/

void StructureGL::Viewpoint::translate(float forward, float up, float sideways) {
	mZ -= forward * cos(mYaw) + sideways * sin(mYaw);
	mX -= forward * sin(mYaw) - sideways * cos(mYaw);
	mY += up;
}

/*****************************************************************************/

void StructureGL::Viewpoint::rotate(float yaw, float pitch) {
	mYaw += yaw;
	mPitch += pitch;
	if (mPitch < -M_PI_2)
		mPitch = -M_PI_2;
	if (mPitch > M_PI_2)
		mPitch = M_PI_2;
}

/*****************************************************************************/

void StructureGL::Viewpoint::applyGL() {
	glRotatef(-180.0f * mYaw / M_PI, 0.0f, 1.0f, 0.0f);
	glRotatef(-180.0f * mPitch / M_PI, cos(mYaw), 0.0f, -sin(mYaw));
	glTranslatef(-mX, -mY, -mZ);
}

/*****************************************************************************/

void StructureGL::Viewpoint::applyVelocity(float dt) {
	rotate(mVelTurn * dt, 0.0f);
	translate(mVelForward * dt, mVelUp * dt, mVelSide * dt);
}

