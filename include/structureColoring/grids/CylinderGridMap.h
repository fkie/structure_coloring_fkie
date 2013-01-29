/*
 * Copyright (c) 2013, Fraunhofer FKIE
 *
 * Authors: Jochen Welle
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

#ifndef CYLINDER_GRIDMAP_H_
#define CYLINDER_GRIDMAP_H_

#include "Grid2D.h"
#include "../structures/CylinderPatch.h"

class CylinderGridMap: public Grid2D<bool>
{
public:
    typedef Grid2D<bool> BoolGrid;
	typedef Eigen::Vector3f Vec3;

    template<typename PointCloudT>
    CylinderGridMap(float cellSize, const CylinderPatch::CylinderPatchPtr& cylinderPatch, const PointCloudT& pointCloud);

private:
    typedef CylinderPatch::PointIndices PntInd;

    void pointToHeigthAngle(float& height, float& angle, const Vec3& p) const;
    void heigthAngleToXY(unsigned int& x, unsigned int& y, float height, float angle) const;
    template<typename PointT>
    void addPoint(const PointT& p);
    void addVec3(const Vec3& p);
    template<typename PointCloudT>
    void addPoints(const PntInd& indices, const PointCloudT& pointCloud);

    float mCellSize;
    CylinderPatch::CylinderPatchPtr mCylinder;
};

template<typename PointCloudT>
CylinderGridMap::CylinderGridMap(float cellSize, const CylinderPatch::CylinderPatchPtr& cylinderPatch, const PointCloudT& pointCloud)
    : BoolGrid(((cylinderPatch->getRadius()*2*M_PI)/cellSize)+1, (cylinderPatch->getHeight()/cellSize)+1, false),
    mCellSize(cellSize),
    mCylinder(cylinderPatch)
{
    addPoints(mCylinder->getInliers(), pointCloud);
}

template<typename PointT>
void CylinderGridMap::addPoint(const PointT& p)
{
    Vec3 v(p.x, p.y, p.z);
    this->addVec3(v);
}

template<typename PointCloudT>
void CylinderGridMap::addPoints(const PntInd& indices, const PointCloudT& pointCloud)
{
    //ROS_INFO("CylinderGridMap: adding %zu, %zu", indices.size(), pointCloud.points.size());
    for (PntInd::const_iterator it = indices.begin(); it != indices.end(); ++it)
    {
        this->addPoint(pointCloud.points[*it]);
    }
}

#endif
