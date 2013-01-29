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

#ifndef POSITIONEDGRID2D_H_
#define POSITIONEDGRID2D_H_

#include "Grid2D.h"
#include <structureColoring/structures/Plane3D.h>
#include <structureColoring/histograms/OutOfRangeException.h>

//#include <iostream>

template<typename CellType>
class PositionedGrid2D : public Grid2D<CellType>{
public:
	typedef OutOfRangeException OutOfRange;
	typedef std::pair<unsigned int, unsigned int> Indices;
	typedef std::pair<int, int> SignedIndices;
	typedef Eigen::Vector3f Vec3;
	typedef std::vector<CellType> Cells;

//constructor
	PositionedGrid2D(): mXMin(0.f), mXMax(0.f), mYMin(0.f), mYMax(0.f), mCellSize(0.f){}

	PositionedGrid2D(const Plane3D& plane, const float& xMin, const float& xMax,
			const float& yMin, const float& yMax, const float& cellSize)
	: Grid2D<CellType>(std::floor((xMax - xMin)/cellSize)+1, std::floor((yMax - yMin)/cellSize)+1),
		mPlane(plane), mXMin(xMin), mXMax(xMax), mYMin(yMin), mYMax(yMax), mCellSize(cellSize)
	{
		mXMax = mXMin + (float)Grid2D<CellType>::getWidth() * cellSize;
		mYMax = mYMin + (float)Grid2D<CellType>::getHeight() * cellSize;
	}

//getter
	const Plane3D& getPlane3D() const {return mPlane;}
	float getXMin() const {return mXMin;}
	float getXMax() const {return mXMax;}
	float getYMin() const {return mYMin;}
	float getYMax() const {return mYMax;}
	float getCellSize() const {return mCellSize;}

	typename Cells::const_reference operator() (const Vec3& pos) const {
		Indices indices;
		getIndices(indices, pos);
		return this->Grid2D<CellType>::operator() (indices);
	}
	typename Cells::reference operator() (const Vec3& pos) {
		Indices indices;
		getIndices(indices, pos);
		return this->Grid2D<CellType>::operator() (indices);
	}

//calculations
	void getNeighbors(Cells& cells, const Vec3& pos, unsigned int neighborhoodRadius) const {
		SignedIndices indices;
		getSignedIndicesNoException(indices, pos);
//		std::cout << "center point of get Neighbors is (" << indices.first << ", " << indices.second << ")" << std::endl;
		this->Grid2D<CellType>::getNeighbors(cells, indices, neighborhoodRadius);
	}

protected:
//helper
	void getIndices(Indices& outIndices, const Vec3& pos) const{
		Vec3 transformedPos = mPlane.transformToXYPlane(pos);
		if (transformedPos.x() < mXMin || transformedPos.x() > mXMax
				|| transformedPos.y() < mYMin || transformedPos.y() > mYMax){
			throw OutOfRange("key is out of range");
		}
		outIndices.first = std::floor((transformedPos.x() - mXMin)/mCellSize);
		outIndices.second = std::floor((transformedPos.y() - mYMin)/mCellSize);
	}

	void getSignedIndicesNoException(SignedIndices& outIndices, const Vec3& pos) const{
		Vec3 transformedPos = mPlane.transformToXYPlane(pos);
		outIndices.first = std::floor((transformedPos.x() - mXMin)/mCellSize);
		outIndices.second = std::floor((transformedPos.y() - mYMin)/mCellSize);
	}

//members
	Plane3D mPlane;
	float mXMin;
	float mXMax;
	float mYMin;
	float mYMax;
	float mCellSize;
};

#endif /* POSITIONEDGRID2D_H_ */
