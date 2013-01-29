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

#ifndef GRIDMAP_H_
#define GRIDMAP_H_

#include "PositionedGrid2D.h"
#include "NodeGridCell.h"
#include "ConnectedComponent.h"
#include <structureColoring/OcTree.h>
#include <pcl/point_types.h>
#include <assert.h>
#include <queue>

class GridMap: public PositionedGrid2D<NodeGridCell>{
public:
	typedef NodeGridCell Cell;
	typedef std::vector<int> PointIndices;
	typedef OcTree::NodePointers NodePointers;
	typedef std::vector<NodePointers> NodePointersVec;
	typedef std::vector<int> NodeIndices;
	typedef std::vector<unsigned int> uints;

//constructors
	GridMap() : mGridCCStartPos(std::make_pair(0, 0)){}

	GridMap(const Plane3D& plane, float xMin, float xMax, float yMin, float yMax, float cellSize)
		: PositionedGrid2D<Cell>(plane, xMin, xMax, yMin, yMax, cellSize){}

	//get new x/y Min/Max from "getNewExtremes"-method!
	GridMap(const GridMap& gridMap, float xMin, float xMax, float yMin, float yMax,
			unsigned int xOff, unsigned int xAdd, unsigned int yOff, unsigned int yAdd,
			float cellSize);

//getter
	bool isPopulated(Vec3 pos){
		return (*this)(pos).populated();
	}
	const Indices& getStartPos() const {return mGridCCStartPos;}

	using Grid2D<NodeGridCell>::operator();
	using PositionedGrid2D<NodeGridCell>::operator();

//setter
	void populate(const unsigned int& index, Vec3 pos){ (*this)(pos).populate(index); }

	template<typename PointCloudType>
	void populate(const PointIndices& pointIndices, const PointCloudType& pointCloud);

	void populate(const NodePointers& nodePointers);

	void blindPopulate(Vec3 pos){ (*this)(pos).blindPopulate(); }

	template<typename PointCloudType>
	void blindPopulate(const PointIndices& pointIndices, const PointCloudType& pointCloud);

	void clearIndices();
	void clear();

	void pushPopulatedUnvisitedNeighbors(std::queue<std::pair<unsigned int, unsigned int> >& neighbors, const unsigned int& width, const unsigned int& height) const;

	void print() const;
	void print(const std::pair<unsigned int, unsigned int>& coords) const;

//calculations
	void startConnectedComponentAt(ConnectedComponent& ccc, const Indices& startPos);
	void checkNodesAgainstGridConnection(NodeIndices& outNodeIndices, const NodePointers& inNodeCandidates, float newCellSize,
			unsigned int minNodesCount) const;
	void gatherNodesInCC(NodePointers& outputNodes, const ConnectedComponent& conComp, const NodePointers& octreeNodes) const;
	void gatherNodesInCC(NodeIndices& outputIndices, const ConnectedComponent& conComp) const;
	void getConnectedComponentsAndNotConnectedNodes(NodePointersVec& nodesInCC, const NodePointers& inputOctreeNodes, float cellSize,
			unsigned int minPlaneNodes, unsigned int minCCNodes, NodePointers* notConnectedNodesOutput = NULL);

	void getNewExtremes(float& xMinOut, float& xMaxOut, float& yMinOut, float& yMaxOut,
			unsigned int& xOff, unsigned int& xAdd, unsigned int& yOff, unsigned int& yAdd,
			float xMinIn, float xMaxIn, float yMinIn, float yMaxIn, float stepsize) const;

	bool checkPointConnection(const Vec3& p, const int& connectionNeighbors) const;

	void calculateStartPos();

protected:
//members
	Indices mGridCCStartPos;
};

/*****************************************************************************/

template<typename PointCloudType>
void GridMap::populate(const PointIndices& pointIndices, const PointCloudType& pointCloud){
	for(PointIndices::const_iterator pi_it = pointIndices.begin(); pi_it != pointIndices.end(); ++pi_it){
		populate((unsigned int)(*pi_it), pointCloud.points[*pi_it].getVector3fMap());
	}
}

/*****************************************************************************/

inline void GridMap::populate(const NodePointers& nodePointers){
	for(unsigned int i = 0; i < nodePointers.size(); ++i){
		populate(i, nodePointers[i]->value_.meanPos);
	}
}

/*****************************************************************************/

template<typename PointCloudType>
void GridMap::blindPopulate(const PointIndices& pointIndices, const PointCloudType& pointCloud){
	for(PointIndices::const_iterator pi_it = pointIndices.begin(); pi_it != pointIndices.end(); ++pi_it){
		blindPopulate(pointCloud.points[*pi_it].getVector3fMap());
	}
}

/*****************************************************************************/

inline void GridMap::clearIndices(){
	for (Cells::iterator cell_iter = mGrid.begin(); cell_iter != mGrid.end(); ++cell_iter){
		cell_iter->clearIndices();
	}
}

/*****************************************************************************/

inline void GridMap::clear(){
	for (Cells::iterator cell_iter = mGrid.begin(); cell_iter != mGrid.end(); ++cell_iter){
			*cell_iter = Cell();
	}
}

/*****************************************************************************/

inline bool GridMap::checkPointConnection(const Vec3& p, const int& connectionNeighbors) const {
	Cells cells;
	getNeighbors(cells, p, connectionNeighbors);
	for(Cells::const_iterator cit = cells.begin(); cit != cells.end(); ++cit){
		if (cit->populated()){
//			std::cout << "connection found in grid, point/node ist connected!" << std::endl;
			return true;
		}
	}
//	std::cout << "no connection found in grid" << std::endl;
	return false;
}

/*****************************************************************************/

#endif /* GRIDMAP_H_ */
