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

#include <structureColoring/grids/GridMap.h>
#include <structureColoring/histograms/OutOfRangeException.h>

//#include <iostream>

GridMap::GridMap(const GridMap& gridMap, float xMin, float xMax, float yMin, float yMax,
		unsigned int xOff, unsigned int xAdd, unsigned int yOff, unsigned int yAdd,
		float cellSize)
:PositionedGrid2D<NodeGridCell>(gridMap.getPlane3D(), xMin, xMax, yMin, yMax, cellSize){
	if(gridMap.getXMin() < xMin || gridMap.getXMax() > xMax
			|| gridMap.getYMin() < yMin || gridMap.getYMax() > yMax){
//		std::cout << "INPUT: xmin " << xMin << "  xmax " << xMax << "  ymin " << yMin << "  ymax "<< yMax << std::endl;
//		std::cout << "GridMap dimensions: xmin " << getXMin() << "  xmax " << getXMax() << "  ymin " << getYMin() << "  ymax "<< getYMax() << std::endl;
		throw OutOfRangeException("Min and Max (x and y) have to include gridMap");
	}
//	std::cout << "New GridMap from constructor, cellsize (" << cellSize << "), width ("
//			<< getWidth() << ") height ("<< getHeight() << ")"<< std::endl;
//	std::cout << "INPUT: xmin " << xMin << "  xmax " << xMax << "  ymin " << yMin << "  ymax "<< yMax << std::endl;
//	std::cout << "GridMap dimensions: xmin " << getXMin() << "  xmax " << getXMax() << "  ymin " << getYMin() << "  ymax "<< getYMax() << std::endl;
    float scale = round(gridMap.getCellSize() / cellSize);
	unsigned int regionSize = scale +0.5f; // round to nearest integer
//	assert(regionSize >= 0);
	if (regionSize == 0)
	{
	// TODO why is regionSize 0
		printf("scale: %f, oldCellSize: %f, newCellSize: %f", scale, gridMap.getCellSize(),cellSize);
		regionSize = 1;
	}
	for (unsigned int i = 0; i < gridMap.getWidth(); i++) {
		for (unsigned int j = 0; j < gridMap.getHeight(); j++) {
			if (!gridMap.Grid2D<NodeGridCell>::operator()(i, j).populated())
				continue;
			unsigned int newX = static_cast<unsigned int> ((float(xOff) + scale * float(i)) +0.5f);
			unsigned int newY = static_cast<unsigned int> ((float(yOff) + scale * float(j)) +0.5f);
			for (unsigned int k = 0; k < regionSize; ++k) {
				if (newX + k >= mWidth)
					continue;
				for (unsigned int l = 0; l < regionSize; ++l) {
					if (newY + l >= mHeight)
						continue;
					this->Grid2D<NodeGridCell>::operator()(newX + k, newY + l).blindPopulate();
				}
			}
		}
	}

	mGridCCStartPos.first += xOff;
	mGridCCStartPos.second += yOff;
}

void GridMap::pushPopulatedUnvisitedNeighbors(std::queue<std::pair<unsigned int, unsigned int> >& neighbors, const unsigned int& width, const unsigned int& height) const{
	assert((width<mWidth)&&(height<mHeight));
	//4-neighbors (straight ones)
	if(width > 0){
		std::pair<unsigned int, unsigned int> n = std::make_pair(width-1, height);
		if((!mGrid[getIndex(n.first, n.second, mWidth)].visited()) && (mGrid[getIndex(n.first, n.second, mWidth)].populated())){
			neighbors.push(n);
		}
	}
	if(width < mWidth-1){
		std::pair<unsigned int, unsigned int> n = std::make_pair(width+1, height);
		if((!mGrid[getIndex(n.first, n.second, mWidth)].visited()) && (mGrid[getIndex(n.first, n.second, mWidth)].populated())){
			neighbors.push(n);
		}
	}
	if(height > 0){
		std::pair<unsigned int, unsigned int> n = std::make_pair(width, height-1);
		if((!mGrid[getIndex(n.first, n.second, mWidth)].visited()) && (mGrid[getIndex(n.first, n.second, mWidth)].populated())){
			neighbors.push(n);
		}
	}
	if(height < mHeight-1){
		std::pair<unsigned int, unsigned int> n = std::make_pair(width, height+1);
		if((!mGrid[getIndex(n.first, n.second, mWidth)].visited()) && (mGrid[getIndex(n.first, n.second, mWidth)].populated())){
			neighbors.push(n);
		}
	}
	//8-neighbors (diagonal ones)
	if(width > 0 && height > 0){
		std::pair<unsigned int, unsigned int> n = std::make_pair(width-1, height-1);
		if((!mGrid[getIndex(n.first, n.second, mWidth)].visited()) && (mGrid[getIndex(n.first, n.second, mWidth)].populated())){
			neighbors.push(n);
		}
	}
	if(width < mWidth-1 && height > 0){
		std::pair<unsigned int, unsigned int> n = std::make_pair(width+1, height-1);
		if((!mGrid[getIndex(n.first, n.second, mWidth)].visited()) && (mGrid[getIndex(n.first, n.second, mWidth)].populated())){
			neighbors.push(n);
		}
	}
	if(width > 0 && height < mHeight-1){
		std::pair<unsigned int, unsigned int> n = std::make_pair(width-1, height+1);
		if((!mGrid[getIndex(n.first, n.second, mWidth)].visited()) && (mGrid[getIndex(n.first, n.second, mWidth)].populated())){
			neighbors.push(n);
		}
	}
	if(width < mWidth-1 && height < mHeight-1){
		std::pair<unsigned int, unsigned int> n = std::make_pair(width+1, height+1);
		if((!mGrid[getIndex(n.first, n.second, mWidth)].visited()) && (mGrid[getIndex(n.first, n.second, mWidth)].populated())){
			neighbors.push(n);
		}
	}
}

/*****************************************************************************/

void GridMap::print() const{
	std::cout << "printing grid, width = " << mWidth << " height = " << mHeight << std::endl;
	for(unsigned int h = 0; h < mHeight; h++){
		std::cout << "|";
		for(unsigned int w = 0; w < mWidth; w++){
			if (mGrid[getIndex(w, h, mWidth)].populated()) std::cout << "x|";
			else std::cout << "-|";
		}
		std::cout << std::endl;
	}
}

void GridMap::print(const std::pair<unsigned int, unsigned int>& coords) const{
	std::cout << "printing grid, width = " << mWidth << " height = " << mHeight << std::endl;
	for(unsigned int h = 0; h < mHeight; h++){
		std::cout << "|";
		for(unsigned int w = 0; w < mWidth; w++){
			if (mGrid[getIndex(w, h, mWidth)].populated())
				if(w == coords.first && h == coords.second)
					std::cout << "O|";
				else std::cout << "x|";
			else if(w == coords.first && h == coords.second)
				std::cout << "O|";
			else std::cout << "-|";
		}
		std::cout << std::endl;
	}
	std::cout << coords.first << " " << coords.second << std::endl;
}

/*****************************************************************************/

void GridMap::startConnectedComponentAt(ConnectedComponent& ccc, const Indices& startPos) {
	std::queue<Indices> neighborsQueue;
	assert((startPos.first < getWidth()) && (startPos.second < getHeight()));
	if ((!(*this)(startPos.first, startPos.second).visited()) && ((*this)(startPos.first, startPos.second).populated())) {
		neighborsQueue.push(startPos);
	}
	while (!neighborsQueue.empty()) {
		Indices xy = neighborsQueue.front();
		neighborsQueue.pop();
		if ((!((*this)(xy.first, xy.second).visited())) && ((*this)(xy.first, xy.second).populated())) {
			//fill queue with neighboring cells
			pushPopulatedUnvisitedNeighbors(neighborsQueue, xy.first, xy.second);
			//put current cell to current connected component
			(*this)(xy.first, xy.second).visit();
			ccc.addCell(xy);
			for (std::vector<unsigned int>::const_iterator pop_it = (*this)(xy.first, xy.second).getPopulation().begin(); pop_it
					!= (*this)(xy.first, xy.second).getPopulation().end(); pop_it++) {
				ccc.addNumberOfPoints(1);
			}
		}
	}
}

/*****************************************************************************/

void GridMap::checkNodesAgainstGridConnection(NodeIndices& outNodeIndices, const NodePointers& inNodeCandidates, float newCellSize,
		unsigned int minNodesCount) const {
	float xMin = 0.f, yMin = 0.f, xMax = 0.f, yMax = 0.f;
	unsigned int xOff = 0, xAdd = 0, yOff = 0, yAdd = 0;
	float nodeXMin = std::numeric_limits<float>::max();
	float nodeXMax = -std::numeric_limits<float>::max();
	float nodeYMin = std::numeric_limits<float>::max();
	float nodeYMax = -std::numeric_limits<float>::max();
	outNodeIndices.reserve(inNodeCandidates.size());
	for(NodePointers::const_iterator nit = inNodeCandidates.begin(); nit != inNodeCandidates.end(); ++nit){
		Vec3 tPos(this->getPlane3D().transformToXYPlane((*nit)->value_.meanPos));
		if(tPos.x() < nodeXMin) nodeXMin = tPos.x();
		if(tPos.x() > nodeXMax) nodeXMax = tPos.x();
		if(tPos.y() < nodeYMin) nodeYMin = tPos.y();
		if(tPos.y() > nodeYMax) nodeYMax = tPos.y();
	}
	this->getNewExtremes(xMin, xMax, yMin, yMax, xOff, xAdd, yOff, yAdd, nodeXMin, nodeXMax, nodeYMin, nodeYMax, newCellSize);
	GridMap testGrid(*this, xMin, xMax, yMin, yMax, xOff, xAdd, yOff, yAdd, newCellSize);
	testGrid.calculateStartPos();
	//register OcTreeNodes with grid
	testGrid.populate(inNodeCandidates);
	//visit start-gridcell and start bfs or dfs if it is populated and not yet visited
	//store connected component -> store for each node, which component it is in
	ConnectedComponent ccc;
	testGrid.startConnectedComponentAt(ccc, testGrid.getStartPos());
	if (ccc.getNumPoints() > minNodesCount) { // minimum number of Nodes in one connected Component
		testGrid.gatherNodesInCC(outNodeIndices, ccc);
	}
}

/*****************************************************************************/

void GridMap::gatherNodesInCC(NodePointers& outputNodes, const ConnectedComponent& conComp,	const NodePointers& octreeNodes) const {
	for (unsigned int l = 0; l < conComp.getCells().size(); l++) {
		const uints& gridPopulation = (*this)(conComp.getCells()[l].first, conComp.getCells()[l].second).getPopulation();
		for (unsigned int j = 0; j < gridPopulation.size(); ++j) {
			const unsigned int& node_idx = gridPopulation[j];
			assert(node_idx < octreeNodes.size());
			outputNodes.push_back(octreeNodes[node_idx]);
		}
	}
}

/*****************************************************************************/

void GridMap::gatherNodesInCC(NodeIndices& outputIndices, const ConnectedComponent& conComp) const {
	for (unsigned int l = 0; l < conComp.getCells().size(); l++) {
		const uints& gridPopulation = (*this)(conComp.getCells()[l].first, conComp.getCells()[l].second).getPopulation();
		for (unsigned int j = 0; j < gridPopulation.size(); ++j) {
			const unsigned int& node_idx = gridPopulation[j];
			outputIndices.push_back(node_idx);
		}
	}
}

/*****************************************************************************/

void GridMap::getConnectedComponentsAndNotConnectedNodes(NodePointersVec& nodesInCC, const NodePointers& inputOctreeNodes, float cellSize,
		unsigned int minPlaneNodes, unsigned int minCCNodes, NodePointers* notConnectedNodesOutput){
	//visit every gridcell and start bfs or dfs if it is populated and not yet visited
	//store connected components -> store for each node, which component it is in
	ConnectedComponent ccc;
	this->startConnectedComponentAt(ccc, getStartPos());
	if (ccc.getNumPoints() > minPlaneNodes) {
		if (ccc.getNumPoints() > minCCNodes) { // minimum number of Nodes in one connected Component
			NodePointers ccNodes;
			this->gatherNodesInCC(ccNodes, ccc, inputOctreeNodes);
			nodesInCC.push_back(ccNodes);
		} else {
			// push_back nodes that are not connected with anything. These should be "freed" for HT primitiv detection
			if (notConnectedNodesOutput)
				this->gatherNodesInCC(*notConnectedNodesOutput, ccc, inputOctreeNodes);
		}
	}
	ccc.reset();
	for (unsigned int i = 0; i < mWidth; i++) {
		for (unsigned int j = 0; j < mHeight; j++) {
			this->startConnectedComponentAt(ccc, std::make_pair(i, j));
			if (ccc.getNumPoints() > std::min(minPlaneNodes, minCCNodes)) { // minimum number of Nodes in one connected Component
				NodePointers ccNodes;
				this->gatherNodesInCC(ccNodes, ccc, inputOctreeNodes);
				nodesInCC.push_back(ccNodes);
			} else {
				// push_back nodes that are not connected with anything. These should be freed for HT primitiv detection
				if (notConnectedNodesOutput)
					this->gatherNodesInCC(*notConnectedNodesOutput, ccc, inputOctreeNodes);
			}
			ccc.reset();
		}
	}

}

/*****************************************************************************/

void GridMap::getNewExtremes(float& xMinOut, float& xMaxOut, float& yMinOut, float& yMaxOut,
		unsigned int& xOff, unsigned int& xAdd, unsigned int& yOff, unsigned int& yAdd,
		float xMinIn, float xMaxIn, float yMinIn, float yMaxIn, float stepSize) const {
	xOff = xAdd = yOff = yAdd = 0;
	if (mXMin > xMinIn){
		xOff = std::ceil((mXMin - xMinIn) / stepSize);
		xMinOut = mXMin - xOff * stepSize;
	} else {
		xMinOut = mXMin;
	}
	if (mXMax < xMaxIn){
		xAdd = std::ceil((xMaxIn - mXMax) / stepSize);
		xMaxOut = mXMax + xAdd * stepSize;
	} else {
		xMaxOut = mXMax;
	}
	if (mYMin > yMinIn){
		yOff = std::ceil((mYMin - yMinIn) / stepSize);
		yMinOut = mYMin - yOff * stepSize;
	} else {
		yMinOut = mYMin;
	}
	if (mYMax < yMaxIn){
		yAdd = std::ceil((yMaxIn - mYMax) / stepSize);
		yMaxOut = mYMax + yAdd * stepSize;
	} else {
		yMaxOut = mYMax;
	}
//	for(xMinOut = mXMin; xMinIn < xMinOut; xMinOut -= stepSize) ++xOff;
//	for(xMaxOut = mXMax; xMaxIn > xMaxOut; xMaxOut += stepSize) ++xAdd;
//	for(yMinOut = mYMin; yMinIn < yMinOut; yMinOut -= stepSize) ++yOff;
//	for(yMaxOut = mYMax; yMaxIn > yMaxOut; yMaxOut += stepSize) ++yAdd;
}

/*****************************************************************************/

//this "solution" is hacky and should not be called very often
void GridMap::calculateStartPos(){
	for(unsigned int w=0; w<mWidth; w++){
		for(unsigned int h=0; h<mHeight; h++){
			if((*this)(w, h).populated()){
				mGridCCStartPos = std::make_pair(w, h);
				return;
			}
		}
	}
    assert(false); // if we are here, the grid size is zero or the grid is empty
}
