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

#ifndef OCTREE_H_
#define OCTREE_H_

#include <boost/shared_ptr.hpp>
#include <octreelib/spatialaggregate/octree.h>
#include <octreelib/algorithm/downsample.h>
#include "NormalEstimationValueClass.h"
#include <list>
#include <vector>

class OcTree {
public:
	typedef NormalEstimationValueClass ValueClass;
	typedef spatialaggregate::OcTree<float, ValueClass> Octree;
	typedef Octree* OctreePtr;
	typedef spatialaggregate::OcTreeNodeFixedCountAllocator<float, ValueClass> Allocator;
	typedef boost::shared_ptr<Allocator> AllocatorPtr;
	typedef algorithm::OcTreeSamplingMap<float, ValueClass> SamplingMap;
	typedef spatialaggregate::OcTreeNode<float, ValueClass>* NodePtr;
	typedef spatialaggregate::OcTreeKey<float, ValueClass> OctreeKey;
	typedef std::vector<NodePtr> NodePointers;
	typedef std::list<NodePtr> NodePointerList;
	typedef std::vector<int> PointIndices;
	typedef std::vector<PointIndices> NodeIdToPointIdxVector;
	typedef Eigen::Vector3f Vec3;

	OcTree(float minOctreeResolution, float rho_max, float principalVarFactor, float minPointsForNormal,
			float curvThreshold, float curv2Threshold, float sqDistFactor, size_t numPoints) :
		mOctreePtr(NULL), mOctreeDepth(0), mMinOctreeResolution(minOctreeResolution), mRho_max(rho_max),
				mPrincipalVarFactor(principalVarFactor), mMinPointsForNormal(minPointsForNormal), mCurvThreshold(
						curvThreshold), mCurv2Threshold(curv2Threshold) , mSqDistFactor(sqDistFactor){
		preAllocate(numPoints);
	}

	~OcTree() {
		if (mOctreePtr != NULL)
			delete mOctreePtr;
	}

	void preAllocate(const size_t& numPoints);

	template<typename PointCloudType>
	void buildOctree(const PointCloudType& pointCloud);

	bool checkSamplingMap() const
	{
		for(unsigned int depth = 0; depth <= mOctreeDepth; ++depth)
		{
			NodePointers octreeNodes(getNodesOnDepth(depth));
			if(!checkOctreeNodes(octreeNodes))
			{
				std::cout << "checkSamplingMap test failed on depth " << depth << " of " << mOctreeDepth << std::endl;
				return false;
			}
		}
		return true;
	}

	bool checkGetAllLeafs() const
	{
		NodePointers octreeNodes;
		getAllLeafs(octreeNodes, mOctreePtr->root_);
		return checkOctreeNodes(octreeNodes);
	}

	bool checkOctreeNodes(const NodePointers& nodePointers) const
	{
		for(NodePointers::const_iterator np_it = nodePointers.begin(); np_it != nodePointers.end(); ++np_it)
		{
			if((*np_it)->value_.id >= mNodeIdToPointIdxVector.size())
			{
				return false;
			}
		}
		return true;
	}

	const unsigned int& getMaxDepth() const {
		return mOctreeDepth;
	}

	const SamplingMap& getSamplingMap() const {
		return mSamplingMap;
	}

	const NodePointerList& getNodeListOnDepth(const unsigned int& depth) const {
		assert(depth <= mOctreeDepth);
		return mSamplingMap.find(depth)->second;
	}

	NodePointers getNodesOnDepth(const unsigned int& depth) const {
		assert(depth <= mOctreeDepth);
		NodePointerList nodePointerList = mSamplingMap.find(depth)->second;
		NodePointers nodePointers;
		for(NodePointerList::const_iterator list_it = nodePointerList.begin(); list_it != nodePointerList.end(); ++list_it)
		{
			nodePointers.push_back(*list_it);
		}
		return nodePointers;
	}

	const OctreePtr& getOctreePtr() const
	{
		return mOctreePtr;
	}

	const PointIndices& getPointsFromNodeId(const unsigned int& nodeId) const {
		assert(nodeId < mNodeIdToPointIdxVector.size());
		return mNodeIdToPointIdxVector[nodeId];
	}

	const PointIndices& getPointsFromNodePtr(const NodePtr& nodePtr) const {
		const unsigned int& nodeId = nodePtr->value_.id;
		return getPointsFromNodeId(nodeId);
	}

	float getCellSizeFromDepth(const unsigned int& depth) const {
		return mOctreePtr->volumeSizeForDepth((int) depth);
	}

	void findFinestNormal(Vec3& normal, const Vec3& point) const{//TODO handle Vec3f -> Mat41
		NodePtr nodePtr = mOctreePtr->root_->findRepresentative(Eigen::Vector4f(point(0), point(1), point(2), 1.f), mMinOctreeResolution);
		while(!nodePtr->value_.stable){//there must be a stable node (with normal) for every point, that comes from a surfel (stable node with normal)
			nodePtr = nodePtr->parent_;
		}
		normal = nodePtr->value_.normal;
	}

	void getNeighboringNodes(NodePointers& neighborNodes, const NodePtr& node, const unsigned int& maxSearchDepth,
			const float& neighborhoodSize = 1.f) const;

	void getNeighboringNodes(NodePointers& neighborNodes, const NodePtr& currNode, const unsigned int& maxSearchDepth,
			const unsigned int& currDepth, const OctreeKey& minPos, const OctreeKey& maxPos) const;

	void getAllLeafs(NodePointers& leafNodes, const NodePtr& topNode) const;
private:
	AllocatorPtr mAllocatorPtr;
	OctreePtr mOctreePtr;
	unsigned int mOctreeDepth;
	SamplingMap mSamplingMap;
	NodeIdToPointIdxVector mNodeIdToPointIdxVector;

	// parameters
	float mMinOctreeResolution;
	float mRho_max;
	float mPrincipalVarFactor;
	float mMinPointsForNormal;
	float mCurvThreshold;
	float mCurv2Threshold;
	float mSqDistFactor;
};

inline void OcTree::preAllocate(const size_t& numPoints) {
	mAllocatorPtr.reset(new Allocator(numPoints));
}

template<typename PointCloudType>
void OcTree::buildOctree(const PointCloudType& pointCloud) {
	mNodeIdToPointIdxVector.clear();
	mNodeIdToPointIdxVector.reserve(pointCloud.points.size());
	mAllocatorPtr->reset();

	if (mOctreePtr)
		delete mOctreePtr;
	Vec3 rootPos(0.f, 0.f, 0.f);
	mOctreePtr = new Octree(Eigen::Vector4f(rootPos(0), rootPos(1), rootPos(2), 1.f), mMinOctreeResolution, mRho_max, mAllocatorPtr);
	mOctreeDepth = 0;
	unsigned int idx = 0;
	for (unsigned int i = 0; i < pointCloud.points.size(); i++) {
		if (isnan(pointCloud.points[i].x) || isinf(pointCloud.points[i].x))
			continue;

		if (isnan(pointCloud.points[i].y) || isinf(pointCloud.points[i].y))
			continue;

		if (isnan(pointCloud.points[i].z) || isinf(pointCloud.points[i].z))
			continue;
		Vec3 pos(pointCloud.points[i].getVector3fMap());
		ValueClass val;
		val.id = idx;
		val.numPoints = 1;
		val.summedSquares(0, 0) = pos(0) * pos(0); // xx
		val.summedSquares(0, 1) = val.summedSquares(1, 0) = pos(0) * pos(1); // xy
		val.summedSquares(0, 2) = val.summedSquares(2, 0) = pos(0) * pos(2); // xz
		val.summedSquares(1, 1) = pos(1) * pos(1); // yy
		val.summedSquares(1, 2) = val.summedSquares(2, 1) = pos(1) * pos(2); // yz
		val.summedSquares(2, 2) = pos(2) * pos(2); // zz
		val.summedPos(0) = pos(0);
		val.summedPos(1) = pos(1);
		val.summedPos(2) = pos(2);
		NodePtr n;
		if (mSqDistFactor > 0.f){
			float sqdist = val.summedSquares(0, 0) + val.summedSquares(1, 1) + val.summedSquares(2, 2);
			float minResolution = std::max(mMinOctreeResolution, mSqDistFactor * sqdist);
			n = mOctreePtr->addPoint(pos(0), pos(1), pos(2), val, mOctreePtr->depthForVolumeSize(minResolution));
		}
		else
			n = mOctreePtr->addPoint(pos(0), pos(1), pos(2), val, mOctreePtr->depthForVolumeSize(mMinOctreeResolution));
		if (n->value_.id == idx) {
			mNodeIdToPointIdxVector.push_back(std::vector<int>(1, i));
			idx++;
		} else {
			assert(n->value_.id <= pointCloud.points.size());
			mNodeIdToPointIdxVector[n->value_.id].push_back(i);
		}
		if ((unsigned int)n->depth_ > mOctreeDepth)
			mOctreeDepth = (unsigned int)n->depth_;
	}

	assert(checkGetAllLeafs());

	mSamplingMap.clear();
	mSamplingMap = algorithm::downsampleOcTree(*mOctreePtr, true, mOctreeDepth);
	for (unsigned int d = 0; d <= mOctreeDepth; d++) {
		const NodePointerList& octreeLayer = mSamplingMap[d];
		float principalVariance = mPrincipalVarFactor * getCellSizeFromDepth(d);
		principalVariance *= principalVariance;
		for(NodePointerList::const_iterator np_it = octreeLayer.begin(); np_it != octreeLayer.end(); ++np_it)
		{
			if (!(*np_it)->value_.stable)
			{
				calculateNormal((*np_it), Vec3(0.f, 0.f, 0.f), mMinPointsForNormal, mCurvThreshold, mCurv2Threshold, principalVariance);
			}
		}
	}

	assert(checkSamplingMap());
}

inline void OcTree::getNeighboringNodes(NodePointers& neighborNodes, const NodePtr& node, const unsigned int& maxSearchDepth,
		const float& neighborhoodSize) const {
	Vec3 size;
	size(0) = node->max_key_.getPosition(node->tree_)(0) - node->min_key_.getPosition(node->tree_)(0);
	size(1) = node->max_key_.getPosition(node->tree_)(1) - node->min_key_.getPosition(node->tree_)(1);
	size(2) = node->max_key_.getPosition(node->tree_)(2) - node->min_key_.getPosition(node->tree_)(2);
	float minPosX = node->pos_key_.getPosition(node->tree_)(0) - size(0) * neighborhoodSize;
	float minPosY = node->pos_key_.getPosition(node->tree_)(1) - size(1) * neighborhoodSize;
	float minPosZ = node->pos_key_.getPosition(node->tree_)(2) - size(2) * neighborhoodSize;
	OctreeKey minPos(minPosX, minPosY, minPosZ, node->tree_);
	float maxPosX = node->pos_key_.getPosition(node->tree_)(0) + size(0) * neighborhoodSize;
	float maxPosY = node->pos_key_.getPosition(node->tree_)(1) + size(1) * neighborhoodSize;
	float maxPosZ = node->pos_key_.getPosition(node->tree_)(2) + size(2) * neighborhoodSize;
	OctreeKey maxPos(maxPosX, maxPosY, maxPosZ, node->tree_);
	getNeighboringNodes(neighborNodes, mOctreePtr->root_, maxSearchDepth, 0, minPos, maxPos);
}

inline void OcTree::getNeighboringNodes(NodePointers& neighborNodes, const NodePtr& currNode, const unsigned int& maxSearchDepth,
		const unsigned int& currDepth, const OctreeKey& minPos, const OctreeKey& maxPos) const {
	if (currNode->type_ == spatialaggregate::OCTREE_LEAF_NODE || currDepth == maxSearchDepth) {
		if (currNode->overlap(minPos, maxPos) && !currNode->value_.nodeQuerried) {
			neighborNodes.push_back(currNode);
		}
	} else {
		for (unsigned int i=0; i<8; ++i){
			if (!currNode->children_[i]){
				continue;
			}
			if(currNode->children_[i]->overlap(minPos, maxPos))
				getNeighboringNodes(neighborNodes, currNode->children_[i], maxSearchDepth, currDepth+1, minPos, maxPos);
		}
	}
}

inline void OcTree::getAllLeafs(NodePointers& leafNodes, const NodePtr& topNode) const{
	if(topNode->type_ == spatialaggregate::OCTREE_LEAF_NODE){
		leafNodes.push_back(topNode);
	}
	else {
		// for all siblings
		// call getAllLeafs
		for(unsigned int i = 0; i < 8; i++){
			if(!topNode->children_[i])
				continue;
			getAllLeafs(leafNodes, topNode->children_[i]);
		}
	}
}
#endif /* OCTREE_H_ */
