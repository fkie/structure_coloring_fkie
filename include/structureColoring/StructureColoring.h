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

#ifndef _StructureColoring_h_
#define _StructureColoring_h_

#include <structureColoring/OcTree.h>
#include <structureColoring/grids/GridMap.h>
#include <structureColoring/grids/ConnectedComponent.h>
#include <structureColoring/structures/CylinderPatch.h>
#include <structureColoring/structures/PlanePatch.h>
#include <structureColoring/segcomp/rangeimageio.h>
#include <structureColoring/StrColParams.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <opencv/highgui.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>
#include <utility>
#include <QtCore/QMutex>
#include <omp.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>

// forward declarations:
class RosVisualization;
class SphereUniformSampling;
class NodePair;

class StructureColoring {
public:
	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef boost::shared_ptr<PointCloud> PointCloudPtr;

	typedef Plane3D::Plane3DPtr Plane3DPtr;
	typedef PlanePatch::PlanePatches PlanePatches;
	typedef PlanePatch::PlanePatchPtr PlanePatchPtr;
	typedef std::vector<PlanePatchPtr> PlanePatchVector;

	typedef Cylinder3D::Cylinder3DPtr Cylinder3DPtr;
	typedef CylinderPatch::CylinderPatches CylinderPatches;
	typedef CylinderPatch::CylinderPatchPtr CylinderPatchPtr;
	typedef std::vector<CylinderPatchPtr> CylinderPatchVector;

	typedef std::vector<NodePair> NodePairs;

	/**\brief StructureColoring Construct a new StrcutureColoring object from a nodeHandle and console parameters.
	 * \param n NodeHandle.
	 * \param argc Number of arguments.
	 * \param argv Arguments from console call.
	 */
	StructureColoring(ros::NodeHandle& n, int argc, char* argv[]);

	/**\brief StructureColoring Construct a new StructureColoring object from console parameters only.
	 * \param argc Number of arguments.
	 * \param argv Arguments from console call.
	 */
	StructureColoring(int argc, char* argv[]): mVis(NULL){init(argc, argv);}

	/**\brief StructureColoring Construct a new StructureColoring object without any params.
	 */
	StructureColoring(): mVis(NULL){init();}

	void init(ros::NodeHandle& n, int argc, char* argv[]);

	void init(int argc, char* argv[]);

	void init();

	/**\brief StructureColoring Destructor.
	 */
	virtual ~StructureColoring();

	/**\brief waitOnKinectMsgs Check if object waits for kinect messages.
	 */
	bool waitOnKinectMsgs(){return mParams.mKinect;}

	/**\brief waitOnLaserMsgs Check if object waits for laser messages.
	 */
	bool waitOnLaserMsgs(){return mParams.mLaser;}

	/**\brief pointCloud2Callback This method should be called via subscription to a PointCloud2 topic.
	 * \param msg Message pointer.
	 */
	void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& msg);

	/**\brief pointCloudCallback This method should be called via subscription to a PointCloud topic.
	 * \param msg Message pointer.
	 */
	void pointCloudCallback(const sensor_msgs::PointCloudConstPtr& msg);

	void readPointCloudFromPCD(PointCloud& pointCloud, const std::string& filename, unsigned int& width,
			unsigned int& height, std::vector<unsigned int>& undefPoints, const float& zMin, const float& zMax);
	void filterCloud(PointCloud& pointCloud, std::vector<unsigned int>& undefPoints, const float& zMin, const float& zMax);

	/**\brief getPointsFromFile Start segmentation. Parameters were given through object construction.
	 */
	void getPointsFromFile();

	void doSegmentation(PointCloudPtr pointCloud, PlanePatchVector& pointMapping, PlanePatches& extractedPlanes,
			CylinderPatchVector& pointMappingCylinders, CylinderPatches& extractedCylinders);

	void filterMsgSetupCloud(PointCloud& pointCloud, const sensor_msgs::PointCloud2ConstPtr& pMsg, double zMin, double zMax, bool fromKinect, bool writePic = false, const std::string& picFilename = "", unsigned int picCounter = 0);

	void setParams(StrColParams params){ mParams = params; }
protected:
	typedef boost::shared_ptr<GridMap> GridMapPtr;
	typedef OcTree::NodePtr NodePtr;
	typedef OcTree::ValueClass ValueClass;
	typedef OcTree::OctreePtr OctreePtr;
	typedef OcTree::Allocator OctreeAllocator;
	typedef OcTree::SamplingMap OctreeSamplingMap;
	typedef OcTree::NodePointers NodePointers;
	typedef OcTree::NodePointerList NodePointerList;
	typedef std::vector<int> NodeIndices;
	typedef std::vector<int> PointIndices;
	typedef Eigen::Vector3f Vec3;
	typedef Eigen::Matrix3f Mat3;
	typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3> > Points;
	typedef std::pair<PlanePatches::iterator, bool> PatchIterBoolPair;
	typedef std::vector<PatchIterBoolPair> PatchIterBoolPairVector;
	typedef std::vector<unsigned int> uints;
	typedef std::pair<PlanePatches::iterator, NodePointers> PatchIterNodePointersPair;
	typedef std::vector<PatchIterNodePointersPair> PatchIterNodePointersPairs;
	typedef std::list<NodePtr> NodePtrList;
	typedef std::vector<NodePtrList::iterator> NodePtrListIters;
	typedef std::pair<unsigned int, unsigned int> Pair2ui;
	typedef pcl::PointIndices pclPointIndices;
	typedef std::pair<PlanePatches::iterator, PointIndices > PlanePatchIterPointIndicesPair;
	typedef std::vector<PlanePatchIterPointIndicesPair> PlanePatchIterPointIndicesPairs;
	typedef std::map<PlanePatchPtr, NodePointers, CompPlanePtr> PatchToNodePointersMap;
	typedef std::vector<PlanePatchVector> NodeIndexToPlanePatchPointers;
	typedef std::map<PlanePatchPtr, NodeIndices, CompPlanePtr> PatchToNodeIndicesMap;
	typedef std::map<PlanePatchPtr, PointIndices, CompPlanePtr> PatchToPointIndicesMap;
	typedef std::vector<SphereUniformSampling> SphereUniformSamplings;
	typedef std::vector<int> PairIndices;

	struct SphereBin{
		unsigned int rho;
		unsigned int i; //(phi)
		unsigned int j; //(theta)
		std::list<int>::iterator pit;
		std::list<float>::iterator wit;
	};
	typedef std::vector<SphereBin> SphereBins;
	typedef std::vector<SphereBins> SphereBinsVec;

	/**\brief Get nodes from node indices. As this is very inefficient, you should only use this method for debugging. TODO remove this
	 * \param nodeIndices indices to nodes
	 * \param nodes input nodes
	 * \return copy of indexed NodePointers
	 */
	static NodePointers getNodesFromIndices(const NodeIndices& nodeIndices, const NodePointers& nodes);

	/**\brief Get points from point indices. As this is very inefficient, you should only use this method for debugging. TODO remove this
	 * \param pointIndices indices to points in cloud
	 * \param cloud input pointcloud
	 * \return copy of indexed Points in cloud
	 */
	static Points getPointsFromIndices(const PointIndices& pointIndices, const PointCloud& cloud);

	/**\brief mark a node as segmented
	 * \param currentNode mark this node
	 * \param nextNode doing nothing (for compatibility only)
	 * \param data doing nothing (for compatibility only)
	 */
	static void markAsSegmented(NodePtr currentNode, NodePtr nextNode = NULL, void* data = NULL);

	/**\brief If vec does not point to the viewport, it is flipped.
	 * \param vec Input and output vector to be flipped.
	 * \param viewport Input viewport position.
	 */
	static void flipToViewport(Vec3& vec, const Vec3& viewport);

	void parseInput(std::string& paramFilename, std::string& inputFilename, std::string& outputFilename, int argc, char* argv[]);
	void initNode(ros::NodeHandle& n);
	void initParams();
	void initOctree();
//	void parseParamFile(std::string filename);

	template<typename PointMappingT>
	void initSegmentation(PointMappingT& pointMapping, const PointCloud& pointCloud);

	double segmentPlanes(PlanePatches& extractedPlanes, PlanePatchVector& pointMapping, OcTree& octree, const PointCloudPtr& pointCloud);
	double segmentPlanesSAC(PlanePatches& extractedPlanes, PlanePatchVector& pointMapping, const PointCloudPtr& pointCloud);
	double segmentCylinders(CylinderPatches& cylinderPatches, CylinderPatchVector& pointMapping, OcTree& octree, const PointCloudPtr& pointCloud);
    double segmentCylindersSAC(CylinderPatches& cylinderPatches, CylinderPatchVector& pointMapping, OcTree& octree, const PointCloudPtr& pointCloud);
	void generateTextures(const PlanePatches& extractedPlanes, const PointCloud& pointCloud);
	void callPublisher(const OcTree& octree, PlanePatches& extractedPlanes, const CylinderPatches& extractedCylinders , const PointCloudPtr pointCloud,
			const PlanePatchVector& pointMapping, const CylinderPatchVector& cylinderPointMapping);

	void filterConnectedNodes(std::vector<NodePointers>& CCInlierNodes, const Plane3D& pp, const NodePointers& octreeNodes,
			float cellSize, GridMap* oldGridMap = NULL, NodePointers* notConnectedNodesOutput = NULL);

	void estimatePlane(const pclPointIndices::Ptr& inliers, PlanePatchPtr& planePatch, const PointCloudPtr& pointCloud,
			const float& sacDist);
	void estimateCylinder(pclPointIndices& inliers, CylinderPatchPtr& cylinder, unsigned int& numPointsBefRansac, const NodePointers& octreeNodes,
			const OcTree& octree, const PointCloudPtr& pointCloud, const float& sacDist,
			const float& normalDistanceWeight, const float& minRadius, const float& maxRadius);
	void estimateCylinderNNNormals(CylinderPatchPtr& cylinder,
			const PointCloudPtr& pointCloud, const float& sacDist,
			const float& normalDistanceWeight, const float& minRadius, 
            const float& maxRadius, float searchRadius, const Vec3& initialAxis);
	void estimateCylinderFromNodes(NodeIndices& inliers, CylinderPatchPtr& cylinder, const NodePointers& octreeNodes,
			const OcTree& octree, const PointCloudPtr& pointCloud, const float& sacDist, const float& normalDistanceWeight,
			const float& minRadius, const float& maxRadius);
	void generateSingleSphereOctreeHTBins(SphereUniformSamplings& houghBins, SphereBinsVec& pointNeighborBins,
			const NodePointers& octreeNodes);
	void estimatePlaneSingleSphereOctreeHT(pclPointIndices& inliers, Plane3DPtr& planePatch,
			const SphereUniformSamplings& houghBins, const SphereBinsVec& pointNeighborBins, const NodePointers& octreeNodes,
			NodeIndices& inlierNodes, const PointCloud& pointCloud, const OcTree& octree, const unsigned int& depth,
			const PlanePatchVector& pointMapping);
	void updateHTBins(const NodeIndices& inliers, SphereUniformSamplings& houghBins, SphereBinsVec& pointNeighborBins);

	/**\brief Get all pairs of nodes that are close to each other.
	 * \param nodePairs Output parameter that holds close pairs of nodes.
	 * \param neighborhoodSize Input parameter that determines max neighborhoods distance from node in node sizes.
	 * \param octreeNodes Input parameter but nodes will be marked to avoid double entries.
	 * \param currentOctreeDepth Octree depth where octreeNodes come from.
	 * \param octree Octree to search pairs in.
	 */
	void generateNeighboringNodePairs(NodePairs& nodePairs, NodePointers& octreeNodes, const unsigned int neighborhoodSize,
			const unsigned int& currentOctreeDepth, const OcTree& octree);

	/**\brief Generate Hough space for cylinder orientation.
	 * \param cylinderOrientationBins Output Hough bins.
	 * \param binNeighborhood Output mapping from nodePairs to corresponding Hough bins (for Hough bin update).
	 * \param nodePairs Input pairs of neighboring nodes.
	 */
	void generateCylinderOrientationHB(SphereUniformSampling& cylinderOrientationBins, SphereBinsVec& binNeighborhood,
			const NodePairs& nodePairs);

	/**\brief Estimate cylinder params, starting from cylinder orientation Hough space.
	 * \param cylinderOrientationBins Input Hough space of cylinder oreintations.
	 * \param nodePairs Input pairs of nodes where Hough space indices point to.
	 * \param inlierIndices Output inlier pairs.
	 * \param cylinder Output cylinder patch with inlier estimation (from HT) and cylinder parameters.
	 * \param depth Input octree depth of nodes and node pairs etc.
	 * \param octree Input octree where nodes come from.
	 */
	void estimateCylinderHT(const SphereUniformSampling& cylinderOrientationBins, const NodePairs& nodePairs,
			PairIndices& inlierIndices, CylinderPatchPtr& cylinder, const unsigned int& depth, const OcTree& octree,
			const CylinderPatchVector& pointMapping, const PointCloud& pointCloud, const float& minRadius, const float& maxRadius);

	/**\brief Update Hough space, remove inliers from it
	 * \param cylinderOrientationBins Output Hough space where inlier are removed from.
	 * \param inlierIndices Input indices of inlier to be removed.
	 * \param binNeighborhood Input/Output Neighborhood of indices is stored here, and will be erased (for indices that are removed).
	 */
	void updateCylinderHTBins(SphereUniformSampling& cylinderOrientationBins, const PairIndices& inlierIndices,
			SphereBinsVec& binNeighborhood);

	template<typename StructurePtrT>
	void updatePointMappingAndInliers(std::vector<StructurePtrT>& pointMapping, PointIndices& pointIndices, const StructurePtrT& structurePointer);

	template<typename StructurePtrT>
	void updatePointMapping(std::vector<StructurePtrT>& pointMapping, const PointIndices& pointIndices, const StructurePtrT& structurePointer);

    template<typename StructurePtrT, typename Structures>
    void rebuildPointMapping(std::vector<StructurePtrT>& pointMapping, const Structures& structures);

	/**\brief getPointsFromNodes Get all (non segmented!) points, that contributed to input nodes.
	 * \param pointIndices point's indices in mPointCloud.
	 * \param octreeNodes Input octree nodes. Method starts leaf search here and generates output from those leafs.
	 * Points that are returned here, do not belong to any segment at this moment.
	 * If you also need segmented points, use getAllPointsFromNodes!
	 */
	template<typename PointMappingT>
	void getPointsFromNodes(PointIndices& pointIndices, const NodePointers& octreeNodes,
			const OcTree& octree, const PointCloud& pointCloud, const PointMappingT& pointMapping);
	/**\brief getAllPointsFromNodes Get all (!) points, that contributed to input nodes.
	 * \param pointIndices point's indices in mPointCloud.
	 * \param octreeNodes Input octree nodes. Method starts leaf search here and generates output from those leafs.
	 * No mSegmented check is done here.
	 * If you need only unsegmented points, use getPointsFromNodes!
	 */
	void getAllPointsFromNodes(PointIndices& pointIndices, const NodePointers& octreeNodes, const OcTree& octree,
			const PointCloud& pointCloud);

	/**\brief getAllPointsFromNodes Get all (!) points, that contributed to input nodes.
	 * \param pointIndices point's indices in mPointCloud.
	 * \param nodeIndices Input indices to octree nodes. Only this nodes will be used.
	 * \param octreeNodes Input octree nodes. Method starts leaf search here and generates output from those leafs.
	 * No mSegmented check is done here.
	 * If you need only unsegmented points, use getPointsFromNodes!
	 */
	void getAllPointsFromNodes(PointIndices& pointIndices, const NodeIndices& nodeIndices, const NodePointers& octreeNodes,
			const OcTree& octree, const PointCloud& pointCloud);

	/**\brief addNodesToPlane Add unsegmented points in nodes to plane and it's connected component grid.
	 * \param plane Input and output plane. This is where the points are added to.
	 * \param nodes Input nodes that are added to the plane in this method.
	 */
	void addNodesToPlane(PlanePatchPtr& plane, const NodePointers& nodes, const OcTree& octree,
			const PointCloud& pointCloud, PlanePatchVector& pointMapping);

	/**\brief markNodesAsSegmented Add unsegmented points in nodes to plane and it's connected component grid.
	 * \param pointIndices Input pointIndices. This points are inliers marked as segmented.
	 * \param nodes Input nodes pointers. This nodes are being marked, if almost all points in it are segmented.
	 * \param ratio Input ratio determines, how many points must be marked as segmented to mark the nodes.
	 */
	void markNodesAsSegmented(const PlanePatch& pp, const NodePointers& nodes, const OcTree& octree,
			const PointCloud& pointCloud, const float& maxDistance, const float& ratio);

	/**\brief assignNodesToPlanes Check all nodes in octreeDepth against all
	 * 		planes in mFilteredPlanePatches with respect to node's mean position
	 * 		and it's normal. Generate vector of unassigned nodes.
	 * \param notAssignedOctreeNodes Output unassigned nodes.
	 * \param octreeDepth Input octreeDepth to take nodes from.
	 * This method tries to assign all nodes to the planes that were already found.
	 * If a node can be assign (through normal orientation and mean position) all
	 * it's points are added to the plane.
	 */
	void assignNodesToPlanes(NodePointers& notAssignedOctreeNodes, const unsigned int& octreeDepth,
			const OcTree& octree, PlanePatches& extractedPlanes, const PointCloud& pointCloud, PlanePatchVector& pointMapping);

	/**\brief extractOctreePlanesFromCloud Main segmentation is done here.
	 * 		Most other methods are called from here.
	 * \param extractedPlanes output planes, extracted from octree / pointcloud
	 * \param pointMapping mapping of points to extracted planes
	 * \param octree input octree, filled with points from pointcloud and with normals already calculated
	 * \param pointCloud input point cloud for segmentation
	 */
	void extractOctreePlanesFromCloud(PlanePatches& extractedPlanes, PlanePatchVector& pointMapping,
			const OcTree& octree, const PointCloudPtr& pointCloud);

	/**\brief assignNodePairsToCylinders Check all nodePairs from input if they are candidates for previously found cylinders
	 * \param nodePairs input pairs of nodes, will be updated
	 * \param extractedCylinders previously found cylinders, will be updated
	 */
	void assignNodesToCylinders(NodePointers& octreeNodes, CylinderPatches& extractedCylinders,
			CylinderPatchVector& pointMapping, const OcTree& octree, const PointCloud& pointCloud);

	/**\brief extractOctreePlanesFromCloud Main segmentation is done here.
	 * 		Most other methods are called from here.
	 * \param extractedCylinders output planes, extracted from octree / pointcloud
	 * \param pointMapping mapping of points to extracted cylinders
	 * \param octree input octree, filled with points from pointcloud and with normals already calculated
	 * \param pointCloud input point cloud for segmentation
	 */
	void extractOctreeCylindersFromCloud(CylinderPatches& extractedCylinders, CylinderPatchVector& pointMapping,
			const OcTree& octree, const PointCloudPtr& pointCloud);

	void getNodesFromPairs(NodePointers& nodePointers, const PairIndices& pairIndices, const NodePairs& nodePairs);

	/**\brief spreadNodes assign all nodes to previously determined planes. This is
	 * 			part of the post-processing.
	 * \param depth Should be called with mOctreeDepth.
	 * This method does not check normal orientations but only the node's center-of-gravity against any planes for minimal distance.
	 * This method touches every node and changes any mapping, that was made before.
	 */
	void spreadNodes(const OcTree& octree, PlanePatches& extractedPlanes, PlanePatchVector& pointMapping,
			const PointCloud& pointCloud);
	void spreadPoints();
	void splitIntoConnectedCompononents();
	void mergePlanes(PlanePatches& extractedPlanes, PlanePatchVector& pointMapping, const PointCloud& pointCloud);

	/**\brief planesAreConnected Check if any point in pp1 is a connection to the grid of pp2.
	 * \param pp1 First input planePatchPtr.
	 * \param pp2 Second input planePatchPtr.
	 * \param pointCloud Input PointCloud that inlierIndices refer to.
	 */
	bool planesAreConnected(const PlanePatch& pp1, const PlanePatch& pp2, const PointCloud& pointCloud);
	void addPlanePatchesAndUpdateSegmented(PlanePatches& extractedPlanes, PlanePatchVector& pointMapping,
			const PlanePatches& planePatches);

	template<typename PointMappingT, typename StructureContainerT>
	void writeSegments(const unsigned int& width, const unsigned int& height,
			const std::vector<unsigned int>& undefPoints, const PointMappingT& pointMapping, const std::string& outFilename,
			const StructureContainerT& extractedStructures);

	/**\brief As SACDistanceThreshold depends on current octree cell size, this method determines this threshold for any given (valid octree-)depth.
	 * 		Global parameters may prune this threshold to a given interval.
	 * \param depth Valid octree depth, greater or equal to zero and less or equal to octrees max-depth.
	 * \param octree Input octree.
	 * \return SACDistanceThreshold for given depth and octree.
	 */
	float getSACDistanceFromDepth(const unsigned int& depth, const OcTree& octree);

	/**\brief As HTDistanceThreshold depends on current octree cell size, this method determines this threshold for any given (valid octree-)depth.
	 * 		Global parameters may prune this threshold to a given interval.
	 * \param depth Valid octree depth, greater or equal to zero and less or equal to octrees max-depth.
	 * \param octree Input octree.
	 * \return THDistanceThreshold for given depth and octree.
	 */
	float getHTDistanceFromDepth(const unsigned int& depth, const OcTree& octree);

	void getMinMaxRadiusFromDepth(float& minRadius, float& maxRadius, const OcTree& octree, const float& depth);

	/**\brief As AngleEps(ilon) depends on current octree cell size, this method determines this threshold for any given (valid octree-)depth.
	 * 		Global parameters may prune this threshold to a given interval.
	 * \param depth Valid octree depth, greater or equal to zero and less or equal to octrees max-depth.
	 * \param octree Input octree.
	 * \return AngleEps(ilon) for given depth and octree.
	 */
	float getAngleEpsFromDepth(const unsigned int& depth, const OcTree& octree);

	/**\brief For every node in \param octreeNodes, every plane is being checked
	 * 			if the node is in the planes distanceThreshold
	 * \param outIndicesMap Maps PlanePatchPtr to all nodes(indices), that could possibly belong to its plane.
	 * \param outPointersMap Maps PlanePatchPtr to all nodes(pointers), that could possibly belong to its plane.
	 * \param octreeNodes Input octree nodes to check against every plane in mFilteredPlanePatches.
	 */
	void determineCandidatePlanes(PatchToNodeIndicesMap& outIndicesMap, PatchToNodePointersMap& outPointersMap,
			const NodePointers& octreeNodes, PlanePatches& extractedPlanes);
	void determineCandidatePlanes(NodeIndexToPlanePatchPointers& nodeToPlaneCandidates, const NodePointers& octreeNodes,
			PlanePatches& extractedPlanes);

	/**\brief For every plane in \param nodePointersMap each node is put in it's
	 * 			grid and the main connected component is extracted to output.
	 * \param nodeToPlaneCandidates For each node, each possible plane is stored here.
	 * \param nodeIndicesMap Input Map that maps PlanePatchPtr to candidate nodes,
	 * 			without knowing anything about connected components yet.
	 * \param nodePointersMap Input Map that maps PlanePatchPtr to candidate nodes,
	 * 			without knowing anything about connected components yet.
	 * \param octreeNodes Input octree nodes to map indices to nodePointers.
	 */
	void refineAssignmentWithCCs(NodeIndexToPlanePatchPointers& nodeToPlaneCandidates, const PatchToNodeIndicesMap& nodeIndicesMap,
			const PatchToNodePointersMap& nodePointersMap, const NodePointers& octreeNodes, PlanePatches& extractedPlanes);

	/**\brief Calculate a plane, that parts two planes in the middle through its intersection line.
	 * \param outParams Params of middle plane will be put here.
	 * \param firstPPP PlanePatchPtr to first plane (best hit).
	 * \param secondPPP PlanePatchPtr to second plane (second best hit).
	 */
	void calculateMiddlePlane(Plane3D& outParams, const PlanePatch& firstPPP, const PlanePatch& secondPPP);

	/**\brief Refine edges between every 2 planes by looking at every node and decide
	 * 			which side of an edge it belongs to.
	 * \param finalPlaneNodeAssignmentMap Final plane to node assignment is put here.
	 * \param nodeToPlaneCandidates For each node here must be stored which planes
	 * 			it could possibly belong (after connection check).
	 * \param octreeNodes Input octree nodes to map indices to nodePointers.
	 */
	void refineEdges(PatchToNodePointersMap& finalPlaneNodeAssignmentMap, PatchToPointIndicesMap& finalPlanePointAssignment,
			const NodeIndexToPlanePatchPointers& nodeToPlaneCandidates, const NodePointers& octreeNodes, const OcTree& octree,
			const PointCloud& pointCloud);

    bool checkCylinder(const CylinderPatchPtr& cylinder, const PointCloudPtr& pointCloud) const;
    bool checkCylinderDimensions(const CylinderPatchPtr& cylinder) const;

	ros::NodeHandle* mNodeHandle;
	std::string mKinectTopic;
	ros::Subscriber mPointCloud2Subscriber;
	ros::Subscriber mPointCloudSubscriber;

	PointCloudPtr mPointCloud;
	PlanePatchVector mPointMapping;
	QMutex mPointCloudMutex;
	PlanePatches mPlanePatches;
	QMutex mPlanePatchesMutex;
	CylinderPatches mCylinderPatches;
	QMutex mCylinderPatchesMutex;

    typedef boost::shared_ptr<OcTree> OcTreePtr;
	OcTreePtr mOcTree;

	RosVisualization* mVis;

	SphereBinsVec mPointNeighborBins;

	StrColParams mParams;
	float mLastCellSizeWithNormals;
};

/*****************************************************************************/

inline StructureColoring::NodePointers StructureColoring::getNodesFromIndices(const NodeIndices& nodeIndices, const NodePointers& nodes){
	NodePointers outNodes;
	for(NodeIndices::const_iterator nit = nodeIndices.begin(); nit != nodeIndices.end(); ++nit)
		outNodes.push_back(nodes[*nit]);
	return outNodes;
}

/*****************************************************************************/

inline StructureColoring::Points StructureColoring::getPointsFromIndices(const PointIndices& pointIndices, const PointCloud& cloud){
	Points outPoints;
	for(PointIndices::const_iterator pit = pointIndices.begin(); pit != pointIndices.end(); ++pit)
		outPoints.push_back(cloud.points[*pit].getVector3fMap());
	return outPoints;
}

/*****************************************************************************/

inline void StructureColoring::markAsSegmented(NodePtr currentNode, NodePtr nextNode, void* data){
	currentNode->value_.segmented = true;
	(void)nextNode;
	(void)data;
}

/*****************************************************************************/

template<typename PointMappingT>
void StructureColoring::getPointsFromNodes(PointIndices& pointIndices, const NodePointers& octreeNodes,
		const OcTree& octree, const PointCloud& pointCloud, const PointMappingT& pointMapping) {
	pointIndices.clear();
	pointIndices.reserve(pointCloud.points.size());
	for (NodePointers::const_iterator node_it = octreeNodes.begin(); node_it != octreeNodes.end(); node_it++) {
		NodePointers leaf_nodes;
		octree.getAllLeafs(leaf_nodes, (*node_it));
		for (NodePointers::const_iterator leaf_it = leaf_nodes.begin(); leaf_it != leaf_nodes.end(); leaf_it++) {
			const PointIndices& points = octree.getPointsFromNodePtr(*leaf_it);
			for (std::vector<int>::const_iterator point_it = points.begin(); point_it != points.end(); point_it++) {
				if (!pointMapping[*point_it])
					pointIndices.push_back(*point_it);
			}
		}
	}
}

/*****************************************************************************/

template<typename PointMappingT>
void StructureColoring::initSegmentation(PointMappingT& pointMapping, const PointCloud& pointCloud) {
	pointMapping.resize(pointCloud.points.size());
}

/*****************************************************************************/

template<typename PointMappingT, typename StructureContainerT>
void StructureColoring::writeSegments(const unsigned int& width, const unsigned int& height,
		const std::vector<unsigned int>& undefPoints, const PointMappingT& pointMapping, const std::string& outFilename,
		const StructureContainerT& extractedStructures) {
	size_t id = 10; // plane ids start at 10
	for (typename StructureContainerT::const_iterator it = extractedStructures.begin(); it != extractedStructures.end(); ++it) {
		if ((*it)->getInliers().empty())
			ROS_ERROR("writeSegments() empty structure!");
		else {
			(*it)->setId(id++);
		}
	}
	std::vector<unsigned char> outData;
	outData.resize(width * height, 0);
	unsigned int up_counter = 0;
	for (unsigned int j = 0; j < height; j++) {
		for (unsigned int i = 0; i < width; i++) {
			unsigned int pos = j * width + i;
			unsigned int outDataPos = pos + up_counter;
			while ((up_counter < undefPoints.size()) && (outDataPos == undefPoints[up_counter])) {
				up_counter++;
				outDataPos++;
			}
			if ((pos < pointMapping.size()) && (pointMapping[pos])) {
				outData[outDataPos] = pointMapping[pos]->getId();
			}
		}
	}
	if (mParams.mPGM)
		writePGM(outFilename, width, height, outData);
	else
		writeRasterfile(outFilename, width, height, outData);
}

/*****************************************************************************/

template<typename StructurePtrT>
void StructureColoring::updatePointMappingAndInliers(std::vector<StructurePtrT>& pointMapping, PointIndices& pointIndices, const StructurePtrT& structurePointer){
	if (!structurePointer)return;
	PointIndices indicesToAdd;
	indicesToAdd.reserve(pointIndices.size());
	for(PointIndices::const_iterator pit = pointIndices.begin(); pit != pointIndices.end(); ++pit){
		assert((size_t)*pit < pointMapping.size());
		if (pointMapping[*pit] != structurePointer){
			pointMapping[*pit] = structurePointer;
			indicesToAdd.push_back(*pit);
		}
	}
	pointIndices.swap(indicesToAdd);
}

/*****************************************************************************/

template<typename StructurePtrT>
void StructureColoring::updatePointMapping(std::vector<StructurePtrT>& pointMapping, const PointIndices& pointIndices, const StructurePtrT& structurePointer){
	if (!structurePointer) return;
// assuming pointIndices does not contain duplicate entries, parallising is safe:
#pragma omp parallel for schedule(dynamic,1) 
	for(size_t i = 0; i < pointIndices.size(); ++i){
		assert((size_t)pointIndices[i] < pointMapping.size());
		pointMapping[pointIndices[i]] = structurePointer;
	}
}

/*****************************************************************************/

template<typename StructurePtrT, typename Structures>
void StructureColoring::rebuildPointMapping(std::vector<StructurePtrT>& pointMapping, const Structures& structures)
{
    for (size_t i = 0; i < pointMapping.size(); ++i)
        pointMapping[i].reset();
    for (typename Structures::const_iterator it = structures.begin();
            it != structures.end(); ++it)
    {
        updatePointMapping(pointMapping, (*it)->getInliers(), *it);
    }
}


#endif /*_StructureColoring_h_*/
