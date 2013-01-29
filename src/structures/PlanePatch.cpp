#include <structureColoring/structures/PlanePatch.h>
#include <limits.h>
#include <assert.h>

/*****************************************************************************/

PlanePatch::PlanePatch(const Vec3& normal, const float distance, const float distanceThreshold, const Points& brVertices,
		const float texWidthRatio, const float texHeightRatio, const cv::Mat& heightMap, const cv::Mat& textureMap)
		: mPlane(normal, distance), mBRVertices(brVertices), mTextureMap(cv::Mat(textureMap)), mHeightMap(cv::Mat(heightMap)),
		  mTextureWidthRatio(texWidthRatio), mTextureHeightRatio(texHeightRatio), mDistanceThreshold(distanceThreshold){
	computeNormalMap();
}

/*****************************************************************************/

//PlanePatch::~PlanePatch() {
//}

/*****************************************************************************/

void PlanePatch::getLimitsXY(float& xMin, float& xMax, float& yMin, float& yMax) const {
	xMin = mXMin;
	xMax = mXMax;
	yMin = mYMin;
	yMax = mYMax;
}

/*****************************************************************************/

void PlanePatch::newGrid(const float cellSize) {
	mGrid.reset(new GridMap(mPlane, mXMin, mXMax, mYMin, mYMax, cellSize));
}

/*****************************************************************************/

void PlanePatch::computeTextureSize(float& texPixSize, unsigned int& wPot, unsigned int& hPot, const float texelSizeFactor) {
	float xSpan = fabs(mXMax - mXMin);
	float ySpan = fabs(mYMax - mYMin);
	texPixSize = texelSizeFactor * std::sqrt(xSpan * ySpan / mInlierIndices.size());
	unsigned int width = (xSpan / texPixSize) + 1;
	unsigned int height = (ySpan / texPixSize) + 1;
	compute2PotTexSize(mTextureWidthRatio, mTextureHeightRatio, wPot, hPot, width, height);
}

/*****************************************************************************/

void PlanePatch::compute2PotTexSize(float& widthRatio, float& heightRatio, unsigned int& wPot, unsigned int& hPot,
		const unsigned int width, const unsigned int height){
	wPot = 2;
	hPot = 2;
	while (wPot < width)
		wPot *= 2;
	while (hPot < height)
		hPot *= 2;
	widthRatio = (float) width / (float) wPot;
	heightRatio = (float) height / (float) hPot;
}


/*****************************************************************************/

void PlanePatch::computeRGBTextureMap(const ColoredPointCloud& pointCloud, const float texelSizeFactor,
		const unsigned int dilateIterations) {
	union {
		float rgbfloat;
		uint32_t rgbint;
	} ColorUnion;
	float texPixSize = 0.f;
	unsigned int wPot = 0, hPot = 0;
	computeTextureSize(texPixSize, wPot, hPot, texelSizeFactor);
	mTextureMap = cv::Mat::zeros(hPot, wPot, CV_32FC4);
	cv::Mat texCounter(cv::Mat::zeros(hPot, wPot, CV_8UC1));
	for (PointIndices::const_iterator point_it = mInlierIndices.begin(); point_it != mInlierIndices.end(); ++point_it) {
		ColorUnion.rgbfloat = pointCloud.points[*point_it].rgb;
		int rgb = ColorUnion.rgbint;
		Vec3 p = mPlane.transformToXYPlane(pointCloud.points[*point_it].getVector3fMap());//this does not store the projected points to mPoints
		unsigned int x = fabs(p[0] - mXMin) / texPixSize;
		unsigned int y = fabs(p[1] - mYMin) / texPixSize;
		mTextureMap.at<cv::Vec4f> (y, x)[0] += (float) ((rgb >> 16) & 0xff) / (float) 255;
		mTextureMap.at<cv::Vec4f> (y, x)[1] += (float) ((rgb >> 8) & 0xff) / (float) 255;
		mTextureMap.at<cv::Vec4f> (y, x)[2] += (float) (rgb & 0xff) / (float) 255;
		mTextureMap.at<cv::Vec4f> (y, x)[3] = 1.f;
		texCounter.at<char> (y, x) += 1;
	}
	for (unsigned int hind = 0; hind < hPot; ++hind) {
		for (unsigned int wind = 0; wind < wPot; ++wind) {
			char count = texCounter.at<char> (hind, wind);
			if (count != 0) {
				mTextureMap.at<cv::Vec4f> (hind, wind)[0] /= (float) count;
				mTextureMap.at<cv::Vec4f> (hind, wind)[1] /= (float) count;
				mTextureMap.at<cv::Vec4f> (hind, wind)[2] /= (float) count;
			}
		}
	}
	if (dilateIterations)
		cv::dilate(mTextureMap, mTextureMap, cv::Mat(), cv::Point(-1, -1), dilateIterations);
	//	cv::GaussianBlur(*mTextureMap, *mTextureMap, cv::Size(3,3), 2, 2);
	computeHeightMap(texPixSize, pointCloud, dilateIterations);
	printf("texture and heightmap computed with true size of (%u, %u) = %u pixels", (unsigned int)(wPot * mTextureWidthRatio), (unsigned int)(hPot * mTextureHeightRatio), (unsigned int)((wPot * hPot * mTextureWidthRatio * mTextureHeightRatio) +0.5f));
	cv::Mat img;
	mTextureMap.convertTo(img, CV_8U, 255);
}

/*****************************************************************************/

void PlanePatch::computeIntensityTextureMap(const IntensityPointCloud& pointCloud, const float texelSizeFactor,
		const unsigned int dilateIterations) {
	float texPixSize = 0.f;
	unsigned int wPot = 0, hPot = 0;
	computeTextureSize(texPixSize, wPot, hPot, texelSizeFactor);
	mTextureMap = cv::Mat::zeros(hPot, wPot, CV_32FC2);
	cv::Mat texCounter(cv::Mat::zeros(hPot, wPot, CV_8UC1));
	for (PointIndices::const_iterator point_it = mInlierIndices.begin(); point_it != mInlierIndices.end(); ++point_it) {
		Vec3 p = mPlane.transformToXYPlane(pointCloud.points[*point_it].getVector3fMap());//this does not store the projected points to mPoints
		unsigned int x = fabs(p[0] - mXMin) / texPixSize;
		unsigned int y = fabs(p[1] - mYMin) / texPixSize;
		//TODO normalize intensity to 1 if this is not done before
		mTextureMap.at<cv::Vec2f> (y, x)[0] = pointCloud.points[*point_it].intensity;//assuming that intensity is scaled to ranges 0 .. 1
		mTextureMap.at<cv::Vec2f> (y, x)[1] = 1.f;
		texCounter.at<char> (y, x) += 1;
	}
	for (unsigned int hind = 0; hind < hPot; ++hind) {
		for (unsigned int wind = 0; wind < wPot; ++wind) {
			char count = texCounter.at<char> (hind, wind);
			if (count != 0) {
				mTextureMap.at<cv::Vec2f> (hind, wind)[0] /= (float) count;
			}
		}
	}
	if (dilateIterations)
		cv::dilate(mTextureMap, mTextureMap, cv::Mat(), cv::Point(-1, -1), dilateIterations);
	//	cv::GaussianBlur(*mTextureMap, *mTextureMap, cv::Size(3,3), 2, 2);
	computeHeightMap(texPixSize, pointCloud, dilateIterations);
}

/*****************************************************************************/

void PlanePatch::computeAlphaMap(const PointCloud& pointCloud, const float texelSizeFactor,
		const unsigned int dilateIterations) {
	float texPixSize = 0.f;
	unsigned int wPot = 0, hPot = 0;
	computeTextureSize(texPixSize, wPot, hPot, texelSizeFactor);
	mTextureMap = cv::Mat::zeros(hPot, wPot, CV_8UC1);
	for (PointIndices::const_iterator point_it = mInlierIndices.begin(); point_it != mInlierIndices.end(); ++point_it) {
		Vec3 p = mPlane.transformToXYPlane(pointCloud.points[*point_it].getVector3fMap());
		unsigned int x = fabs(p[0] - mXMin) / texPixSize;
		unsigned int y = fabs(p[1] - mYMin) / texPixSize;
		mTextureMap.at<char> (y, x) = 255;
	}
	if (dilateIterations)
		cv::dilate(mTextureMap, mTextureMap, cv::Mat(), cv::Point(-1, -1), dilateIterations);
	//	cv::GaussianBlur(*mTextureMap, *mTextureMap, cv::Size(3,3), 2, 2);
	computeHeightMap(texPixSize, pointCloud, dilateIterations);
}

/*****************************************************************************/
//http://www.gamedev.net/topic/428776-create-a-normal-map-from-heightmap-in-a-pixel-shader/
/*
float val = tex2D(tx_Heightfield, PixelInput.TextureCoord0);
float valU = tex2D(tx_Heightfield, PixelInput.TextureCoord0 + float2(1 / 512, 0));
float valV = tex2D(tx_Heightfield, PixelInput.TextureCoord0 + float2(0, 1 / 512));
float3 normal = normalize(float3(val - valU, 0.5, val - valV));
*/
void PlanePatch::computeNormalMap() {
	unsigned int height = mHeightMap.rows;
	unsigned int width = mHeightMap.cols;
	mNormalMap = cv::Mat::zeros(height, width, CV_32FC3);
	for (unsigned int hind = 0; hind+1 < height; ++hind) {
		for (unsigned int wind = 0; wind+1 < width; ++wind) {
			float val = mHeightMap.at<float>(hind, wind);
			float valU = mHeightMap.at<float>(hind+1, wind);
			float valV = mHeightMap.at<float>(hind, wind+1);
			Vec3 vec(val - valU, 0.3f, val-valV);
			vec.normalize();
			vec += Vec3(1.f, 1.f, 1.f);
			vec *= 0.5f;
			mNormalMap.at<cv::Vec3f> (hind, wind)[0] = vec.x();
			mNormalMap.at<cv::Vec3f> (hind, wind)[1] = vec.y();
			mNormalMap.at<cv::Vec3f> (hind, wind)[2] = vec.z();
		}
	}
	cv::GaussianBlur(mNormalMap, mNormalMap, cv::Size(3, 3), 2, 2);
}

/*****************************************************************************/

void PlanePatch::computeOrientedBoundingBox() {
	//set mBBVertices (vertices of BoundingBox)
	mBBVertices.clear();
	mBBVertices.push_back(Eigen::Vector3f(mXMin, mYMin, mZMin));
	mBBVertices.push_back(Eigen::Vector3f(mXMin, mYMax, mZMin));
	mBBVertices.push_back(Eigen::Vector3f(mXMax, mYMin, mZMin));
	mBBVertices.push_back(Eigen::Vector3f(mXMax, mYMax, mZMin));
	mBBVertices.push_back(Eigen::Vector3f(mXMax, mYMin, mZMax));
	mBBVertices.push_back(Eigen::Vector3f(mXMax, mYMax, mZMax));
	mBBVertices.push_back(Eigen::Vector3f(mXMin, mYMin, mZMax));
	mBBVertices.push_back(Eigen::Vector3f(mXMin, mYMax, mZMax));
	//transform from plane-coordinate system to "normal" xyz-coordinates
	for (unsigned int i = 0; i < mBBVertices.size(); i++) {
		mBBVertices[i] = mPlane.transformToXYZCoords(mBBVertices[i]);
	}
}

/*****************************************************************************/

void PlanePatch::computeBoundingRectangle() {
	//set mBRVertices (vertices of BoundingRectangle on plane)
	mBRVertices.clear();
	mBRVertices.push_back(Eigen::Vector3f(mXMin, mYMin, 0.f));
	mBRVertices.push_back(Eigen::Vector3f(mXMin, mYMax, 0.f));
	mBRVertices.push_back(Eigen::Vector3f(mXMax, mYMax, 0.f));
	mBRVertices.push_back(Eigen::Vector3f(mXMax, mYMin, 0.f));
	//transform from plane-coordinate system to "normal" xyz-coordinates
	for (unsigned int i = 0; i < mBRVertices.size(); i++) {
		mBRVertices[i] = mPlane.transformToXYZCoords(mBRVertices[i]);
	}
}

/*****************************************************************************/

bool PlanePatch::checkPointConnection(const Vec3& p, const int connectionNeighbors) const {
	return mGrid->checkPointConnection(p, connectionNeighbors);
}

/*****************************************************************************/

float PlanePatch::distanceToOBB(const Vec3& p) const {
	const Eigen::Vector3f localP = mPlane.transformToXYPlane(p);
	float a = 0, b = 0, c = 0;
	if (localP.x() < mXMin)
		a = mXMin - localP.x();
	else if (localP.x() > mXMax)
		a = localP.x() - mXMax;
	if (localP.y() < mYMin)
		b = mYMin - localP.y();
	else if (localP.y() > mYMax)
		b = localP.y() - mYMax;
	if (localP.z() < mZMin)
		c = mZMin - localP.z();
	else if (localP.z() > mZMax)
		c = localP.z() - mZMax;
	return std::sqrt(a * a + b * b + c * c);
}
