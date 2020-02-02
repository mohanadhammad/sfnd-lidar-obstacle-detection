// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for (size_t i : inliers->indices) {
        planeCloud->points.push_back(cloud->points.at(i));
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::tuple<int, int, int> ProcessPointClouds<PointT>::selectRandom3PtsIndices(const typename pcl::PointCloud<PointT>::Ptr cloud)
{
	int first, second, third;
	
	// select first random point
	first = rand() % cloud->points.size();

	for (size_t i = 0; i < cloud->points.size(); i++) {		
		// select second random point
		second = rand() % cloud->points.size();

		// ensure that they are not the same line
		if (second != first) {
			break;
		}
	}

	for (size_t i = 0; i < cloud->points.size(); i++) {		
		// select second random point
		third = rand() % cloud->points.size();

		// ensure that they are not the same line
		if (third != first && third != second) {
			break;
		}
	}

	return std::make_tuple(first, second, third);
}

template<typename PointT>
std::tuple<float, float, float, float> ProcessPointClouds<PointT>::fitPlane(const typename pcl::PointCloud<PointT>::Ptr cloud, int index1, int index2, int index3)
{
	const float x1{ cloud->points[index1].x };
	const float y1{ cloud->points[index1].y };
	const float z1{ cloud->points[index1].z };

	const float x2{ cloud->points[index2].x };
	const float y2{ cloud->points[index2].y };
	const float z2{ cloud->points[index2].z };

	const float x3{ cloud->points[index3].x };
	const float y3{ cloud->points[index3].y };
	const float z3{ cloud->points[index3].z };

	const float i = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
	const float j = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
	const float k = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);

	return std::make_tuple(i, j, k, -(i*x1 + j*y1 + k*z1));
}

template<typename PointT>
float ProcessPointClouds<PointT>::calcPerpDistToPlane(const PointT &pt, const float a, const float b, const float c, const float d)
{
	const float nemo = (a * pt.x) + (b * pt.y) + (c*pt.z) + d;
	const float deno = std::sqrt(a*a + b*b + c*c);
	return (fabs(nemo) / (deno + std::numeric_limits<float>::epsilon()));
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::ransacPlane(const typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
	for (size_t i = 0; i < maxIterations; ++i) {
		
		// Randomly sample subset and fit line
		int index1, index2, index3;
		std::tie(index1, index2, index3) = selectRandom3PtsIndices(cloud);

		float a, b, c, d;
		std::tie(a, b, c, d) = fitPlane(cloud, index1, index2, index3);

		std::unordered_set<int> tmpInliersResult;
		tmpInliersResult.insert(index1);
		tmpInliersResult.insert(index2);
		tmpInliersResult.insert(index3);

		// Measure distance between every point and fitted line
		for (size_t j = 0; j < cloud->points.size(); ++j)
		{
			if (tmpInliersResult.count(j) == 0)
			{
				const float d = calcPerpDistToPlane(cloud->points[j], a, b, c, d);

				// If distance is smaller than threshold count it as inlier
				if (d < distanceTol)
				{
					tmpInliersResult.insert(j);
				}	
			}
		}

		if (tmpInliersResult.size() > inliersResult.size())
		{
			// copy contents if the new iteration has more inliers than the old ones
			inliersResult = tmpInliersResult;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	auto endTime = std::chrono::steady_clock::now();
	std::chrono::duration<double> elapsed_seconds = endTime - startTime;
	std::cout << "RANSAC Plane elapsedtime = " << elapsed_seconds.count() << " sec " << std::endl;

	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.

    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    // pcl::SACSegmentation<PointT> seg;
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SacModel::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);
    // seg.setInputCloud(cloud);
    // seg.segment(*inliers, *coefficients);

    std::unordered_set<int> inliers = ransacPlane(cloud, 100, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index)) {
            cloudInliers->points.push_back(point);
        }
		else {
            cloudOutliers->points.push_back(point);
        }
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
    
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}