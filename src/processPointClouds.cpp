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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        float filterRes,
        Eigen::Vector4f minPoint,
        Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered( new pcl::PointCloud<PointT>() );
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.setInputCloud(cloud);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion( new pcl::PointCloud<PointT>() );
    pcl::CropBox<PointT> roi(true);
    roi.setMin(minPoint);
    roi.setMax(maxPoint);
    roi.setInputCloud(cloudFiltered);
    roi.filter(*cloudRegion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers( new pcl::PointIndices );
    for(int index : indices) {
        inliers->indices.push_back(index);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
        pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
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
std::unordered_set<int> ProcessPointClouds<PointT>::chooseRandPoints(
        const typename pcl::PointCloud<PointT>::Ptr cloud, uint16_t numOfPoints)
{
    std::unordered_set<int> points;
    while (points.size() < numOfPoints)
    {
        points.insert( rand() % cloud->points.size() );
    }

    return points;
}

template<typename PointT>
std::array<float, 4> ProcessPointClouds<PointT>::fitPlane(
        const typename pcl::PointCloud<PointT>::Ptr cloud, std::unordered_set<int>::iterator inliersItr)
{
    std::array<float, 4> coeff;

    const float x1{ cloud->points[*inliersItr].x };
    const float y1{ cloud->points[*inliersItr].y };
    const float z1{ cloud->points[*inliersItr].z };
    *inliersItr++;
    const float x2{ cloud->points[*inliersItr].x };
    const float y2{ cloud->points[*inliersItr].y };
    const float z2{ cloud->points[*inliersItr].z };
    *inliersItr++;
    const float x3{ cloud->points[*inliersItr].x };
    const float y3{ cloud->points[*inliersItr].y };
    const float z3{ cloud->points[*inliersItr].z };

	const float i = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
	const float j = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
	const float k = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);

    coeff[0] = i;
    coeff[1] = j;
    coeff[2] = k;
    coeff[3] = -(i*x1 + j*y1 + k*z1);

    return coeff;
}

template<typename PointT>
float ProcessPointClouds<PointT>::calcPerpDistToPlane(const PointT &pt, const std::array<float, 4> coeff)
{
    const float nemo = (coeff[0] * pt.x) + (coeff[1] * pt.y) + (coeff[2]*pt.z) + coeff[3];
    const float deno = std::sqrt(coeff[0]*coeff[0] + coeff[1]*coeff[1] + coeff[2]*coeff[2]);
    return (fabs(nemo) / deno);
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::ransacPlane(
        const typename pcl::PointCloud<PointT>::Ptr cloud,
        unsigned int maxIterations,
        float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
    while (maxIterations--)
    {
		// Randomly sample subset and fit line
        std::unordered_set<int> inliers = chooseRandPoints(cloud, 3);

        const std::array<float, 4> coeff { fitPlane(cloud, inliers.begin()) };

		// Measure distance between every point and fitted line
		for (size_t j = 0; j < cloud->points.size(); ++j)
		{
            // check if the current point is not included in the selected random 3 points
            if (inliers.count(j) == 0)
			{
                // calculate the perpedicular distance between the current point and the plane surface
                const float dist = calcPerpDistToPlane(cloud->points[j], coeff);

				// If distance is smaller than threshold count it as inlier
                if (dist <= distanceTol)
				{
                    inliers.insert(j);
				}	
			}
		}

        if (inliers.size() > inliersResult.size())
		{
			// copy contents if the new iteration has more inliers than the old ones
            inliersResult = inliers;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	auto endTime = std::chrono::steady_clock::now();
	std::chrono::duration<double> elapsed_seconds = endTime - startTime;
	std::cout << "RANSAC Plane elapsedtime = " << elapsed_seconds.count() << " sec " << std::endl;

	return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(
        typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

//    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//    pcl::SACSegmentation<PointT> seg;
//    seg.setOptimizeCoefficients(true);
//    seg.setModelType(pcl::SacModel::SACMODEL_PLANE);
//    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setMaxIterations(maxIterations);
//    seg.setDistanceThreshold(distanceThreshold);
//    seg.setInputCloud(cloud);
//    seg.segment(*inliers, *coefficients);
//    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    std::unordered_set<int> inliers = ransacPlane(cloud, maxIterations, distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliers.count(index)) {
            cloudInliers->points.push_back(point);
        }
        else {
            cloudOutliers->points.push_back(point);
        }
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
        typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    // loop on clusters
    for (pcl::PointIndices clusterIndex : clusterIndices) {

        // loop on points
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());

        for (int pointIndex : clusterIndex.indices) {
            cluster->points.push_back(cloud->points[pointIndex]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

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
