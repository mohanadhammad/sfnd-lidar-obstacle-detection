/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <utility>
#include <tuple>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::pair<int, int> selectRandomPtsIndices(const typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	std::pair<int, int> twoRandPtsIndices;
	
	// select first random point
	twoRandPtsIndices.first = rand() % cloud->points.size();

	for (size_t i = 0; i < cloud->points.size(); i++) {		
		// select second random point
		twoRandPtsIndices.second = rand() % cloud->points.size();

		// ensure that they are not the same line
		if (twoRandPtsIndices.second != twoRandPtsIndices.first) {
			break;
		}
	}
	return twoRandPtsIndices;
}

std::tuple<float, float, float> fitLine(const typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::pair<int, int> &indices)
{
	const float x1{ cloud->points[indices.first].x };
	const float y1{ cloud->points[indices.first].y };

	const float x2{ cloud->points[indices.second].x };
	const float y2{ cloud->points[indices.second].y };

	const float a{ y1 - y2 };
	const float b{ x2 - x1 };
	const float c{ (x1 * y2) - (x2 * y1) };

	return std::make_tuple(a, b, c);
}

float calcPerpDistToLine(const pcl::PointXYZ &pt, const float a, const float b, const float c)
{
	const float nemo = (a * pt.x) + (b * pt.y) + c;
	const float deno = std::sqrt(a*a + b*b);
	return abs(nemo / (deno + std::numeric_limits<float>::epsilon()));
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations
	for (size_t i = 0; i < maxIterations; ++i) {
		
		// Randomly sample subset and fit line
		auto indices = selectRandomPtsIndices(cloud);

		float a, b, c;
		std::tie(a, b, c) = fitLine(cloud, indices);

		std::unordered_set<int> tmpInliersResult;
		// Measure distance between every point and fitted line
		for (size_t j = 0; j < cloud->points.size(); ++j)
		{
			const float d = calcPerpDistToLine(cloud->points[j], a, b, c);

			// If distance is smaller than threshold count it as inlier
			if (d < distanceTol)
			{
				tmpInliersResult.insert(j);
			}	
		}

		if (tmpInliersResult.size() > inliersResult.size())
		{
			// swap contents if the new iteration has more inliers than the old ones
			inliersResult.swap(tmpInliersResult);
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
