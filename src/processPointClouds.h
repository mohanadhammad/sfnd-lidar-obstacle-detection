// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <tuple>
#include <unordered_set>
#include "render/box.h"
#include "kdtree.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

private:
    std::unordered_set<int> chooseRandPoints(const typename pcl::PointCloud<PointT>::Ptr cloud, uint16_t numOfPoints);
    std::array<float, 4> fitPlane(const typename pcl::PointCloud<PointT>::Ptr cloud, std::unordered_set<int>::iterator inliersItr);
    float calcPerpDistToPlane(const PointT &pt, const std::array<float, 4> coeff);
    std::unordered_set<int> ransacPlane(const typename pcl::PointCloud<PointT>::Ptr cloud, unsigned int maxIterations, float distanceTol);
  
    ///
    /// \brief proximity
    /// \param index
    /// \param points
    /// \param tree
    /// \param distanceTol
    /// \param processed
    /// \param cluster
    ///
    void proximity(
        size_t index,
        const std::vector<std::vector<float>>& points,
        KdTree<3> *tree,
        float distanceTol,
        std::vector<bool>& processed,
        std::vector<int> &cluster);

    ///
    /// \brief euclideanCluster
    /// \param points
    /// \param tree
    /// \param distanceTol
    /// \return
    ///
    std::vector<std::vector<int>> euclideanCluster(
            const std::vector<std::vector<float>>& points,
            KdTree<3> *tree,
            float distanceTol);
};
#endif /* PROCESSPOINTCLOUDS_H_ */
