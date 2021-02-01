#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "GeoGrasp.h"

// callback signature
std::array<double,16> cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    bool flag = true;
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud viewer"));

    // Remove background points
    pcl::PassThrough<pcl::PointXYZ> ptFilter;
    ptFilter.setInputCloud(cloud);
    ptFilter.setFilterFieldName("z");
    ptFilter.setFilterLimits(0.5, 1.5);
    ptFilter.filter(*cloud);

    // plane coef
    pcl::ModelCoefficients planeCoef;
    planeCoef.values.resize (4);
    planeCoef.values[0] = -0.07;
    planeCoef.values[1] = 0.25;
    planeCoef.values[2] = 0.97;
    planeCoef.values[3] = -0.98;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ecExtractor;
    ecExtractor.setClusterTolerance(0.01);
    ecExtractor.setMinClusterSize(100);
    ecExtractor.setMaxClusterSize(100000);
    ecExtractor.setSearchMethod(tree);
    ecExtractor.setInputCloud(cloud);
    ecExtractor.extract(clusterIndices);

    std::array<double,16> result {};
    for (int i=0; i < 16; i++)
    {
        result[i] = 0;
    }

    if (clusterIndices.empty()) {
        // Visualize the result
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 0,255,0);
//        if (flag)
//        {
//            viewer->removeAllPointClouds();
//            viewer->removeAllShapes();
//            viewer->addPointCloud<pcl::PointXYZ>(cloud, rgb, "Main cloud");
//            viewer->spinOnce();
//        }

    }
    else {
        std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin();
        int objectNumber = 0;
//        if (flag)
//        {
//            viewer->removeAllPointClouds();
//            viewer->removeAllShapes();
//        }

        // Every cluster found is considered an object
        // only use the largest cluster
        for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZ>());
            std::cout<<"Cluster Num: "<<it->indices.size()<<std::endl;

            for (std::vector<int>::const_iterator pit = it->indices.begin();
                 pit != it->indices.end(); ++pit)
                objectCloud->points.push_back(cloud->points[*pit]);

            objectCloud->width = objectCloud->points.size();
            objectCloud->height = 1;
            objectCloud->is_dense = true;

            // Create and initialise GeoGrasp
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlaneXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::copyPointCloud(*objectCloud, *objectCloudXYZ);

            GeoGrasp geoGraspPoints;
//            geoGraspPoints.setBackgroundCloud(cloudPlaneXYZ);
            geoGraspPoints.setBackgroundPlaneCoeff(planeCoef);
            geoGraspPoints.setObjectCloud(objectCloudXYZ);
            geoGraspPoints.setGripTipSize(25); // 25mm grip
            geoGraspPoints.setGrasps(1); // Keep track only of the best
            // Calculate grasping points
            geoGraspPoints.compute();

            // Extract best pair of points
            GraspContacts bestGrasp = geoGraspPoints.getBestGrasp();
            GraspPose bestPose = geoGraspPoints.getBestGraspPose();

//            std::cout<<bestPose.midPointPose.translation()<<std::endl;
//            std::cout<<bestPose.midPointPose.linear()<<std::endl;

            Eigen::Affine3f xxx = bestPose.midPointPose;
            Eigen::Matrix4f m;
            m = xxx.matrix();
            std::cout<<m<<std::endl;

            result[0] = m(0,0);
            result[1] = m(0,1);
            result[2] = m(0,2);
            result[3] = m(0,3);
            result[4] = m(1,0);
            result[5] = m(1,1);
            result[6] = m(1,2);
            result[7] = m(1,3);
            result[8] = m(2,0);
            result[9] = m(2,1);
            result[10] = m(2,2);
            result[11] = m(2,3);
            result[12] = m(3,0);
            result[13] = m(3,1);
            result[14] = m(3,2);
            result[15] = m(3,3);

            // Visualize the result
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 0,255,0);


            std::string objectLabel = "";
            std::ostringstream converter;

            converter << objectNumber;
            objectLabel += converter.str();
            objectLabel += "-";

            pcl::ModelCoefficients axeX;
            axeX.values.resize (6);    // We need 6 values
            axeX.values[0] = bestPose.midPointPose.translation()[0];
            axeX.values[1] = bestPose.midPointPose.translation()[1];
            axeX.values[2] = bestPose.midPointPose.translation()[2];
            axeX.values[3] = bestPose.midPointPose.linear()(0, 0);
            axeX.values[4] = bestPose.midPointPose.linear()(1, 0);
            axeX.values[5] = bestPose.midPointPose.linear()(2, 0);

            pcl::ModelCoefficients axeY;
            axeY.values.resize (6);    // We need 6 values
            axeY.values[0] = bestPose.midPointPose.translation()[0];
            axeY.values[1] = bestPose.midPointPose.translation()[1];
            axeY.values[2] = bestPose.midPointPose.translation()[2];
            axeY.values[3] = bestPose.midPointPose.linear()(0, 1);
            axeY.values[4] = bestPose.midPointPose.linear()(1, 1);
            axeY.values[5] = bestPose.midPointPose.linear()(2, 1);

            pcl::ModelCoefficients axeZ;
            axeZ.values.resize (6);    // We need 6 values
            axeZ.values[0] = bestPose.midPointPose.translation()[0];
            axeZ.values[1] = bestPose.midPointPose.translation()[1];
            axeZ.values[2] = bestPose.midPointPose.translation()[2];
            axeZ.values[3] = bestPose.midPointPose.linear()(0, 2);
            axeZ.values[4] = bestPose.midPointPose.linear()(1, 2);
            axeZ.values[5] = bestPose.midPointPose.linear()(2, 2);


//            if (flag)
//            {
//                viewer->addPointCloud<pcl::PointXYZ>(objectCloud, rgb, objectLabel + "Object");
//
//                viewer->addSphere(bestGrasp.firstPoint, 0.01, 0, 0, 255, objectLabel + "First best grasp point");
//                viewer->addSphere(bestGrasp.secondPoint, 0.01, 255, 0, 0, objectLabel + "Second best grasp point");
//                viewer->addLine(axeX, objectLabel + "Pose axeX");
//                viewer->addLine(axeY, objectLabel + "Pose axeY");
//                viewer->addLine(axeZ, objectLabel + "Pose axeZ");
//            }

            objectNumber++;
            break;
        }
//        if (flag){
//            while (!viewer->wasStopped())
//                viewer->spinOnce(100);
//        }
    }
    return result;

}

std::array<double,16> PreGrasp(pybind11::array_t<double>& input1) {
    pybind11::buffer_info buf1 = input1.request();
    double *ptr1 = (double *) buf1.ptr;  //指针访问读写 numpy.ndarray

    if (buf1.ndim != 2) {
        throw std::runtime_error("numpy.ndarray dims must be 2!");
    }
    auto h = buf1.shape[0];
    auto w = buf1.shape[1];
    // define point cloud type XYZ
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    // new point cloud
    PointCloud::Ptr pointCloud(new PointCloud);

    for (int i = 0; i < buf1.shape[0]; i++) {
        PointT p;
        p.x = ptr1[i * buf1.shape[1] + 0];
        p.y = ptr1[i * buf1.shape[1] + 1];
        p.z = ptr1[i * buf1.shape[1] + 2];

        pointCloud->points.push_back(p);

    }
    pointCloud->is_dense = false;
    cout << "number of points:" << pointCloud->size() << endl;

//    cloudCallback(pointCloud);
    std::array<double,16> graspPose = cloudCallback(pointCloud);
    return graspPose;

}

PYBIND11_MODULE(GeoGrasp, m) {
//    pybind11::class_<GeoGrasp>(m, "GeoGrasp")
    m.doc() = "get pcd from python!";
    m.def("run", &PreGrasp);

}


//int main(int argc, char **argv) {
//// load point cloud
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::io::loadPCDFile("../data/objects-example.pcd", *cloud);
//    std::array<double,16> xxx= cloudCallback(cloud);
////  std::cout<<xxx[0]<<std::endl;
//
// return 0;
//}
