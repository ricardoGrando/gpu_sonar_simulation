#include <iostream>
#include <numeric>
#include <string>
#include <QApplication>

// Sonar includes
#include <Sonar.hpp>
#include <Utils.hpp>
#include <widget_collection/sonar_widget/SonarPlot.h>
#include <widget_collection/sonar_widget/SonarWidget.h>
#include <vizkit3d_normal_depth_map/src/NormalDepthMap.hpp>
#include <vizkit3d_normal_depth_map/src/ImageViewerCaptureTool.hpp>
#include <base-types/base/Angle.hpp>

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Openscenegraph includes
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>

#include <std_msgs/Float64MultiArray.h>

// ROS includes
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf_conversions/tf_eigen.h>

using namespace gpu_sonar_simulation;
using namespace vizkit3d_normal_depth_map;
using namespace cv;

#define BEAM_WIDTH 90.0
#define BEAM_HEIGHT 15.0
#define BIN_COUNT 1000//502
#define BEAM_COUNT 256//256//768
#define GAIN 0.5
#define RANGE 20

// /* FLS parameters */
//     std::cout << "Computing FLS performance..." << std::endl;
//     float resolution_constant = 2.56;
//     std::vector<uint> beam_counts = {128,256};
//     std::vector<uint> bin_counts = {250, 500, 750, 1000};
//     std::vector<float> beam_widths = {90};
//     std::vector<float> beam_heights = {15};
//     bool isScanning = false;
//    computePerformance(beam_counts, bin_counts, beam_widths, beam_heights, N, resolution_constant, isScanning);

//     /* MSIS parameters */
//      std::cout << "Computing MSIS performance..." << std::endl;
//      resolution_constant = 1;
//      beam_counts = {1};
//      bin_counts = {250, 500, 750, 1000};
//      beam_widths = {2.0};
//      beam_heights = {40.0};
//      isScanning = true;
//      computePerformance(beam_counts, bin_counts, beam_widths, beam_heights, N, resolution_constant, isScanning);


#define myrand ((float)(random())/(float)(RAND_MAX) )


geometry_msgs::Pose sonar_pose;

void poseCallback(const geometry_msgs::Pose& msg){
    sonar_pose = msg;
}

// create a depth and normal matrixes to test
cv::Mat createRandomImage(int rows, int cols) {
    cv::Mat raw_image = cv::Mat::zeros(rows, cols, CV_32FC3);

    for (int k = 0; k < raw_image.channels() - 1; k++)
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                raw_image.at<Vec3f>(i, j)[k] = myrand;

    return raw_image;
}

void addScene(osg::ref_ptr<osg::Group> root){
    std::string current_path(__FILE__);
    current_path = current_path.substr(0, current_path.find_last_of("/"));
    osg::Node* scene = osgDB::readNodeFile(current_path + "/../uwmodels/yacht/stage2.osgb");
    // osg::Node* terrain = osgDB::readNodeFile(current_path + "/../uwmodels/yacht/terrain.osgb");
    // root->addChild(terrain);
    root->addChild(scene);
}

cv::Mat QImage2Mat(QImage const& src){
     cv::Mat tmp(src.height(),src.width(),CV_8UC3,(uchar*)src.bits(),src.bytesPerLine());
     cv::Mat result;
     cvtColor(tmp, result,CV_BGR2RGB);
     return result;
}

int main(int argc, char* argv[]){
    QApplication a(argc,argv);
    SonarPlot s;
    //SonarWidget s;

    osg::ref_ptr<osg::Image> osg_image;

    /* generate a random shader image */
    uint width = BIN_COUNT;//*5.12;  // 5.12 pixels are needed for each bin;
    base::Angle beam_width = base::Angle::fromDeg(BEAM_WIDTH);
    base::Angle beam_height = base::Angle::fromDeg(BEAM_HEIGHT);
    uint height = width * tan(beam_height.rad * 0.5) / tan(beam_width.rad * 0.5);

    float range = RANGE;
    std::string shaderPathFrag = "/home/ricardo/catkin_ws/src/gpu_sonar_simulation/include/vizkit3d_normal_depth_map/resources/shaders/normalDepthMap.frag";
    std::string shaderPathVert = "/home/ricardo/catkin_ws/src/gpu_sonar_simulation/include/vizkit3d_normal_depth_map/resources/shaders/normalDepthMap.vert";
    NormalDepthMap normal_depth_map(range, beam_width.getDeg(), beam_height.getDeg(), shaderPathFrag, shaderPathVert);
    ImageViewerCaptureTool capture = ImageViewerCaptureTool(beam_height.getRad(), beam_width.getRad(), width, false);
    capture.setBackgroundColor(osg::Vec4d(0.0, 0.0, 0.0, 1.0));

    // add oilrig
    osg::ref_ptr<osg::Group> root = new osg::Group();
    addScene(root);
    normal_depth_map.addNodeChild(root);

    // init ros node and subscribe to pose topic
    ros::init(argc, argv, "gpu_sonar_simulation");
    ros::NodeHandle n;
    // ros::Subscriber pose_sub = n.subscribe("/rexrov/ground_truth_to_tf_rexrov/pose", 10, poseCallback);
    ros::Subscriber pose_sub = n.subscribe("/hydrone_aerial_underwater/ground_truth/pose", 10, poseCallback);

    // node to publish sonar topic
    ros::init(argc, argv, "sonar");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_normal_depth_map = it.advertise("/sonar_sensor/normal_depth_map", 1);
    image_transport::Publisher pub_sonar = it.advertise("/sonar_sensor/sonar_image", 1);

    ros::Publisher pub_distance = n.advertise<std_msgs::Float64MultiArray>("/sonar_distance", 1000);

    ros::Rate loop_rate(10);

    int count = 0;
    while(ros::ok()){
        // set up for camera's pov
        osg::Matrixd cameraMatrix, cameraRotation, cameraTrans;

        cameraRotation.setRotate(osg::Quat(-sonar_pose.orientation.x, -sonar_pose.orientation.y, -sonar_pose.orientation.z, sonar_pose.orientation.w));
        // openscenegraph has a -90 degrees offset heading compared to Gazebo
        cameraRotation = cameraRotation * osg::Matrixd::rotate(osg::DegreesToRadians(90.0), osg::Vec3(0,0,1));// * osg::Matrixd::rotate(osg::DegreesToRadians(-20.0), osg::Vec3(0,1,0));
        cameraTrans.makeTranslate(osg::Vec3(-sonar_pose.position.x, -sonar_pose.position.y, -sonar_pose.position.z));

        cameraMatrix = cameraTrans * cameraRotation;
        capture.setViewMatrix(cameraMatrix*osg::Matrix::rotate(-M_PI/2.0, 1,0,0));
        // capture.setViewMatrix(cameraMatrix*osg::Matrix::rotate(-M_PI/9.0, 0,1,0));

        // grab capture
        osg::ref_ptr<osg::Image> osgImage = capture.grabImage(normal_depth_map.getNormalDepthMapNode());
        cv::Mat cv_image;
        convertOSG2CV(osgImage, cv_image);

        // receives shader image in opencv format
    	  cv::Mat cv_depth;
        osg::ref_ptr<osg::Image> osg_depth = capture.getDepthBuffer();
        gpu_sonar_simulation::convertOSG2CV(osg_depth, cv_depth);

        // replace depth matrix
        std::vector<cv::Mat> channels;
        cv::split(cv_image, channels);
        channels[1] = cv_depth;
        cv::merge(channels, cv_image);

        // image flip in y axis is needed to fix mirrored problem
        cv::flip(cv_image, cv_image, 0);
        //cv::imshow("Normal Depth Map", cv_image);

        // converting cv_image (normal_depth_map) to publish in ros
        cv::Mat image_ndm;
        cv_image.convertTo(image_ndm, CV_8UC3, 255);
        sensor_msgs::ImagePtr msg_ndm = cv_bridge::CvImage(std_msgs::Header(),"bgr8", image_ndm).toImageMsg();

        /* initialize Sonar Simulation */
        uint32_t bin_count = BIN_COUNT;
        uint32_t beam_count = BEAM_COUNT;
        Sonar sonar_sim(bin_count, beam_count, beam_width, beam_height);

        /* simulate sonar image */
        std::vector<float> bins;
        sonar_sim.decodeShader(cv_image, bins);

        /* apply additional gain */
        float gain = GAIN;
        sonar_sim.applyAdditionalGain(bins, gain);

        /* encapsulate in rock's sonar structure */
        //float range = 22.0;
        base::samples::Sonar sonar = sonar_sim.simulateSonar(bins, RANGE);

        int aux = 0;
        float highest = 0.0;
        float threshold = 0.01;
        std::vector<float> distance;
        float dist = (float)RANGE;
        for (int k = 0; k < BEAM_COUNT; k++){
            highest = 0.0;
            dist = (float)RANGE;
            for (int i = k*BIN_COUNT; i < (k+1)*BIN_COUNT; i++){
                // std::cout << sonar.bins[i] << ' ';
                aux = aux + 1;

                if (aux == BIN_COUNT){
                    aux = 0;
                    // std::cout << "\n" << ' ';
                }

                if (sonar.bins[i] > threshold){
                    // highest = sonar.bins[i];
                    // if (sonar.bins[i] > threshold){
                    dist = RANGE*((float)(i+1 - k*BIN_COUNT))/((k+1)*BIN_COUNT - k*BIN_COUNT);
                    // }
                    break;
                }
            }
            distance.insert(distance.end(), dist);
        }

        std_msgs::Float64MultiArray msg_distance;

        // set up dimensions
        msg_distance.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg_distance.layout.dim[0].size = distance.size();
        msg_distance.layout.dim[0].stride = 1;
        msg_distance.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

        // copy in the data
        msg_distance.data.clear();
        msg_distance.data.insert(msg_distance.data.end(), distance.begin(), distance.end());

        // std::cout << "\n\n" << ' ';

        // for (std::vector<float>::const_iterator i = distance.begin(); i != distance.end(); ++i){
        //     std::cout << *i << ' ';
        // }

        // std::cout << "\n\n" << ' ';

        s.show();
        s.setData(sonar);

        //set sonar image to publish in ros
        cv::Mat image_sonar = QImage2Mat(s.getImg());
        sensor_msgs::ImagePtr msg_sonar = cv_bridge::CvImage(std_msgs::Header(),"bgr8", image_sonar).toImageMsg();

        pub_sonar.publish(msg_sonar);
        pub_normal_depth_map.publish(msg_ndm);
        pub_distance.publish(msg_distance);

        a.processEvents();

        loop_rate.sleep();

        ros::spinOnce();

        // break;
    }
}
