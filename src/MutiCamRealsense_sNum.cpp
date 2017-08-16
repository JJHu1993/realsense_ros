
#include <librealsense/rs.hpp>
#include <cstdio>
#include <geometry_msgs/Pose.h>
#include "ros/ros.h"
#include "rosbag/bag.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include "turtlesim/Pose.h"
#include "rosgraph_msgs/Log.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <realsense_ros/camera_intrin.h>

#include <boost/thread/thread.hpp>


std::vector<rs::extrinsics> z_extrinsic;
void publishTF(const int dev_num, const std::string dev_sNum)
{
  tf::Transform tr;
  tf::Quaternion q;
  tf::TransformBroadcaster tf_broadcaster;
  ros::Duration sleeper(0.1); // 100ms
  
  
  
  while (ros::ok()){
    
    ros::Time time_stamp = ros::Time::now()+sleeper;
    // transform base frame to depth frame
    tr.setOrigin(tf::Vector3(z_extrinsic[dev_num].translation[2], -z_extrinsic[dev_num].translation[0], -z_extrinsic[dev_num].translation[1]));
    tr.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, "camera_link_" + dev_sNum, "camera_depth_frame_" + dev_sNum));
    
    // transform depth frame to depth optical frame
    tr.setOrigin(tf::Vector3(0,0,0));
    q.setEuler( M_PI/2, 0.0, -M_PI/2 );
    tr.setRotation( q );
    tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, "camera_depth_frame_" + dev_sNum, "camera_depth_optical_frame_" + dev_sNum));
    
    // transform base frame to color frame (these are the same)
    tr.setOrigin(tf::Vector3(0,0,0));
    tr.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, "camera_link_" + dev_sNum, "camera_rgb_frame_" + dev_sNum));
    
    // transform color frame to color optical frame
    tr.setOrigin(tf::Vector3(0,0,0));
    q.setEuler( M_PI/2, 0.0, -M_PI/2 );
    tr.setRotation( q );
    tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, "camera_rgb_frame_" + dev_sNum, "camera_rgb_optical_frame_" + dev_sNum));
    
    sleeper.sleep();
  }
}

int main(int argc,char** argv) try
  {
    ros::init(argc, argv, "realsense_ros");
    ros::NodeHandle node_handle;  
    image_transport::ImageTransport it (node_handle);
    std::vector<image_transport::Publisher> pubRgb;
	std::vector<image_transport::Publisher> pubIr;
    std::vector<image_transport::Publisher> pubDepth;
    std::vector<image_transport::Publisher> pubRegisteredDepth;
    std::vector<ros::Publisher> realsense_pointWiColor;
    std::vector<ros::Publisher> camera_info_rgb_publisher;
    std::vector<ros::Publisher> camera_info_depth_publisher;
    ros::Rate loop_rate(30);

    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    ////rs::log_to_console(rs::log_severity::warn);
    ////rs::log_to_file(rs::log_severity::debug, "librealsense.log");
    
    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    int connected_device_count = ctx.get_device_count();
    ROS_INFO("There are %d connected RealSense devices.\n", connected_device_count);
    if(connected_device_count == 0) 
    {
     ROS_INFO("Please insert supported devices\n");
     return EXIT_FAILURE;
    }
    
    if(argc==1)
	{	
		for(int i=0;i<connected_device_count;i++)
		{
	    rs::device * tmp = ctx.get_device(i);
	    ROS_INFO("    device %d, an %s\n",i, tmp->get_name());
	    ROS_INFO("    Serial number: %s\n", tmp->get_serial());
	    ROS_INFO("    Firmware version: %s\n", tmp->get_firmware_version());
		}
		ROS_ERROR("\n  ERROR! Plese input the Serial Number as a argument of the device you want to open!\n");
        ROS_ERROR("\n  Command format: roslaunch realsense_ros realsense_ros.launch sNum1:=xxx\n");

		return EXIT_FAILURE;
	}
    
    // extend to multiple devices
    std::vector<rs::device *> dev;
    int use_device_count=0;
    for(int i=0;i<connected_device_count;i++)
      {
	rs::device * tmp = ctx.get_device(i);
	for(int j= 1;j<argc;j++)
	  {
	    if(strcmp(tmp->get_serial() , argv[j]) == 0)
	      {
		dev.push_back(tmp);
		ROS_INFO("    successful to open device %d, an %s\n",use_device_count, dev[use_device_count]->get_name());
		ROS_INFO("    Serial number: %s\n", dev[use_device_count]->get_serial());
		ROS_INFO("    Firmware version: %s\n", dev[use_device_count]->get_firmware_version());
		realsense_pointWiColor.push_back( node_handle.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points_"+
											boost::lexical_cast<std::string>(dev[use_device_count]->get_name()).substr(16,16)+
											"_"+
											boost::lexical_cast<std::string>(dev[use_device_count]->get_serial()), 1000));
		pubRgb.push_back(it.advertise("camera/image/rgb_"+
											boost::lexical_cast<std::string>(dev[use_device_count]->get_serial()), 1000));
		pubIr.push_back(it.advertise("camera/image/ir_raw_"+
											boost::lexical_cast<std::string>(dev[use_device_count]->get_serial()), 1000));
        pubDepth.push_back(it.advertise("camera/image/depth_"+
											boost::lexical_cast<std::string>(dev[use_device_count]->get_serial()), 1000));
        pubRegisteredDepth.push_back(it.advertise("camera/image/registered_depth__"+
											boost::lexical_cast<std::string>(dev[use_device_count]->get_serial()), 1000));
        camera_info_rgb_publisher.push_back(node_handle.advertise<realsense_ros::camera_intrin>("camera/camera_info/rgb_"+
											boost::lexical_cast<std::string>(dev[use_device_count]->get_serial()), 1000));
        camera_info_depth_publisher.push_back(node_handle.advertise<realsense_ros::camera_intrin>("camera/camera_info/depth_"+
											boost::lexical_cast<std::string>(dev[use_device_count]->get_serial()), 1000));
		dev[use_device_count]->enable_stream(rs::stream::depth, rs::preset::best_quality);
		dev[use_device_count]->enable_stream(rs::stream::color, rs::preset::best_quality);
		dev[use_device_count]->enable_stream(rs::stream::infrared, 640, 480, rs::format::y8, 60);

        dev[use_device_count]->start();
		use_device_count++;
		continue;
	      }
	  }
	
      }
    
    if(use_device_count == 0)
	{
		ROS_ERROR("\n  ERROR! The Serial Number you wrote is fake  !\n");
		return EXIT_FAILURE;
	}

//     ROS_INFO("    Using %d camera device(s)! Check serial number(s) provided if this is wrong.", use_device_count);

    //create threads for each device to publish tf, with sNum appended to the link names
    for(int i = 0; i < use_device_count; i++)
	{
      rs::extrinsics tmp = dev[i]->get_extrinsics(rs::stream::depth, rs::stream::color);
      z_extrinsic.push_back(tmp);
      boost::thread transform_thread_(publishTF, i, dev[i]->get_serial());
    }
    while(ros::ok())
	{	
		for(int i=0;i<use_device_count;i++)
		{
			dev[i]->wait_for_frames();
			ros::Time stamp_ = ros::Time::now();
			// Retrieve our images
			const uint16_t * depth_image = (const uint16_t *)dev[i]->get_frame_data(rs::stream::depth);
            const uint16_t * registered_depth_image = (const uint16_t *)dev[i]->get_frame_data(rs::stream::depth_aligned_to_color);
			const uint8_t * color_image = (const uint8_t *)dev[i]->get_frame_data(rs::stream::color);
			const uint8_t * ir_image = (const uint8_t *)dev[i]->get_frame_data(rs::stream::infrared);

			//publish ir image for calibration
			cv::Mat cvImgIr = cv::Mat(480, 640, CV_8UC1, cv::Scalar (0));
			cvImgIr.data = (uchar *)ir_image;
			sensor_msgs::ImagePtr msgIr = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8,cvImgIr).toImageMsg();
			msgIr->header.frame_id = "camera_depth_optical_frame"+ boost::lexical_cast<std::string>(dev[i]->get_serial());
			msgIr->header.stamp = stamp_; // Publish timestamp to synchronize frames.
			msgIr->width = 640;
			msgIr->height = 480;
			msgIr->is_bigendian = false;
			msgIr->step = 640 * sizeof (unsigned char) * 1;
			pubIr[i].publish(msgIr);

			//publish depth image
            cv::Mat cvImgDep = cv::Mat(480, 640, CV_16UC1, cv::Scalar (0));
			cvImgDep.data = (uchar *)depth_image;
			sensor_msgs::ImagePtr msgDep = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1,cvImgDep).toImageMsg();
			msgDep->header.frame_id = "camera_depth_optical_frame"+ boost::lexical_cast<std::string>(dev[i]->get_serial());
			msgDep->header.stamp = stamp_; // Publish timestamp to synchronize frames.
			msgDep->width = 640;
			msgDep->height = 480;
			msgDep->is_bigendian = false;
			msgDep->step = 640 * sizeof (uint16_t) * 1;
			pubDepth[i].publish(msgDep);
            
            //publish registered depth image
            cv::Mat cvImgRegDep = cv::Mat(480, 640, CV_16UC1, cv::Scalar (0));
			cvImgRegDep.data = (uchar *)registered_depth_image;
			sensor_msgs::ImagePtr msgRegDep = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::TYPE_16UC1,cvImgRegDep).toImageMsg();
			msgRegDep->header.frame_id = "camera_color_optical_frame"+ boost::lexical_cast<std::string>(dev[i]->get_serial());
			msgRegDep->header.stamp = stamp_; // Publish timestamp to synchronize frames.
			msgRegDep->width = 640;
			msgRegDep->height = 480;
			msgRegDep->is_bigendian = false;
			msgRegDep->step = 640 * sizeof (uint16_t) * 1;
			pubRegisteredDepth[i].publish(msgRegDep);
            
            
			//publish color image for calibration
			cv::Mat cvImg = cv::Mat(480, 640, CV_8UC3, cv::Scalar (0, 0, 0));
			cvImg.data = (uchar *)color_image;
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8",cvImg).toImageMsg();
			msg->header.frame_id = "camera_rgb_optical_frame_" + boost::lexical_cast<std::string>(dev[i]->get_serial());
			msg->header.stamp = stamp_; // Publish timestamp to synchronize frames.
			msg->width = 640;
			msg->height = 480;
			msg->is_bigendian = false;
			msg->step = 640 * sizeof (unsigned char) * 3;
			pubRgb[i].publish(msg);

			sensor_msgs::PointCloud2 msg_pointcloud;
			msg_pointcloud.width = dev[i]->get_stream_width(rs::stream::depth);
			msg_pointcloud.height = dev[i]->get_stream_height(rs::stream::depth);
			msg_pointcloud.header.stamp = stamp_;
			msg_pointcloud.is_dense = false;
			
			
			sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);
			modifier.setPointCloud2Fields(4, "x", 1,
							sensor_msgs::PointField::FLOAT32, "y", 1,
							sensor_msgs::PointField::FLOAT32, "z", 1,
							sensor_msgs::PointField::FLOAT32, "rgb", 1,
							sensor_msgs::PointField::FLOAT32);
			
			modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
			
			sensor_msgs::PointCloud2Iterator<float>iter_x(msg_pointcloud, "x");
			sensor_msgs::PointCloud2Iterator<float>iter_y(msg_pointcloud, "y");
			sensor_msgs::PointCloud2Iterator<float>iter_z(msg_pointcloud, "z");
			sensor_msgs::PointCloud2Iterator<uint8_t>iter_r(msg_pointcloud, "r");
			sensor_msgs::PointCloud2Iterator<uint8_t>iter_g(msg_pointcloud, "g");
			sensor_msgs::PointCloud2Iterator<uint8_t>iter_b(msg_pointcloud, "b");
			
			// Retrieve camera parameters for mapping between depth and color
			rs::intrinsics depth_intrin = dev[i]->get_stream_intrinsics(rs::stream::depth);
			rs::extrinsics depth_to_color = dev[i]->get_extrinsics(rs::stream::depth, rs::stream::color);
			rs::intrinsics color_intrin = dev[i]->get_stream_intrinsics(rs::stream::color);
            float scale = dev[i]->get_depth_scale();
            
			//publish camera info
            realsense_ros::camera_intrin rgb_intrin;
            rgb_intrin.ppx = color_intrin.ppx;
            rgb_intrin.ppy = color_intrin.ppy;
            rgb_intrin.fx = color_intrin.fx;
            rgb_intrin.fy = color_intrin.fy;
            rgb_intrin.dev_depth_scale = scale;
            for(int i=0;i<5;i++)
                rgb_intrin.coeffs.push_back(color_intrin.coeffs[i]);
            camera_info_rgb_publisher[i].publish(rgb_intrin);
            
            realsense_ros::camera_intrin dep_intrin;
            dep_intrin.ppx = depth_intrin.ppx;
            dep_intrin.ppy = depth_intrin.ppy;
            dep_intrin.fx = depth_intrin.fx;
            dep_intrin.fy = depth_intrin.fy;
            dep_intrin.dev_depth_scale = scale;
            for(int i=0;i<5;i++)
                dep_intrin.coeffs.push_back(depth_intrin.coeffs[i]);
            camera_info_depth_publisher[i].publish(dep_intrin);
            
			//for(int i=0;i<3;i++)
			//std::cout<<color_intrin.width<<"  "<<color_intrin.height;
			//std::cout<<std::endl;;
							

			// Set up a perspective transform in a space that we can rotate by clicking and dragging the mouse
			for(int dy=0; dy<depth_intrin.height; ++dy)
			{
				for(int dx=0; dx<depth_intrin.width; ++dx)
				{
				if(!(dx == 0 && dy == 0))
				{
					++iter_r;
					++iter_g;
					++iter_b;
					++iter_x;
					++iter_y;
					++iter_z;
				}
				// Retrieve the 16-bit depth value and map it into a depth in meters
				uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
				float depth_in_meters = depth_value * scale;
				
				// Skip over pixels with a depth value of zero, which is used to indicate no data
				if(depth_value == 0) continue;
				
				// Map from pixel coordinates in the depth image to pixel coordinates in the color image
				rs::float2 depth_pixel = {(float)dx, (float)dy};
				rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
				rs::float3 color_point = depth_to_color.transform(depth_point);
				rs::float2 color_pixel = color_intrin.project(color_point);
				
				// Use the color from the nearest color pixel, or pure white if this point falls outside the color image
				const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
				if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)
				{
					*iter_r = (uint8_t) 0;
					*iter_g = (uint8_t) 0;
					*iter_b = (uint8_t) 0;
					
				}
				else
				{
					*iter_r = (uint8_t) color_image[(cy * color_intrin.width + cx) * 3];
					*iter_g = (uint8_t) color_image[(cy * color_intrin.width + cx) * 3 + 1];
					*iter_b = (uint8_t) color_image[(cy * color_intrin.width + cx) * 3 + 2];
				}
				
				// Emit a vertex at the 3D location of this depth pixel
				*iter_x = depth_point.x;
				*iter_y = depth_point.y;
				*iter_z = depth_point.z;
				
				}
				
			}
			
			
			msg_pointcloud.header.frame_id = "camera_depth_optical_frame_" + boost::lexical_cast<std::string>(dev[i]->get_serial());
			realsense_pointWiColor[i].publish(msg_pointcloud);		
			}
			
		ros::spinOnce();
		loop_rate.sleep();
	  }
	return EXIT_SUCCESS;
 }
 catch(const rs::error & e)
   {
     // Method calls against librealsense objects may throw exceptions of type rs::error
     ROS_INFO("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
     ROS_INFO("    %s\n", e.what());
     return EXIT_FAILURE;
   }
