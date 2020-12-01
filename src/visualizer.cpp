#include "ros/ros.h"
#include "ros/package.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "robot_live_vslam/Tag.h"
#include "robot_live_vslam/TagMap.h"
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"
#include "string"

class Node
{
    public:
        ros::NodeHandle n;
        ros::Publisher cam_pub;
        ros::Publisher tag_pub; 
        ros::Subscriber sub;
        robot_live_vslam::TagMap::ConstPtr message;

        Node()
        {
            cam_pub = n.advertise<visualization_msgs::Marker>("cam_marker",1000);
            tag_pub = n.advertise<visualization_msgs::MarkerArray>("tag_markers",1000);
            sub = n.subscribe("targets_map", 1000, &Node::visCallback,this);

        }

        void visCallback(const robot_live_vslam::TagMap::ConstPtr& msg)
        {
	        visualization_msgs::MarkerArray tags;
	        visualization_msgs::Marker cam;

            if (!msg->tags.empty())
            {
                cam = loadCamMarker();
		
                for (int i = 0; i < msg->tags.size(); i++)
                {
	                visualization_msgs::Marker tag;
                    tag = loadTagMarker(i);
                    tags.markers.push_back(tag);
                }
            }

            tag_pub.publish(tags);
            cam_pub.publish(cam);

        }

        Eigen::Quaterniond rodrigToQuat(double tempvec[3])
        {
        	cv::Mat rodrigues=cv::Mat(1,3,CV_64F,tempvec);
        	cv::Mat tempmat;
        	cv::Rodrigues(rodrigues,tempmat);
        	Eigen::Matrix3d rot;
        	rot << tempmat.at<double>(0, 0), tempmat.at<double>(0, 1), tempmat.at<double>(0, 2),
        		   tempmat.at<double>(1, 0), tempmat.at<double>(1, 1), tempmat.at<double>(1, 2), 
        	       tempmat.at<double>(2, 0), tempmat.at<double>(2, 1), tempmat.at<double>(2, 2);
        	Eigen::Quaterniond quat(rot);
        	return quat;
        }

        geometry_msgs::Pose loadMarkerPose(int loc)
        {
        	geometry_msgs::Pose pose;

        	pose.position.x = message->tags[loc].wTtag[3];
        	pose.position.y = message->tags[loc].wTtag[4];
        	pose.position.z = message->tags[loc].wTtag[5];

        	double rodrigues[3] = {message->tags[loc].wTtag[0], message->tags[loc].wTtag[1], message->tags[loc].wTtag[2]};

        	Eigen::Quaterniond quat = rodrigToQuat(rodrigues);
        	pose.orientation.x = quat.x();
        	pose.orientation.y = quat.y();
        	pose.orientation.z = quat.z();
        	pose.orientation.w = quat.w();

        	return pose;
        }

        geometry_msgs::Pose loadCamPose()
        {
        	geometry_msgs::Pose pose;

            pose.position.x = message->cam_trans[0];
        	pose.position.y = message->cam_trans[1];
        	pose.position.z = message->cam_trans[2];

            double rodrigues[3] = {message->cam_rot[0], message->cam_rot[1], message->cam_rot[2]};

        	Eigen::Quaterniond quat = rodrigToQuat(rodrigues);
        	pose.orientation.x = quat.x();
        	pose.orientation.y = quat.y();
        	pose.orientation.z = quat.z();
        	pose.orientation.w = quat.w();

        	return pose;
        }

        visualization_msgs::Marker loadTagMarker(int loc)
        {
            visualization_msgs::Marker marker;

        	marker.header.frame_id = "world"; ////
        	marker.header.stamp = ros::Time();////
        	marker.ns = "tag_" + std::to_string(message->tags[loc].id);
        	marker.id = message->tags[loc].id;
        	marker.type = visualization_msgs::Marker::CUBE;////
        	marker.action = visualization_msgs::Marker::ADD;////
        	marker.pose = loadMarkerPose(loc);
        	marker.scale.x = message->tags[loc].size;////
        	marker.scale.y = message->tags[loc].size;////
        	marker.scale.z = 0.0005;////
        	marker.color.a = 1.0;////
        	marker.color.r = 1;////
        	marker.color.g = 1;////
        	marker.color.b = 1;////
        	marker.lifetime = ros::Duration();////

            return marker;
        }

        visualization_msgs::Marker loadCamMarker()
        {
            visualization_msgs::Marker marker;

        	marker.header.frame_id = "world"; ////
        	marker.header.stamp = ros::Time();////
        	marker.ns = "camera"; ////
        	marker.id = -1;////
        	marker.type = visualization_msgs::Marker::CUBE;////
        	marker.action = visualization_msgs::Marker::ADD;////
        	marker.pose = loadCamPose();
        	marker.scale.x = 0.08;////
        	marker.scale.y = 0.08;////
        	marker.scale.z = 0.04;////
        	marker.color.a = 1.0; ////
        	marker.color.r = 1.00;////
        	marker.color.g = 0.41;////
        	marker.color.b = 0.71;////
        	marker.lifetime = ros::Duration();////

            return marker; 
        }

};


int main(int argc , char **argv)
{
    ros::init(argc, argv, "visualizer");
    Node vis;
    ros::spin();

    return 0;
}