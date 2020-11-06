#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "Eigen/Dense"
#include "boost/bind.hpp"
#include "opencv2/opencv.hpp"
#include "ros/package.h"
#include "iostream"

#define WORLD 0
#define KNOWN 1
#define UNKNOWN 2

struct tag
{
    int id;
    double size;
    Eigen::MatrixXd wTtag;
};

class Node
{
    public:
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber sub;
        std::vector<tag> known_tags;
        std::vector<tag> unknown_tags; // probably don't need this in real-time
        tag world;   
        Eigen::MatrixXd wTcam;
        bool first_view;
        int world_loc;
        int known_tag_loc; 
        cv::Mat camMatrix;
        cv::Vec<double, 5> distCoeffs;

        Node()
        {
            sub = n.subscribe("sampled_detections", 1000, &Node::camViewCallback,this);

            camMatrix = cv::Mat(3,3,CV_64F,cv::Scalar(0));
            intrinsicLoad();
            //std::cout<< camMatrix << std::endl;
            //std::cout<< distCoeffs << std::endl;

            first_view = true;

            /////
            // tag item;
            // item.id =1;
            // item.size=10;
            // known_tags.push_back(item);
            // item.id =2;
            // item.size=20;
            // known_tags.push_back(item);

        }

        void intrinsicLoad()
        {
          std::vector<double> intrinsic;
          if((n.hasParam("/camera_matrix/data"))&&(n.hasParam("/distortion_coefficients/data")))
          {
            n.getParam("/camera_matrix/data", intrinsic);
            memcpy(camMatrix.data,intrinsic.data(),intrinsic.size()*sizeof(double));
            n.getParam("/distortion_coefficients/data", intrinsic);
            distCoeffs={intrinsic[0],intrinsic[1],intrinsic[2],intrinsic[3],intrinsic[4]};
            ROS_INFO("Camera intrinsic parameters loaded");          
          }
          else 
          {
            ROS_ERROR("Camera intrinsics not loaded to parameter server!");
            ros::shutdown();
          }
        }

        void camViewCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
        {
            if (!msg->detections.empty())
            {
                if (first_view == true)
                {
                    world.id = msg->detections[0].id[0];
                    world.size = msg->detections[0].size[0];
                    first_view = false;
                    ROS_INFO("World tag set with tag ID %i",world.id);
                }

                int detection = detectionType(msg);
                // ROS_INFO("Detection type %i", detection); /////

                if (!known_tags.empty())////
                {
                    for (int i = 0; i != known_tags.size(); i++)
                    std::cout<<" "<<known_tags[i].id;
                    std::cout<<"\n";
                }
                
                if (detection == WORLD)
                {
                    wTcam =  camTtag(msg, world_loc, true);
                    // std::cout << wTcam << std::endl;

                    for (int i = 0; i != msg->detections.size(); i++)
                    {
                        if (i != world_loc)
                        {
                            //check if tag is unknown
                            auto it = std::find_if( known_tags.begin(), known_tags.end(), boost::bind(&tag::id,_1) == msg->detections[i].id[0]);
                            if (it == known_tags.end())
                            {
                                tag temp;
                                temp.id = msg->detections[i].id[0];
                                temp.size = msg->detections[i].size[0];
                                temp.wTtag = wTcam * camTtag(msg, i,false);
                                known_tags.push_back(temp);
                            }
                        }

                    }
                }
                else if (detection == KNOWN)
                {
                    auto it = std::find_if( known_tags.begin(), known_tags.end(), boost::bind(&tag::id,_1) == msg->detections[known_tag_loc].id[0]);
                    wTcam = known_tags[it - known_tags.begin()].wTtag * camTtag(msg, known_tag_loc, true);
                    // std::cout << wTcam << std::endl;

                    for (int i = 0; i != msg->detections.size(); i++)
                    {
                        //check if tag is unknown
                        auto it = std::find_if( known_tags.begin(), known_tags.end(), boost::bind(&tag::id,_1) == msg->detections[i].id[0]);
                        if (it == known_tags.end())
                        {
                            tag temp;
                            temp.id = msg->detections[i].id[0];
                            temp.size = msg->detections[i].size[0];
                            temp.wTtag = wTcam * camTtag(msg, i,false);
                            known_tags.push_back(temp);
                        }
                    }
                }
                else
                {
                   // no reference to map any tags or update camera 
                }
            }

        }

        int detectionType(apriltag_ros::AprilTagDetectionArray::ConstPtr msg)
        {
            int type = UNKNOWN;
            for (int i = 0; i != msg->detections.size(); i++)
            {
                if (msg->detections[i].id[0] == world.id)
                {
                    world_loc = i;
                    return WORLD;
                }
                else
                {
                    auto it = std::find_if( known_tags.begin(), known_tags.end(), boost::bind(&tag::id,_1) == msg->detections[i].id[0]);
                    if (it != known_tags.end())
                    {
                        known_tag_loc = i;
                        type = KNOWN;
                    }
                }
            }
            return type;
        }

        Eigen::MatrixXd camTtag(apriltag_ros::AprilTagDetectionArray::ConstPtr msg, int loc, bool inverse)
        {
            Eigen::MatrixXd cam_T_tag(4, 4);

            std::vector<cv::Point2d> img_pts;
            for(int i = 0; i < 4; i++)
            {    
                img_pts.push_back(cv::Point2d(msg->detections[loc].pixel_corners_x[i], msg->detections[loc].pixel_corners_y[i]));
            }

            std::vector<cv::Point3d> obj_pts;
            double tag_size = msg->detections[loc].size[0]; 
            obj_pts.push_back(cv::Point3d(-(tag_size / 2), -(tag_size / 2), 0));
            obj_pts.push_back(cv::Point3d((tag_size / 2), -(tag_size / 2), 0));
            obj_pts.push_back(cv::Point3d((tag_size / 2), (tag_size / 2), 0));
            obj_pts.push_back(cv::Point3d(-(tag_size / 2), (tag_size / 2), 0));

            cv::Mat camTtag_rvec;
            cv::Mat camTtag_tvec;
            cv::Mat rodrigues_rvec;
            cv::solvePnP(obj_pts, img_pts, camMatrix, distCoeffs, rodrigues_rvec, camTtag_tvec, false, cv::SOLVEPNP_ITERATIVE);
            cv::Rodrigues(rodrigues_rvec, camTtag_rvec); 
            
            cam_T_tag << camTtag_rvec.at<double>(0, 0), camTtag_rvec.at<double>(0, 1), camTtag_rvec.at<double>(0, 2), camTtag_tvec.at<double>(0, 0), 
                         camTtag_rvec.at<double>(1, 0), camTtag_rvec.at<double>(1, 1), camTtag_rvec.at<double>(1, 2), camTtag_tvec.at<double>(0, 1), 
                         camTtag_rvec.at<double>(2, 0), camTtag_rvec.at<double>(2, 1), camTtag_rvec.at<double>(2, 2), camTtag_tvec.at<double>(0, 2), 
                         0, 0, 0, 1;
            
            if (inverse == true)
            {
                return cam_T_tag.inverse().eval();
            }
            return cam_T_tag;
        }

};


int main(int argc , char **argv)
{
    ros::init(argc, argv, "vslam");
    Node vslam;
    ros::spin();

    return 0;
}