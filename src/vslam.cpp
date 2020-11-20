#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "Eigen/Dense"
#include "boost/bind.hpp"
#include "opencv2/opencv.hpp"
#include "ros/package.h"
#include "iostream"
#include "rviz_simulator/camera_calibration_optimizer.h"

using namespace camera_calibration;

#define WORLD 0
#define KNOWN 1
#define UNKNOWN 2

class Node
{
    public:
        ros::NodeHandle n;
        ros::Subscriber sub;   
        Eigen::MatrixXd wTcam;
        bool first_view;
        int world_loc;
        int known_tag_loc; 
        cv::Mat camMatrix;
        cv::Vec<double, 5> distCoeffs;
        std::vector<int> toBeOpt;
        std::vector<int> known_in_frame;
        std::vector<int> opt_in_frame; 
        int opt_frames;
        bool optimizing; // if optimization requires more time than callback loops
        std::string package_path;

        Node()
        {
            std::string package_path = ros::package::getPath("robot_live_vslam");
            sub = n.subscribe("sampled_detections", 1000, &Node::camViewCallback,this);
            intrinsicLoad();
            //std::cout<< camMatrix << std::endl;
            //std::cout<< distCoeffs << std::endl;

            first_view = true;
            
        }

        void intrinsicLoad()
        {
          camMatrix = cv::Mat(3,3,CV_64F,cv::Scalar(0));
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
                    tag world;
                    world.id = msg->detections[0].id[0];
                    world.avg_id = world.id; 
                    world.size = msg->detections[0].size[0];
                    Eigen::MatrixXd wTworld(4,4);
                    wTworld << 1, 0, 0, 0, 
                               0, 1, 0, 0,
                               0, 0, 1, 0,
                               0, 0, 0, 1;
                    world.wTtag = wTworld;
                    world.wTtag_vec = { 0, 0, 0, 0, 0, 0};
                    known_tags.push_back(world);
                    first_view = false;
                    ROS_INFO("World tag set with tag ID %i",world.id);
                }

                int detection = detectionType(msg);
                // ROS_INFO("Detection type %i", detection); /////

                // if (!known_tags.empty())////
                // {
                //     for (int i = 0; i != known_tags.size(); i++)/////
                //     std::cout<<" "<<known_tags[i].avg_id<<"("<<known_tags[i].pose_count<<")";/////
                //     std::cout<<"\n";/////
                // }
                // known_tags[0].pose_count= 0; ///
                
                if (detection == WORLD)
                {
                    wTcam =  camTtag(msg, world_loc, true);
                    // std::cout << wTcam << std::endl;

                    for (int i = 0; i != msg->detections.size(); i++)
                    {
                        if (i != world_loc)
                        {
                            auto it = std::find_if( known_tags.begin(), known_tags.end(), boost::bind(&tag::avg_id,_1) == msg->detections[i].id[0]);
                            if (it == known_tags.end()) //if tag is unknown
                            {
                                newTag(msg, i);
                            }
                        }

                    }
                }
                else if (detection == KNOWN)
                {
                    auto it = std::find_if( known_tags.begin(), known_tags.end(), boost::bind(&tag::id,_1) == msg->detections[known_tag_loc].id[0]); //use id
                    wTcam = known_tags[it - known_tags.begin()].wTtag * camTtag(msg, known_tag_loc, true);
                    // std::cout << wTcam << std::endl;

                    for (int i = 0; i != msg->detections.size(); i++)
                    {
                        //check if tag is unknown or has insufficient pose_count
                        auto it = std::find_if( known_tags.begin(), known_tags.end(), boost::bind(&tag::avg_id,_1) == msg->detections[i].id[0]);
                        if (it == known_tags.end()) //if tag is known
                        {
                            newTag(msg, i);
                        }
                    }
                }
                else
                {
                   // no reference to map any tags or update camera 
                }
                
                if ((known_in_frame.size() > 1) && (opt_in_frame.size() > 0)) //at least 2 known tags, 1 pending optimization
                {
                    cached_pictures.push_back(msg2Picture(msg));
                    opt_frames ++; //cache picture
                    for (int i = 0; i < opt_in_frame.size(); i++)
                    {
                        if (opt_in_frame[i] != known_tags[0].id) //don't account for world tag optimization
                        {
                            auto it = std::find_if( known_tags.begin(), known_tags.end(), boost::bind(&tag::id,_1) == opt_in_frame[i]);
                            int index = it - known_tags.begin();
                            if (known_in_frame.size() > known_tags[index].pair_count.size()) //increases pair_count size based on neighbour tag view
                            {
                                for (int j = 0; j < (known_in_frame.size() - known_tags[index].pair_count.size()); j++)
                                {
                                    known_tags[index].pair_count.push_back(0);
                                }
                            }
                            known_tags[index].pair_count[0] += 1; //frames including tag
                            known_tags[index].pair_count[known_in_frame.size() - 1] += 1; //frames with n tags inclusive
                        }                    
                    }
                }
                
                if (opt_frames > 50) // check for optimization criteria 
                {
                    camera_calibration::CameraCalibrationOptimizer tag_optimizer(package_path, 1);
                    tag_optimizer.optimize();
                    tag_optimizer.printResultsToConsole();
                    opt_frames = 0; 
                    //optimization initialization 
                    // run optimization 
                    //process data, update and delete
                }

            }
            known_in_frame.clear();
            opt_in_frame.clear();
        }

        int detectionType(apriltag_ros::AprilTagDetectionArray::ConstPtr msg)
        {
            int type = UNKNOWN;
            for (int i = 0; i != msg->detections.size(); i++)
            {
                if (msg->detections[i].id[0] == known_tags[0].id) //world present?
                {
                    known_tags[0].pose_count = 1; ///
                    world_loc = i;
                    type = WORLD;
                    known_in_frame.push_back(msg->detections[i].id[0]);

                }
                else
                {
                    auto it = std::find_if( known_tags.begin(), known_tags.end(), boost::bind(&tag::avg_id,_1) == msg->detections[i].id[0]);
                    if (it != known_tags.end()) //known tag present?
                    {
                        known_tag_loc = i;
                        if (type != WORLD)
                        {
                            type = KNOWN;
                        }
                        known_in_frame.push_back(msg->detections[i].id[0]);
                    }
                    auto itor = std::find( toBeOpt.begin(), toBeOpt.end(), msg->detections[i].id[0]); //is tag pending optimization?
                    if (itor != toBeOpt.end())
                    {
                        opt_in_frame.push_back(msg->detections[i].id[0]);
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
            // std::cout << rodrigues_rvec << "   previous" << std::endl; 
            cv::Rodrigues(rodrigues_rvec, camTtag_rvec);  

            cam_T_tag << camTtag_rvec.at<double>(0, 0), camTtag_rvec.at<double>(0, 1), camTtag_rvec.at<double>(0, 2), camTtag_tvec.at<double>(0, 0), 
                         camTtag_rvec.at<double>(1, 0), camTtag_rvec.at<double>(1, 1), camTtag_rvec.at<double>(1, 2), camTtag_tvec.at<double>(0, 1), 
                         camTtag_rvec.at<double>(2, 0), camTtag_rvec.at<double>(2, 1), camTtag_rvec.at<double>(2, 2), camTtag_tvec.at<double>(0, 2), 
                         0, 0, 0, 1;
    
            // std::array<double, 6> rvec;
            // rvec = poseMat2Vec(cam_T_tag);
            // std::cout << rvec[0] << std::endl;
            // std::cout << rvec[1] << std::endl;
            // std::cout << rvec[2] << std::endl;
            
            if (inverse == true)
            {
                return cam_T_tag.inverse().eval();
            }
            return cam_T_tag;
        }

        std::array<double, 6> poseMat2Vec(Eigen::MatrixXd tempmat)
        {
          double data[9] = {tempmat(0,0), tempmat(0,1), tempmat(0,2),
                            tempmat(1,0), tempmat(1,1), tempmat(1,2),
                            tempmat(2,0), tempmat(2,1), tempmat(2,2)};
          cv::Mat camTtag_rvec;
          camTtag_rvec = cv::Mat(3,3,CV_64F,data);
          cv::Mat rodrigues_rvec;

          cv::Rodrigues(camTtag_rvec, rodrigues_rvec); 

          std::array<double, 6> w_T_tag;
          w_T_tag[0] = rodrigues_rvec.at<double>(0,0);
          w_T_tag[1] = rodrigues_rvec.at<double>(0,1);
          w_T_tag[2] = rodrigues_rvec.at<double>(0,2);  

          w_T_tag[3] = tempmat(0,3);
          w_T_tag[4] = tempmat(1,3);
          w_T_tag[5] = tempmat(2,3);

          return w_T_tag;
        }

        void newTag(apriltag_ros::AprilTagDetectionArray::ConstPtr msg, int loc)
        {
            auto itor = std::find_if( known_tags.begin(), known_tags.end(), boost::bind(&tag::id,_1) == msg->detections[loc].id[0]);
            if (itor == known_tags.end()) //unseen tag
            {
                tag temp;
                temp.id = msg->detections[loc].id[0];
                temp.avg_id = temp.id + 1000;
                temp.size = msg->detections[loc].size[0];
                temp.wTtag = wTcam * camTtag(msg, loc,false);
                temp.pose_sum = temp.wTtag;
                temp.pose_count = 1;
                temp.pair_count = { 0, 0 }; 
                known_tags.push_back(temp);
                // std::cout<<temp.wTtag<<std::endl;
            }
            else if (itor != known_tags.end()) //seen tag 
            {
                int index = itor - known_tags.begin();
                known_tags[index].pose_sum += wTcam * camTtag(msg, loc,false);
                known_tags[index].pose_count += 1;
                // std::cout<<known_tags[index].pose_sum<<std::endl;

                if (known_tags[index].pose_count == 10) //criteria for pose_count to be set based on frame sample publishing rate
                {
                    known_tags[index].avg_id -= 1000;
                    known_tags[index].wTtag = known_tags[index].pose_sum / known_tags[index].pose_count;
                    known_tags[index].wTtag_vec = poseMat2Vec(known_tags[index].wTtag);
                    // std::cout << known_tags[index].wTtag << std::endl;

                    toBeOpt.push_back(known_tags[index].id); //acknowledge beginning of optimization cache
                }
            }
        }

        Picture msg2Picture(apriltag_ros::AprilTagDetectionArray::ConstPtr msg)
        {
            Picture pic;
            pic.world_T_camera = poseMat2Vec(wTcam);
            pic.camera_T_world = poseMat2Vec(wTcam.inverse().eval());
            
            for (int i = 0; i != msg->detections.size(); i++)
            {
                Detection temp;
                temp.targetID = msg->detections[i].id[0];
                temp.size = {msg->detections[i].size[0], msg->detections[i].size[0]};

                std::array<std::array<double, 2>, 4> corners;
                for (int j = 0; j < 4; j++)
                {
                    corners[j] = { msg->detections[i].pixel_corners_x[j], msg->detections[i].pixel_corners_y[j] };
                }
                temp.corners = corners;

                pic.detections.push_back(temp);
            }

            return pic;
        }

};

int main(int argc , char **argv)
{
    ros::init(argc, argv, "vslam");
    Node vslam;
    ros::spin();

    return 0;
}