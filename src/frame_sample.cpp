#include "ros/ros.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

class Node
{
    public:
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber sub;
        ros::Timer timer;
        apriltag_ros::AprilTagDetectionArray::ConstPtr message;
        double sample_rate;
        bool wait;

        Node()
        {
            pub = n.advertise<apriltag_ros::AprilTagDetectionArray>("sampled_detections",1000);
            sub = n.subscribe("tag_detections", 1000, &Node::sampleCallback,this);

            //load system parameters
            if (n.hasParam("/sample_rate"))
            {
                n.getParam("/sample_rate",sample_rate);
            }
            else 
            {
              ROS_ERROR("Sampling rate not loaded to parameter server!");
              ros::shutdown();
            }
            
            wait = false;
        } 

        void sampleCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
        {
            message = msg;
            ROS_INFO("Received message!"); //subscription indicator
            //debugging output for tags observed
            if(!msg->detections.empty())
            {
                ROS_INFO("Observed tags:");
                for(int i=0; i<msg->detections.size();i++)
                {
                    ROS_INFO("  -%i",msg->detections[i].id[0]);
                }
            }
            
            if (wait == false)
            {
                //ros timer call for sampling parameter
                timer = n.createTimer(ros::Duration(1 / sample_rate),&Node::publishMsg, this);
                wait = true; 
            }
        }

        void publishMsg(const ros::TimerEvent &)
        {
            pub.publish(message);
            ROS_INFO("PUBLISHED message!"); // publishing indicator
            wait == false;
        }
        
};

int main(int argc , char **argv)
{
    ros::init(argc, argv, "frame_sample");
    Node frame_sampler;
    ros::spin();

    return 0;
}
