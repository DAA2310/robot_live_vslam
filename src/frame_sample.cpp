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
        bool wait;

        Node()
        {
            pub = n.advertise<apriltag_ros::AprilTagDetectionArray>("publish13",1000);
            sub = n.subscribe("tag_detections", 1000, &Node::sampleCallback,this);
            wait = false;
        } 

        void sampleCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
        {
            ROS_INFO("I heard %f", msg->translation[0]); /////
            message = msg;
            if (wait == false)
            {
                timer = n.createTimer(ros::Duration(0.5),&Node::publishMsg, this);
                wait = true; 
            }
        }

        void publishMsg(const ros::TimerEvent &)
        {
            pub.publish(message);
            ROS_INFO("spoke %f", message->translation[0]); /////
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
