#ifndef THRESHOLDING_H
#define THRESHOLDING_H

#include <ros/ros.h>
#include <rosneuro_msgs/NeuroFrame.h> 
#include <rosneuro_msgs/NeuroEvent.h> 
#include <vector>

// Class to manage the threshold node
class Thresholding {
public:
    Thresholding(ros::NodeHandle& nh); 

    // Callback function to elaborate EEG data
    void thresholdingCallback(const rosneuro_msgs::NeuroFrame::ConstPtr& msg); 

    // Function to generate the message to publish
    rosneuro_msgs::NeuroEvent generateMessage(float value, int seq);

private:
    ros::Publisher pub; 
    ros::Subscriber sub; 
    int selected_channel;   // EEG channnel selected to check the threshold, rangee  [0,nchannels)
    float threshold;        // Threshold value
};

#endif