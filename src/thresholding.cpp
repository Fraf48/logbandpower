#include "../include/logbandpower/thresholding.h"
#include <numeric>
#include <iostream>

Thresholding::Thresholding(ros::NodeHandle& nh) {
    // Default value
    threshold = 0.7;
    selected_channel = 9;
    checkExceeded = false;

    // Getting external parameters if setted
    ros::param::get("thresholding/threshold", threshold);
    ros::param::get("thresholding/channel", selected_channel);
    selected_channel -= 1;    // scaled on [0,nchannels) range

    // Print selected channel and threshold
    ROS_INFO("Using channel: %d", selected_channel+1);
    ROS_INFO("Using threshold: %.2f", threshold);

	// Publisher and Subsriber initialization
    pub = nh.advertise<rosneuro_msgs::NeuroEvent>("/events/bus", 1);
    sub = nh.subscribe("/eeg/bandpower", 1, &Thresholding::thresholdingCallback, this);
}

void Thresholding::thresholdingCallback(const rosneuro_msgs::NeuroFrame::ConstPtr& msg) {
    int nchannels = msg->eeg.info.nchannels;   
    int nsamples = msg->eeg.info.nsamples;
    
    // Checks if selected channel is valid
    if (selected_channel < 0 || selected_channel >= nchannels) {
        ROS_ERROR("Selected channel %d is out of range (1-%d).", selected_channel+1, nchannels);
        return;
    }
    
    // Extracts data for selected channel
    std::vector<float> channel_data;
    for (size_t s = 0; s < nsamples; s++) {
        // Neuroframe contains data as [samples][channels] in a vector (i.e. s1c1, s1c2, s1c3... s2c1, ...)
        channel_data.push_back(msg->eeg.data[s * nchannels + selected_channel]);
    }   

    // Checks if any sample crosses the threshold and send a NeuroEvent
    for (size_t i = 0; i < nsamples; i++) {
        if(!checkExceeded){
            if (channel_data[i] > threshold) {
                ROS_INFO("Signal exceeded threshold %.2f on channel %d", threshold, selected_channel+1);
                rosneuro_msgs::NeuroEvent event_msg = generateMessage();
                pub.publish(event_msg);
                checkExceeded = true;
            }            
        } else {
            // Once the sample return under the threshold we can keep analyzing the signal and eventually send another NeuroEvent
          //  ROS_INFO("Signal with value %f", channel_data[i]);
            if (channel_data[i] <= threshold) {
                checkExceeded = false;
            }              
        }
    }
}

rosneuro_msgs::NeuroEvent Thresholding::generateMessage(){
    rosneuro_msgs::NeuroEvent event_msg;
    // Set the header
    event_msg.header.stamp = ros::Time::now();
    event_msg.header.frame_id = "eeg_thresholding";
    // Set description
    char description[100];
    sprintf(description, "Signal exceeded threshold %.2f on channel %d", threshold, selected_channel+1);
    event_msg.description = std::string(description);
    return event_msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "thresholding");
    ros::NodeHandle nh;

    Thresholding thresholding_node(nh);

    ros::spin();

    return 0;
}