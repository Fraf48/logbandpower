#ifndef BANDPOWER_H
#define BANDPOWER_H

#include <ros/ros.h>
#include <rosneuro_msgs/NeuroFrame.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>

// Class to manage the bandpower node
class BandPowerNode {
public:
    BandPowerNode();
    ~BandPowerNode() = default;

    void run();

private:
    // Callback function
    void callback(const rosneuro_msgs::NeuroFrame::ConstPtr& msg);

    // Manages buffering and bandpower computing
    std::vector<float> bufferedBandPower(const rosneuro_msgs::NeuroFrame::ConstPtr& msg);

    // Generates the message to publish
    std_msgs::Float32MultiArray generateNewMessage(
        const std::vector<float>& bpvals,
        const rosneuro_msgs::NeuroFrame::ConstPtr& msg
    );

    // Variables to handle node activity
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;

    // Circular buffer[channel][n]
    std::vector<std::vector<float>> buffer;

    // Flags and state indicators
    bool newDataFlag;                                   // True if callback received new data
    rosneuro_msgs::NeuroFrame::ConstPtr currentFrame;   // Last data frame received

    // Parameters
    int sampleRate;     // Sample rate
    int ringSize;       // Circular buffer dimension (in samples)
    int seq;            // Counter to check for full buffer
};

#endif 
