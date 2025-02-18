#ifndef BANDPOWER_H
#define BANDPOWER_H

#include <ros/ros.h>
#include <rosneuro_msgs/NeuroFrame.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>

// Class to manage the bandpower node
class BandPower {
public:
    BandPower();
    ~BandPower() = default;

    void run();

private:
    // Callback function
    void callback(const rosneuro_msgs::NeuroFrame::ConstPtr& msg);

    // Function to manage buffering and bandpower computation
    std::vector<float> bufferedBandPower(const rosneuro_msgs::NeuroFrame::ConstPtr& msg);

    // Function to generate the message to publish
    rosneuro_msgs::NeuroFrame generateNewMessage(
        const std::vector<float>& bandpower,
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
