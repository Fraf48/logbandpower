#include "logbandpower/bandpower.h"
#include <numeric>
#include <cmath>

BandPowerNode::BandPowerNode() : newDataFlag(false), seq(0) {
    // set variables from parameters
    nh.param<int>("samplerate", sampleRate, 512);
    ringSize = sampleRate; // buffer size is 1 second

    // Subscriber and Publisher initialization
    sub = nh.subscribe("/eeg/filtered", 1, &BandPowerNode::callback, this);
    pub = nh.advertise<std_msgs::Float32MultiArray>("/eeg/bandpower", 1);

    ROS_INFO("[BandPowerNode] Initialized with sample rate=%d and ring_size=%d", sampleRate, ringSize);
}

void BandPowerNode::run() {
    // Elaboration framerate from parameters
    int framerate;
    nh.param<int>("framerate", framerate, 16); 
    ros::Rate r(framerate);

    while (ros::ok()) {
        ros::spinOnce();
        if (newDataFlag) {
            std::vector<float> bpvals = bufferedBandPower(currentFrame);
            std_msgs::Float32MultiArray msg = generateNewMessage(bpvals, currentFrame);
            pub.publish(msg);
            newDataFlag = false;
        }
        r.sleep();
    }
}

void BandPowerNode::callback(const rosneuro_msgs::NeuroFrame::ConstPtr& msg) {
    // saves data received and set the flag to true
    currentFrame = msg;
    newDataFlag = true;
}

std::vector<float> BandPowerNode::bufferedBandPower(const rosneuro_msgs::NeuroFrame::ConstPtr& msg) {
    int nchannels = msg->eeg.info.nchannels;   
    int nsamples = msg->eeg.info.nsamples;

    // If seq == 0, buffer's structure must be initialized
    if (seq == 0) {
        buffer.resize(nchannels);
        for (int ch = 0; ch < nchannels; ch++) {
            buffer[ch].resize(ringSize, 0.0f); // initialization to 0.0f
        }
    }

    //  EEG data estraction
    std::vector<std::vector<float>> eegData(nchannels, std::vector<float>(nsamples, 0.0f));
    int index = 0;
    for (int s = 0; s < nsamples; s++) {
        for (int ch = 0; ch < nchannels; ch++) {
            eegData[ch][s] = msg->eeg.data[index++];
        }
    }

    // buffer shifting to remove older samples and add new ones
    for (int ch = 0; ch < nchannels; ch++) {        
        int shift = ringSize - nsamples;
        for (int i = 0; i < shift; i++) {
            buffer[ch][i] = buffer[ch][i + nsamples];
        }
        
        for (int i = 0; i < nsamples; i++) {
            buffer[ch][shift + i] = eegData[ch][i];
        }
    }

    // sequence number update
    seq += 1;
    
    std::vector<float> avgs(nchannels, 0.0f);
    // if the buffer is filled:
    if (seq * nsamples >= ringSize) {
        for (int ch = 0; ch < nchannels; ch++) {
            float sum = 0.0f;
            for (int i = 0; i < ringSize; i++) {
                float val = buffer[ch][i];
                // x^2
                val = val * val;
                // log10(x^2)
                val = std::log10(val + 1e-9); // +1e-9 to avoid log(0)
                sum += val;
            }
            avgs[ch] = sum / ringSize;
        }
    }

    return avgs; 
}

std_msgs::Float32MultiArray BandPowerNode::generateNewMessage(
    const std::vector<float>& bandpower,
    const rosneuro_msgs::NeuroFrame::ConstPtr& oldMsg
) {
    std_msgs::Float32MultiArray newMsg;

    // layout setup
    newMsg.layout.dim.resize(1);
    newMsg.layout.dim[0].label = "height";
    newMsg.layout.dim[0].size = bandpower.size();
    newMsg.layout.dim[0].stride = bandpower.size();
    newMsg.layout.data_offset = 0;

    // data
    newMsg.data.insert(newMsg.data.end(), bandpower.begin(), bandpower.end());

    return newMsg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bandpower");
    BandPowerNode bpnode;
    bpnode.run();
    return 0;
}
