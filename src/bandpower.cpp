#include "logbandpower/bandpower.h"
#include <numeric>
#include <cmath>

BandPowerNode::BandPowerNode() : newDataFlag(false), seq(0) {
    // set variables from parameters
    this->sampleRate = 512;
    ros::param::get("bandpower/samplerate", this->sampleRate);

    this->ringSize = this->sampleRate; // buffer size is 1 second

    // Subscriber and Publisher initialization
    this->sub = nh.subscribe("/eeg/filtered", 1, &BandPowerNode::callback, this);
    this->pub = nh.advertise<rosneuro_msgs::NeuroFrame>("/eeg/bandpower", 1);

    ROS_INFO("[BandPowerNode] Initialized with sample rate=%d and ring_size=%d", this->sampleRate, this->ringSize);
}

void BandPowerNode::run() {
    // Elaboration framerate from parameters
    int framerate = 16;
    ros::param::get("bandpower/framerate", framerate);
    ros::Rate r(framerate);

    while (ros::ok()) {
        ros::spinOnce();
        if (newDataFlag) {
            std::vector<float> bpvals = bufferedBandPower(currentFrame);
            rosneuro_msgs::NeuroFrame msg = generateNewMessage(bpvals, currentFrame);
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

rosneuro_msgs::NeuroFrame BandPowerNode::generateNewMessage(
    const std::vector<float>& bandpower,
    const rosneuro_msgs::NeuroFrame::ConstPtr& oldMsg
) {
    rosneuro_msgs::NeuroFrame newMsg;

    // header
    newMsg.header.stamp = ros::Time::now();
    newMsg.header.frame_id = "eeg_bandpower";

    newMsg.exg.data.clear();
    newMsg.tri.data.clear();


    // eeg setup
    newMsg.eeg.info.nchannels = bandpower.size();
    newMsg.eeg.info.nsamples = 1;
    newMsg.eeg.info.stride = 1;
    newMsg.eeg.info.unit = "dB";

    // data
    newMsg.eeg.data = bandpower;

    return newMsg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bandpower");
    BandPowerNode bpnode;
    bpnode.run();
    return 0;
}
