#include "../include/logbandpower/thresholding.h"
#include <numeric>
#include <iostream>

Thresholding::Thresholding(ros::NodeHandle& nh) {
    // Lettura dei parametri in ingresso
    nh.param("channel", this->selected_channel, 1);    // Canale di default: 1
    nh.param("threshold", this->threshold, 0.7f);       // Soglia di default: 0.7

    // Stampa parametri e soglia selezionati
    ROS_INFO("Using channel: %d", this->selected_channel);
    ROS_INFO("Using threshold: %.2f", this->threshold);

	// Inizializza il publisher per inviare messaggi di tipo NeuroEvent sul topic "/events/bus"
    this->pub = nh.advertise<rosneuro_msgs::NeuroEvent>("/events/bus", 1);
    this->sub = nh.subscribe("/eeg/bandpower", 1, &Thresholding::thresholdingCallback, this);
}

void Thresholding::thresholdingCallback(const rosneuro_msgs::NeuroFrame::ConstPtr& msg) {
    int nchannels = msg->eeg.info.nchannels;   
    int nsamples = msg->eeg.info.nsamples;
    
    // Controllo che il canale selezionato sia valido
    if (this->selected_channel < 0 || this->selected_channel >= nchannels) {
        ROS_ERROR("Selected channel %d is out of range (1-%d).", this->selected_channel, nchannels);
        return;
    }
    
    // Estrarre i dati del canale selezionato
    std::vector<float> channel_data;
    for (size_t s = 0; s < nsamples; s++) {
        // Neuroframe contains data as [samples][channels] in a vector (i.e. s1c1, s1c2, s1c3... s2c1, ...)
        channel_data.push_back(msg->eeg.data[s * nchannels + this->selected_channel]);
    }   

    // Controllo se uno dei campioni supera la soglia
    bool event_detected = false;
    for (size_t i = 0; i < msg->eeg.info.nsamples; i++) {
        if (channel_data[i] > this->threshold) {
            ROS_INFO("Threshold exceeded on channel %d with threshold %.2f", this->selected_channel, this->threshold);
            rosneuro_msgs::NeuroEvent event_msg = generateMessage(channel_data[i], msg->header.seq);
            this->pub.publish(event_msg);
        }
    }
}

rosneuro_msgs::NeuroEvent Thresholding::generateMessage(float value, int seq){
    rosneuro_msgs::NeuroEvent event_msg;

    event_msg.header.stamp = ros::Time::now();

    char description[100];
    sprintf(description, "Signal exceeded threshold %.2f on channel %d with value %f at seq %d", this->threshold, this->selected_channel, value, seq);
    event_msg.description = std::string(description);
    return event_msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "thresholding");
    ros::NodeHandle nh("~");

    Thresholding thresholding_node(nh);

    ros::spin();

    return 0;
}
