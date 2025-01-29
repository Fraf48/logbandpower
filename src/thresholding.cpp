#include "../include/logbandpower/thresholding.h"
#include <numeric>
#include <iostream>

Thresholding::Thresholding(ros::NodeHandle& nh) {
    // default value
    this->threshold = 0.7;
    this->selected_channel = 9;

    // getting external parameters if setted
    ros::param::get("thresholding/threshold", this->threshold);
    ros::param::get("thresholding/channel", this->selected_channel);

    // Stampa parametri e soglia selezionati
    ROS_INFO("Using channel: %d", this->selected_channel);
    ROS_INFO("Using threshold: %.2f", this->threshold);

	// Inizializza il publisher per inviare messaggi di tipo NeuroEvent sul topic "/events/bus"
    this->thresholding_pub = nh.advertise<rosneuro_msgs::NeuroEvent>("/events/bus", 1);
    this->sub = nh.subscribe("/eeg/bandpower", 1, &Thresholding::thresholdingCallback, this);
}

void Thresholding::thresholdingCallback(const rosneuro_msgs::NeuroFrame::ConstPtr& msg) {
    // Controllo che il canale selezionato sia valido
    if (this->selected_channel < 0 || this->selected_channel >= msg->eeg.info.nchannels) {
        ROS_ERROR("Selected channel %d is out of range (1-%d).", this->selected_channel, msg->eeg.info.nchannels);
        return;
    }
    
    // Estrarre i dati del canale selezionato
    std::vector<float> channel_data;
    for (size_t i = 0; i < msg->eeg.info.nsamples; i++) {
        // data index = channel selected * number of sample + i
        channel_data.push_back(msg->eeg.data[(this->selected_channel - 1)* msg->eeg.info.nsamples + i]);
    }

    // Controllo se uno dei campioni supera la soglia
    bool event_detected = false;
    for (size_t i = 0; i < msg->eeg.info.nsamples; i++) {
        if (channel_data[i] > this->threshold) {
            event_detected = true;
            break;
        }
    }

    // Se viene rilevato un evento pubblica un NeuroEvent
    if (event_detected) {
        ROS_INFO("Threshold exceeded on channel %d with threshold %.2f", this->selected_channel, this->threshold);

        rosneuro_msgs::NeuroEvent event_msg;
        event_msg.description = "Signal exceeded on channel %d with threshold %.2f", this->selected_channel, this->threshold;

        this->thresholding_pub.publish(event_msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "thresholding");
    ros::NodeHandle nh;

    Thresholding thresholding_node(nh);

    ros::spin();

    return 0;
}
