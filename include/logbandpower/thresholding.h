#ifndef THRESHOLDING_H
#define THRESHOLDING_H

#include <ros/ros.h>
#include <rosneuro_msgs/NeuroFrame.h> 
#include <rosneuro_msgs/NeuroEvent.h> 
#include <vector>

// Classe Thresholding per la gestione del superamento soglie su dati EEG
class Thresholding {
public:
    Thresholding(ros::NodeHandle& nh); 
    void thresholdingCallback(const rosneuro_msgs::NeuroFrame::ConstPtr& msg); // Funzione di callback per elaborare i dati EEG

private:
    ros::Publisher thresholding_pub; 
    ros::Subscriber sub; 
    int selected_channel;   // Canale EEG selezionato per il controllo della soglia
    float threshold;        // Valore della soglia per il rilevamento di eventi
};

#endif
