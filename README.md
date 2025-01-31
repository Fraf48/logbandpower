readme.md

# **NeuroRobotics Project - Bandpower & Thresholding Nodes**

## **Team Contribution**

This project was developed collaboratively, with all team members contributing to its implementation and refinement.

### Team members of *GruppoGe:*

- De Carlo Claudia
- De Nicola Francesco
- Nardin Jaele
- Tecchio Giulia


---

## **Bandpower Node**

The **bandpower** computation follows a slightly modified approach compared to the official tutorial. Specifically, we **compute the logarithm after averaging**, rather than before.

This decision was based on theoretical insights discussed during lectures, ensuring consistency with the expected methodology.

---

## **Thresholding Node**

The **thresholding** node was designed with **code reusability** in mind.
Although the **`/eeg/bandpower`** topic **always provides data with `nsamples = 1`**, the implementation **iterates over `nsamples`** to allow the same node to function correctly in scenarios where **`/eeg/bandpower` messages may have different `nsamples` values** in the future.

Additionally, to **prevent redundant event detections**, an event is published **only once per peak**, rather than at every sample exceeding the threshold.


### **Channel Selection**

The channel used for thresholding was selected by **computing the Fisher Map in MATLAB**.

//IMMAGINE

This analysis helped identify C3 as the most discriminative EEG channel for our task.


### **Threshold Selection**

The **threshold** was determined by **identifying the peaks** in the bandpower signal. The goal was to set a threshold that effectively isolates meaningful fluctuations in the EEG data.

---

## **Usage**

To run the project, we launch the ROS nodes using:

```
roslaunch logbandpower logbandpower.launch
```

