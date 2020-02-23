#pragma once

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <vector>

#define CAM_X           1440
#define CAM_Y           770
#define INFERENCE_TIME 160

class Centroid {
public:
    Centroid(Spinnaker::InferenceBoundingBox box);
    bool IsBoxMatch(Spinnaker::InferenceBoundingBox box);
    void updateCentroid(Spinnaker::InferenceBoundingBox box);
    int updateCentroid();

    bool getDir(void);
    ~Centroid();

private:
    // Counter for how many frames this has not appeared in
    int count;

    /*
     * This is a 4-element vector containing the current
     * estimate of :
     *      (*x)[0] = x position in pixels
     *      (*x)[1] = y position in pixels
     *      (*x)[2] = x velocity in pixels/ms
     *      (*x)[3] = length of box diagonal
     * 
     * Note that the velocity is relative to the point (0,0), so a
     * positive velocity means that the box is moving from right to left
     * in the frame. And if there is a negative velocity, the box is moving
     * from left to right.
     */
    std::vector<double>* state;

    std::vector<double> MakeStateVector(Spinnaker::InferenceBoundingBox box);
};