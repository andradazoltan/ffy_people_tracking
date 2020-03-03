#pragma once
/*
 * Centroid.h
 *
 * Class for tracking an InferenceBoundingBox using only the x-position
 * of the center of the bounding box. 
 *
 * Created on: 6 Feb 2020
 * Author: Andrada Zoltan
 */

#include "Tracker.h"
#include <vector>

class Centroid : public Tracker {
public:
    Centroid(Spinnaker::InferenceBoundingBox box);

    bool isBoxMatch(Spinnaker::InferenceBoundingBox box);
    void updateTracker(Spinnaker::InferenceBoundingBox box);
    int updateTracker(void);
    bool getDir(void);

    ~Centroid();

private:
    // Direction of travel of the bounding box
    bool dir;

    // Previous bounding box center
    std:: vector<int>* centerPrev;
};