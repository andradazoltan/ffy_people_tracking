#pragma once
/*
 *  Tracker.h
 *
 *  Abstract class for defining the low level bounding box tracker.
 *
 *  Created on: Mar 2, 2020
 *  Author: Andrada Zoltan
 */

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

// Number of frames that a bounding box is missing from before it is deleted
#define MISSING_THRESH 5

// Camera resolution
#define CAM_X            1440
#define CAM_Y            770

// Timing constants
#define CAM_MS_PER_FRAME 40
#define INFERENCE_TIME   160

// Directions into/out of frame
#define LEFT  0
#define RIGHT 1

class Tracker {
    public:
        virtual bool isBoxMatch(Spinnaker::InferenceBoundingBox box) = 0;
        virtual void updateTracker(Spinnaker::InferenceBoundingBox box) = 0;
        virtual bool getDir(void) = 0;

        int updateTracker(void) {
            count++;
            if (count > MISSING_THRESH)
                return -1;
            else
                return 0;
        }

    protected:
        // Counter for how many frames this has not appeared in
        int count;
};
