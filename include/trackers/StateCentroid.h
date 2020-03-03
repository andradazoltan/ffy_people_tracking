#pragma once
/*
 * StateCentroid.h
 *
 * Class for tracking an InferenceBoundingBox using only the x-position
 * of the center of the bounding box.
 *
 * Created on: 22 Feb 2020
 * Author: Andrada Zoltan
 */

#include "Tracker.h"

class StateCentroid : public Tracker {
	public:
		StateCentroid(Spinnaker::InferenceBoundingBox box);

        bool isBoxMatch(Spinnaker::InferenceBoundingBox box);
        void updateTracker(Spinnaker::InferenceBoundingBox box);
        bool getDir(void);

        ~StateCentroid();

    private:
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