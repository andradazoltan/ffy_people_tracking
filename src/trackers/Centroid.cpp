/*
 *  Centroid.cpp
 *
 *  Created on: Feb 6, 2020
 *  Author: Andrada Zoltan
 */

#include "Centroid.h"
#include <iostream>
#include <thread>

#define DIST_TOLERANCE 300
#define MISSING_THRESH 15

using namespace Spinnaker;

using std::cout;
using std::vector;
using std::thread;

Centroid::Centroid(InferenceBoundingBox box) {
    count = 0;
    centerPrev = new vector<int>(2);

    (*centerPrev)[0] = (box.rect.bottomRightXCoord + box.rect.topLeftXCoord) / 2;
    (*centerPrev)[1] = (box.rect.bottomRightYCoord + box.rect.topLeftYCoord) / 2;

    if ((*centerPrev)[0] < (CAM_X / 2))
        dir = RIGHT;
    else
        dir = LEFT;
}

bool Centroid::isBoxMatch(Spinnaker::InferenceBoundingBox box) {
    // Get the distance from the previous center
    int centerXCurr = (box.rect.bottomRightXCoord + box.rect.topLeftXCoord) / 2;
    int centerYCurr = (box.rect.bottomRightYCoord + box.rect.topLeftYCoord) / 2;

    if (abs(centerXCurr - (*centerPrev)[0]) > DIST_TOLERANCE)
        return false;
    else
        return true;
}

void Centroid::updateTracker(InferenceBoundingBox box) {
    // Reset missing counter
    count = 0;

    // Get current centers
    int centerXCurr = (box.rect.bottomRightXCoord + box.rect.topLeftXCoord) / 2;
    int centerYCurr = (box.rect.bottomRightYCoord + box.rect.topLeftYCoord) / 2;

    // Update the centroid
    (*centerPrev)[0] = centerXCurr;
    (*centerPrev)[1] = centerYCurr;
}

int Centroid::updateTracker(void) {
    return (Tracker::updateTracker());
}

bool Centroid::getDir(void) {
    return dir;
}

Centroid::~Centroid() {
    delete centerPrev;
}