/*
 *  PeopleTracker.cpp
 *
 *  Created on: Feb 6, 2020
 *  Author: Andrada Zoltan
 */

#include "PeopleTracker.h"
#include <iostream>
#include <thread>

#define LEFT true // Into trail
#define RIGHT false // Out of trail
#define HALF_RES 720

#define DIST_TOLERANCE 300
#define MISSING_THRESH 15

#define PERSON_ID 15
#define CONFIDENCE_THRESH 0.70

using namespace Spinnaker;

using std::cout;
using std::vector;
using std::thread;

/******************** Centroid Class ******************************/
class Centroid {
public:
    Centroid::Centroid(InferenceBoundingBox box) {
        missingCnt = 0;
        centerPrev = new vector<int>(2);

        (*centerPrev)[0] = (box.rect.bottomRightXCoord + box.rect.topLeftXCoord) / 2;
        (*centerPrev)[1] = (box.rect.bottomRightYCoord + box.rect.topLeftYCoord) / 2;

        if ((*centerPrev)[0] < HALF_RES)
            dir = RIGHT;
        else
            dir = LEFT;
    }

    int getDistance(InferenceBoundingBox box) {
        int centerXCurr = (box.rect.bottomRightXCoord + box.rect.topLeftXCoord) / 2;
        int centerYCurr = (box.rect.bottomRightYCoord + box.rect.topLeftYCoord) / 2;
        return abs(centerXCurr - (*centerPrev)[0]);
    }

    bool getDir() {
        return dir;
    }

    void updateCentroid(InferenceBoundingBox box) {
        // Reset missing counter
        missingCnt = 0;

        // Get current centers
        int centerXCurr = (box.rect.bottomRightXCoord + box.rect.topLeftXCoord) / 2;
        int centerYCurr = (box.rect.bottomRightYCoord + box.rect.topLeftYCoord) / 2;

        // Update direction
        int xDiff = centerXCurr - (*centerPrev)[0];
        if (abs(xDiff) > DIST_TOLERANCE) {
            if (xDiff > 0) {
                dir = LEFT;
            }
            else {
                dir = RIGHT;
            }
        }

        // Update the centroid
        (*centerPrev)[0] = centerXCurr;
        (*centerPrev)[1] = centerYCurr;
    }

    int updateCentroid() {
        missingCnt++;
        if (missingCnt > MISSING_THRESH)
            return -1;
        else
            return 0;
    }

    Centroid::~Centroid() {
        delete centerPrev;
    }

private:
    bool dir;
    int missingCnt;
    vector<int>* centerPrev;
};

/********************* People Tracker Class ****************************/
PeopleTracker::PeopleTracker() : peopleCount(0), endTrackingSignal(false){
    centroidTracker = new vector<Centroid*>();
    mCam = new HikerCam();
}

int PeopleTracker::InitTracker() {
    if (mCam->InitCamera())
        return -1;
}

void PeopleTracker::StartTracking() {
    // Create acquisition thread
    thread acqThread(&HikerCam::StartAcquisition, mCam);

    while (!endTrackingSignal) {
        // Wait a little before each attempt
        Sleep(50);

        vector<InferenceBoundingBox> boundingBoxes;
        mCam->GetBoundingBoxData(boundingBoxes);

        if (centroidTracker->size() == 0) {
            // Make new boxes for each of them 
            for (auto it = boundingBoxes.begin(); it != boundingBoxes.end(); ++it) {
                InferenceBoundingBox box = *it;

                // Create new centroid
                if (box.classId == PERSON_ID && box.confidence > CONFIDENCE_THRESH) {
                    Centroid* centr = new Centroid(box);
                    centroidTracker->push_back(centr);
                }
            }
        } 
        else {
            // Compare the distances with all existing objects
            for (auto it = boundingBoxes.begin(); it != boundingBoxes.end(); ++it) {
                InferenceBoundingBox box = *it;

                if (box.classId == PERSON_ID && box.confidence > CONFIDENCE_THRESH) {
                    bool match = false;
                    for (auto it_ctr = centroidTracker->begin(); it_ctr != centroidTracker->end(); ++it_ctr) {
                        int dist = (*it_ctr)->getDistance(box);
                        cout << "DIST: " << dist << "\n";
                        if (dist < DIST_TOLERANCE) {
                            match = true;
                            (*it_ctr)->updateCentroid(box);
                            break;
                        }
                    }

                    // Make a new centroid if the existing ones don't match
                    if (!match) {
                        cout << "NEW\n";
                        Centroid* centr = new Centroid(box);
                        centroidTracker->push_back(centr);
                    }
                }
            }
            
            // Update all centroids
            for (auto it_ctr = centroidTracker->begin(); it_ctr != centroidTracker->end(); ++it_ctr) {
                if ((*it_ctr)->updateCentroid()) {
                    // Update people counter
                    if ((*it_ctr)->getDir() == LEFT)
                        peopleCount.store(peopleCount + 1);
                    else if (peopleCount != 0)
                        peopleCount.store(peopleCount - 1);
                    delete (*it_ctr);
                    *it_ctr = NULL;
                }
            }

            // Erase whatever trackers were deallocated in the previous step
            int numTrackers = centroidTracker->size();
            for (int i = 0; i < numTrackers; i++) {
                if ((*centroidTracker)[i] == NULL) {
                    centroidTracker->erase(centroidTracker->begin() + i);
                    numTrackers--;
                }
                    
            }
        }
    }

    // Stop acquistion
    mCam->EndAcquisition();

    // Wait for acquistion thread to end
    acqThread.join();
}

void PeopleTracker::StopTracking() {
    endTrackingSignal.store(true);
}

int PeopleTracker::GetPeopleCount() {
    return peopleCount;
}

PeopleTracker::~PeopleTracker() {
    delete centroidTracker;
}

