/*
 *  PeopleTracker.cpp
 *
 *  Created on: Feb 6, 2020
 *  Author: Andrada Zoltan
 */

#include "PeopleTracker.h"
#include <iostream>
#include <thread>

#define COUNT_THRESH 5
#define PERSON_ID 15
#define CONFIDENCE_THRESH 0.70

#define LEFT 0
#define RIGHT 1

using namespace Spinnaker;

using std::cout;
using std::vector;
using std::thread;

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
        Sleep(INFERENCE_TIME);
        vector<InferenceBoundingBox> boundingBoxes;
        mCam->GetBoundingBoxData(boundingBoxes);

        if (centroidTracker->size() == 0) {
            // Make new boxes for each of them 
            for (auto it = boundingBoxes.begin(); it != boundingBoxes.end(); ++it) {
                InferenceBoundingBox box = *it;

                // Create new centroid
                if (box.classId == PERSON_ID && box.confidence > CONFIDENCE_THRESH) {
                    Centroid* cent = new Centroid(box);
                    centroidTracker->push_back(cent);
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
                        if ((*it_ctr)->IsBoxMatch(box)) {
                            match = true;
                            (*it_ctr)->updateCentroid(box);
                            cout << "match\n";
                            break;
                        }
                    }

                    // Make a new tracker if the existing ones don't match
                    if (!match) {
                        cout << "NEW!\n";
                        Centroid* cent = new Centroid(box);
                        centroidTracker->push_back(cent);
                    }
                }
            }
        }

        // Update all trackers for next round of comparison
        for (auto it_ctr = centroidTracker->begin(); it_ctr != centroidTracker->end(); ++it_ctr) {
            if ((*it_ctr)->updateCentroid() == -1) {
                cout << "DELETE\n";
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

