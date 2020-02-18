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
    kalmanTracker = new vector<Kalman*>();
    mCam = new HikerCam();
}

int PeopleTracker::InitTracker() {
    if (mCam->InitCamera())
        return -1;
}

void PeopleTracker::StartTracking() {
    // Create acquisition thread
    thread acqThread(&HikerCam::StartAcquisition, mCam);

    // Sleep to misalign readings with new bounding box entries
    Sleep(60);

    while (!endTrackingSignal) {
        Sleep(INFERENCE_TIME);
        vector<InferenceBoundingBox> boundingBoxes;
        mCam->GetBoundingBoxData(boundingBoxes);

        if (kalmanTracker->size() == 0) {
            // Make new boxes for each of them 
            for (auto it = boundingBoxes.begin(); it != boundingBoxes.end(); ++it) {
                InferenceBoundingBox box = *it;

                // Create new centroid
                if (box.classId == PERSON_ID && box.confidence > CONFIDENCE_THRESH) {
                    Kalman* kalm = new Kalman(box);
                    kalmanTracker->push_back(kalm);
                }
            }
        } 
        else {
            // Compare the distances with all existing objects
            for (auto it = boundingBoxes.begin(); it != boundingBoxes.end(); ++it) {
                InferenceBoundingBox box = *it;

                if (box.classId == PERSON_ID && box.confidence > CONFIDENCE_THRESH) {
                    bool match = false;
                    for (auto it_ctr = kalmanTracker->begin(); it_ctr != kalmanTracker->end(); ++it_ctr) {
                        if ((*it_ctr)->IsBoxMatch(box)) {
                            match = true;
                            (*it_ctr)->Update(box);
                            cout << "match\n";
                            break;
                        }
                    }

                    // Make a new tracker if the existing ones don't match
                    if (!match) {
                        cout << "NEW!\n";
                        Kalman* kalm = new Kalman(box);
                        kalmanTracker->push_back(kalm);
                    }
                }
            }
        }

        // Update all trackers for next round of comparison
        for (auto it_ctr = kalmanTracker->begin(); it_ctr != kalmanTracker->end(); ++it_ctr) {
            (*it_ctr)->Predict();
            if ((*it_ctr)->updateCounters() > COUNT_THRESH) {
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
        int numTrackers = kalmanTracker->size();
        for (int i = 0; i < numTrackers; i++) {
            if ((*kalmanTracker)[i] == NULL) {
                kalmanTracker->erase(kalmanTracker->begin() + i);
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
    delete kalmanTracker;
}

