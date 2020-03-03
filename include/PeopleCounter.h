#pragma once
/*
 *  PeopleCounter.h
 *
 *  Created on: Feb 6, 2020
 *  Author: Andrada Zoltan
 */

#include "Tracker.h"
#include "HikerCam.h"
#include <vector>
#include <atomic>
#include <iostream>
#include <thread>

#define COUNT_THRESH 5
#define PERSON_ID 15
#define CONFIDENCE_THRESH 0.70

using namespace Spinnaker;

using std::cout;
using std::vector;
using std::thread;
using std::atomic;

template <class T>
class PeopleCounter {
    public:
        PeopleCounter();
        ~PeopleCounter();

        int InitPeopleCounter();
        void StartPeopleCounter();
        void StopPeopleCounter();
        int GetPeopleCount();

    private:
        atomic<int> peopleCount;

        HikerCam* mCam;
        atomic<bool> endTrackingSignal;
        vector<T*>* tracker;
};

/******************* Function Definitions ******************/
template <class T>
PeopleCounter<T>::PeopleCounter() : peopleCount(0), endTrackingSignal(false) {
    tracker = new vector<T*>();
    mCam = new HikerCam();
}

template <class T>
int PeopleCounter<T>::InitPeopleCounter() {
    if (mCam->InitCamera())
        return -1;
    else
        return 0;
}

template <class T>
void PeopleCounter<T>::StartPeopleCounter() {
    // Create acquisition thread
    thread acqThread(&HikerCam::StartAcquisition, mCam);

    while (!endTrackingSignal) {
        Sleep(INFERENCE_TIME);
        vector<InferenceBoundingBox> boundingBoxes;
        mCam->GetBoundingBoxData(boundingBoxes);

        if (tracker->size() == 0) {
            // Make new boxes for each of them 
            for (auto it = boundingBoxes.begin(); it != boundingBoxes.end(); ++it) {
                InferenceBoundingBox box = *it;

                // Create new centroid
                if (box.classId == PERSON_ID && box.confidence > CONFIDENCE_THRESH) {
                    T* tr = new T(box);
                    tracker->push_back(tr);
                }
            }
        }
        else {
            // Compare the distances with all existing objects
            for (auto it = boundingBoxes.begin(); it != boundingBoxes.end(); ++it) {
                InferenceBoundingBox box = *it;

                if (box.classId == PERSON_ID && box.confidence > CONFIDENCE_THRESH) {
                    bool match = false;
                    for (auto it_ctr = tracker->begin(); it_ctr != tracker->end(); ++it_ctr) {
                        if ((*it_ctr)->isBoxMatch(box)) {
                            match = true;
                            (*it_ctr)->updateTracker(box);
                            break;
                        }
                    }

                    // Make a new tracker if the existing ones don't match
                    if (!match) {
                        T* tr = new T(box);
                        tracker->push_back(tr);
                    }
                }
            }
        }

        // Update all trackers for next round of comparison
        for (auto it_ctr = tracker->begin(); it_ctr != tracker->end(); ++it_ctr) {
            if ((*it_ctr)->updateTracker() == -1) {
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
        int numTrackers = (int)tracker->size();
        for (int i = 0; i < numTrackers; i++) {
            if ((*tracker)[i] == NULL) {
                tracker->erase(tracker->begin() + i);
                numTrackers--;
            }
        }
    }

    // Stop acquistion
    mCam->EndAcquisition();

    // Wait for acquistion thread to end
    acqThread.join();
}

template <class T>
void PeopleCounter<T>::StopPeopleCounter() {
    endTrackingSignal.store(true);
}

template <class T>
int PeopleCounter<T>::GetPeopleCount() {
    return peopleCount;
}

template <class T>
PeopleCounter<T>::~PeopleCounter() {
    delete tracker;
}