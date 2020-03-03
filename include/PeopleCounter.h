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
        std::atomic<int> peopleCount;

        HikerCam* mCam;
        std::atomic<bool> endTrackingSignal;
        std::vector<T*>* tracker;
};
