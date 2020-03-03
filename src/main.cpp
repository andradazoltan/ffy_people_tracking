/*
 * main.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: Andrada Zoltan
 */

#include "PeopleCounter.h"
#include "PeopleCounter.cpp" // Get rid of linker error
#include "Tracker.h"
#include "Centroid.h"
#include "Kalman.h"
#include "StateCentroid.h"
#include <iostream>
#include <thread>

/*
 * Defines which tracker implementation to use:
 *      1 - Centroid
 *      2 - Kalman
 *      3 - StateCentroid
 */
#define TRACKER_IMPL 3

using namespace Spinnaker;
using std::cout;
using std::thread;
using std::vector;

int main(void) {
    int err = 0;

#if (TRACKER_IMPL == 1) 
    PeopleCounter<Centroid>* cntr = new PeopleCounter<Centroid>();
#elif (TRACKER_IMPL == 2)
    PeopleCounter<Kalman>* cntr = new PeopleCounter<Kalman>();
#elif (TRACKER_IMPL == 3)
    PeopleCounter<StateCentroid>* cntr = new PeopleCounter<StateCentroid>();
#endif

    err = cntr->InitPeopleCounter();
    if (err) {
        cout << "Error InitTracker!\n";
        return -1;
    }

    // Create acquisition thread.
#if (TRACKER_IMPL == 1) 
    thread acqThread(&PeopleCounter<Centroid>::StartPeopleCounter, cntr);
#elif (TRACKER_IMPL == 2)
    thread acqThread(&PeopleCounter<Kalman>::StartPeopleCounter, cntr);
#elif (TRACKER_IMPL == 3)
    thread acqThread(&PeopleCounter<StateCentroid>::StartPeopleCounter, cntr);
#endif

    while (1) {
        cout << cntr->GetPeopleCount() << "\n";
        Sleep(500);
    }

    delete cntr;
    return 0;
}


