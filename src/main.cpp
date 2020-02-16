/*
 * main.cpp
 *
 *  Created on: Feb 4, 2020
 *      Author: Andrada Zoltan
 */

#include "PeopleTracker.h"
#include <iostream>
#include <thread>

using namespace Spinnaker;
using std::cout;
using std::thread;
using std::vector;

int main(void) {
    int err = 0;
    PeopleTracker* tracker = new PeopleTracker();

    err = tracker->InitTracker();
    if (err) {
        cout << "Error InitTracker!\n";
        return -1;
    }

    // Create acquisition thread.
    thread acqThread(&PeopleTracker::StartTracking, tracker);

    while (1) {
        cout << tracker->GetPeopleCount() << "\n";
        Sleep(500);
    }

    delete tracker;
    return 0;
}


