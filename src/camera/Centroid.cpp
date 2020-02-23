#include "Centroid.h"
#include <iostream>

#define MISSING_THRESH 5
#define DIST_X_THRESH 200
#define DIST_Y_THRESH 20
#define VEL_X_THRESH  2
#define BOX_SIZE_THRESH 100

using namespace Spinnaker;

using std::cout;
using std::vector;

Centroid::Centroid(InferenceBoundingBox box) {
    count = 0;
    state = new vector<double>(4);
    *state = MakeStateVector(box);

    cout << "NEW: ";
    for (int i = 0; i < 4; i++)
        cout << (*state)[i] << " ";
    cout << "\n";
}

/*
 * Determines if the provided box matches the current filter
 * by comparing it to the predicted state.
 */
bool Centroid::IsBoxMatch(InferenceBoundingBox box) {
    vector<double> obs = MakeStateVector(box);
    bool ret = true;

    if (abs(obs[0] - (*state)[0]) > DIST_X_THRESH&&
        abs(obs[1] - (*state)[1]) > DIST_Y_THRESH&&
        abs(obs[2] - (*state)[2]) > VEL_X_THRESH&&
        abs(obs[3] - (*state)[3]) > BOX_SIZE_THRESH)
        ret = false;
    cout << "\n";
    for (int i = 0; i < 4; i++) {
        cout << i << " o:" << obs[i] << " p:" << (*state)[i] << " ";
    }

    return ret;
}

bool Centroid::getDir(void) {
    return ((*state)[2] > 0);
}

void Centroid::updateCentroid(InferenceBoundingBox box) {
    // Reset missing counter
    count = 0;

    // Update the state vector
    *state = MakeStateVector(box);
}

int Centroid::updateCentroid() {
    count++;
    if (count > MISSING_THRESH)
        return -1;
    else
        return 0;
}

Centroid::~Centroid() {
    delete state;
}

/*
 * Takes in a bounding box and creates a state vector that
 * represents the state of the system at this point.
 */
vector<double> Centroid::MakeStateVector(InferenceBoundingBox box) {
    vector<double> ret;
    ret.push_back((box.rect.bottomRightXCoord + box.rect.topLeftXCoord) / 2); // X position
    ret.push_back((box.rect.bottomRightYCoord + box.rect.topLeftYCoord) / 2); // Y position

    // If a previous x-position exists, use it to calcualte the 
    // current velocity.
    if ((*state)[0] > 0) {
        if ((ret[0] - (*state)[0]) != 0)
            ret.push_back((ret[0] - (*state)[0]) / INFERENCE_TIME);
        else
            ret.push_back((*state)[2]);
    }
    else {
        // X velocity should be negative if person is moving from left to right
        if (ret[0] > CAM_X / 2)
            ret.push_back((ret[0] - CAM_X) / INFERENCE_TIME);
        else
            ret.push_back(ret[0] / INFERENCE_TIME);
    }

    // Length of box diagonal
    ret.push_back(sqrt(pow(box.rect.bottomRightXCoord - box.rect.topLeftXCoord, 2) +
        pow(box.rect.bottomRightYCoord - box.rect.topLeftYCoord, 2)));

    return ret;
}