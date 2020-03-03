/*
 *  Kalman.cpp
 *
 *  Created on: Feb 16, 2020
 *  Author: Andrada Zoltan
 */

#include "Kalman.h"
#include <iostream>
#include <thread>

using namespace Spinnaker;

using std::cout;
using std::vector;
using std::thread;

#define DIST_THRESH 200

Kalman::Kalman(InferenceBoundingBox box) {
    // Initialize starting vector
    x = new vector<double>(4);
    *x = this->MakeStateVector(box);

    // Initialize covariance matrix
    double covTemp[][4] = {{1, 0, 2, 0},
                        {0, 1, 0, 0},
                        {2, 0, 20, 0},
                        {0, 0, 0, 4}};
    memcpy(cov, covTemp, sizeof(covTemp));

    count = 0;
}

/*
 * Determines if the provided box matches the current filter
 * by comparing it to the predicted state.
 */
bool Kalman::isBoxMatch(InferenceBoundingBox box) {
    // Predict the state vector
    this->Predict();
    
    // Compare the predicted vector with the observed vector
    vector<double> obs = this->MakeStateVector(box);

    double squaredSum = 0;
    for (int i = 0; i < 4; i++) {
        squaredSum += (pow(obs[i] - (*x)[i], 2));
    }
    double dist = sqrt(squaredSum);

    return (dist < DIST_THRESH);
}

/*
 * Update the current estimate with a new bounding box measurement.
 * Creates a state vector for the observed box and calls the full
 * update function.
 */
void Kalman::updateTracker(InferenceBoundingBox box) {
    // Reset missing counter
    count = 0;

    vector<double> sv = this->MakeStateVector(box);
    double obsCov[][4] = { {1, 0, 0, 0},
                        {0, 1, 0, 0},
                        {0, 0, 10, 0},
                        {0, 0, 0, 2} };

    this->Update(sv, obsCov);
}

int Kalman::updateTracker(void) {
    return (Tracker::updateTracker());
}

bool Kalman::getDir(void) {
    return ((*x)[2] > 0);
}

Kalman::~Kalman() {
    delete x;
}

/************************ Private Functions ****************************/
/* 
 * Predict the next state vector and covariance matrix using
 * the prediction matrix, and update the current state with
 * the new estimate.
 *
 * x(k) = F * x(k-1)
 * P(k) = F * P(k-1) * F_T
 *
 * Returns the predicted state vector.
 */
vector<double> Kalman::Predict(void) {
    vector<double> state;
    
    // Find predicted state vector
    for (int i = 0; i < 4; i++) {
        double val = 0;

        for (int j = 0; j < 4; j++) {
            val += (*x)[j] * pred[i][j]; // x(k) = F * x(k-1)
            
        }
        state.push_back(val);
    }
    *x = state;

    // Find predicted covariance matrix
    double tempMat[4][4] = { 0 };
    matMultiply(cov, pred, tempMat);      // F * P(k-1)
    matMultiply(tempMat, predTrans, cov); // P(k) = F * P(k-1) * F_T

    return state;
}

/* 
 * Update the current estimate with a new observed state.
 *
 * x(K) = x(k) + K * (z(k) - F * x(k))
 * P(K) = P(k) - K * F * P(k)
 *
 * Where K = P(k) * F_T * (F * P(k) * F_T + R(k))^-1
 */
void Kalman::Update(std::vector<double> obsState /* z(k) */, double obsCov[][4] /* R(k) */) {
    // Calculate the kalman game
    double gain[4][4] = { 0 };

    double tempMat[4][4] = { 0 };
    matMultiply(pred, cov, tempMat);          // F * P(k)
    matMultiply(tempMat, predTrans, tempMat); // F * P(k) * F_T

    // F * P(k) * F_T + R(k)
    /*for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            tempMat[i][j] += obsCov[i][j];
        }
    }*/

    // (F * P(k) * F_T + R(k))^-1
    matInverse(tempMat);

    // P(k) * F_T * (F * P(k) * F_T + R(k))^-1
    matMultiply(cov, predTrans, gain);
    matMultiply(gain, tempMat, gain);


    // Calculate the updated state vector
    for (int i = 0; i < 4; i++) {
        double subVal = 0;
        for (int j = 0; j < 4; j++) {
            subVal += pred[i][j] * (*x)[j];
        }

        obsState[i] -= subVal;
    }

    for (int i = 0; i < 4; i++) {
        double addVal = 0;
        for (int j = 0; j < 4; j++) {
            addVal += gain[i][j] * obsState[i];
        }

        (*x)[i] += addVal;
    }

    // Calculate the updated covariance matrix
    matMultiply(gain, pred, tempMat);
    matMultiply(tempMat, cov, tempMat);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            cov[i][j] -= tempMat[i][j];
        }
    }
}

/* 
 * Takes in a bounding box and creates a state vector that 
 * represents the state of the system at this point.
 */
vector<double> Kalman::MakeStateVector(InferenceBoundingBox box) {
    vector<double> ret;
    ret.push_back((box.rect.bottomRightXCoord + box.rect.topLeftXCoord) / 2); // X position
    ret.push_back((box.rect.bottomRightYCoord + box.rect.topLeftYCoord) / 2); // Y position

    // If a previous x-position exists, use it to calcualte the 
    // current velocity.
    if ((*x)[2] > 0) {
        ret.push_back((ret[0] - (*x)[0]) / INFERENCE_TIME);
    }
    else {
        // X velocity should be negative if person is moving from left to right
        if (ret[0] > CAM_X / 2)
            ret.push_back((ret[0] - CAM_X) / CAM_MS_PER_FRAME);
        else
            ret.push_back(ret[0] / CAM_MS_PER_FRAME);
    }

    // Length of box diagonal
    ret.push_back(sqrt(pow(box.rect.bottomRightXCoord - box.rect.topLeftXCoord, 2) +
        pow(box.rect.bottomRightYCoord - box.rect.topLeftYCoord, 2)));

    return ret;
}

/*
 * Matrix multiplication for 4x4 matrices.
 *
 * TODO: Hardware accelerate this
 */
void Kalman::matMultiply(const double mat1[][4], const double mat2[][4], double res[][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            res[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                res[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

/* 
 * Matrix inverse for 4x4 matrices.
 */
void Kalman::matInverse(double mat[4][4]) {
    double det = matDeterminant(mat, 4);
    double adj[4][4];
    matAdjoint(mat, adj);

    if (det == 0)
        det = 1;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            mat[i][j] = adj[i][j] / det;
        }
    }
}

/*
 * Matrix adjoint for 4x4 matrices.
 */
int Kalman::matDeterminant(double mat[4][4], int n) {
    double det = 0;
    double temp[4][4]; // Cofactors
    int sign = 1;

    for (int i = 0; i < n; i++) {
        getCofactor(mat, temp, 0, i, n);
        det += sign * mat[0][i] * matDeterminant(temp, n - 1);

        sign = -sign;
    }

    return (int)det;
}

/*
 * Matrix cofactor for 4x4 matrices.
 */
void Kalman::getCofactor(double mat[4][4], double temp[4][4], int p, int q, int n) {
    int i = 0;
    int j = 0;

    for (int row = 0; row < n; row++) {
        for (int col = 0; col < n; col++) {
            if (row != p && col != q) {
                temp[i][j++] = mat[row][col];

                if (j == (n - 1)) {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

/*
 * Matrix adjoint for 4x4 matrices.
 */
void Kalman::matAdjoint(double mat[4][4], double adj[4][4]) {
    int sign = 1;
    double temp[4][4];

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            getCofactor(mat, temp, i, j, 4);

            // Sign of adj[j][i] positive if sum of row 
            // and column indexes is even. 
            sign = ((i + j) % 2 == 0) ? 1 : -1;

            // Interchanging rows and columns to get the 
            // transpose of the cofactor matrix 
            adj[j][i] = sign * (matDeterminant(temp, 3));
        }
    }
}