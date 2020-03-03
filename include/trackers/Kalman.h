#pragma once
/*
 *  Kalman.h
 *
 *  Class for tracking an InferenceBoundingBox using a Kalman filter
 *  to predict the next state of the box.
 *
 *  Created on: Feb 16, 2020
 *  Author: Andrada Zoltan
 */

#include "Tracker.h"
#include <vector>
#include <atomic>

class Kalman : public Tracker {
    public:
        Kalman(Spinnaker::InferenceBoundingBox box);

        bool isBoxMatch(Spinnaker::InferenceBoundingBox box);
        void updateTracker(Spinnaker::InferenceBoundingBox box);
        int updateTracker(void);
        bool getDir(void);

        ~Kalman();

    private:
        /* 
         * This is a 4-element vector containing the current
         * estimate of :
         *      (*x)[0] = x position in pixels
         *      (*x)[1] = y position in pixels
         *      (*x)[2] = x velocity in pixels/ms 
         *      (*x)[3] = length of box diagonal
         *
         * Note that the velocity is relative to the point (0,0), so a 
         * positive velocity means that the box is moving from right to left
         * in the frame. And if there is a negative velocity, the box is moving
         * from left to right.
         */
        std::vector<double>* x;

        // Covariance matrix (P)
        double cov[4][4];

        // Prediction matrix (F)
        const double pred[4][4] = {{1, 0, CAM_MS_PER_FRAME, 0},
                                {0, 1, 0, 0},
                                {0, 0, 1, 0},
                                {0, 0, 0, 1}};
        // Prediction matrix transpose (F_T)
        const double predTrans[4][4] = { {1, 0, 0, 0},
                                      {0, 1, 0, 0},
                                      {CAM_MS_PER_FRAME, 0, 1, 0},
                                      {0, 0, 0, 1}};

        // Filter function
        std::vector<double> Predict(void);
        void Update(std::vector<double> obsState, double obsCov[][4]);
        std::vector<double> MakeStateVector(Spinnaker::InferenceBoundingBox box);

        // Matrix utility functions
        void matMultiply(const double mat1[][4], const double mat2[][4], double res[][4]);
        void matInverse(double mat[4][4]);
        int matDeterminant(double mat[4][4], int n);
        void getCofactor(double mat[4][4], double temp[4][4], int p, int q, int n);
        void matAdjoint(double mat[4][4], double adj[4][4]);
};
