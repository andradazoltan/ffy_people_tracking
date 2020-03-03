/*
 *  HikerCam.cpp
 *
 *  Created on: Feb 4, 2020
 *  Author: Andrada Zoltan
 */

#include "HikerCam.h"
#include <iostream>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

using std::cout;
using std::vector;
using std::mutex;

HikerCam::HikerCam() : mSystem(NULL), mCamera(NULL), endAcquistionSignal(false) {
    bufferMutex = new mutex();
    boundingBoxBuffer = new vector<InferenceBoundingBox>();
}

int HikerCam::InitCamera(void) {
    // Get the current system
    mSystem = System::GetInstance();

    // Get list of cameras connected to the system
    CameraList camList = mSystem->GetCameras();
    unsigned numCameras = camList.GetSize();

    // Check if any cameras are connected
    if (numCameras == 0)
    {
        camList.Clear();
        mSystem->ReleaseInstance();

        cout << "No cameras connected!\n";
        return -1;
    }

    // Get the connected camera, assume there is only one connected
    mCamera = camList.GetByIndex(0);
    if (mCamera == NULL)
        return -1;

    try {
        // Initalize the camera
        mCamera->Init();

        // Get the nodemap
        INodeMap& mNodeMap = mCamera->GetNodeMap();
        
        // Enable inference settings
        if (EnableInference(mNodeMap))
            return -1;
    }
    catch (Spinnaker::Exception& e) {
        cout << "Spinnaker exception caught: " << e.GetErrorMessage() << ".\n";
        return -1;
    }

    return 0;
}

int HikerCam::StartAcquisition(void) {
    endAcquistionSignal.store(false);

    try {
        // Get the nodemap
        INodeMap& mNodeMap = mCamera->GetNodeMap();

        // Start Acquisition
        mCamera->BeginAcquisition();

        while (!endAcquistionSignal) {
            ImagePtr img = mCamera->GetNextImage();
            
            if (img->IsIncomplete()) {
                cout << "Image is incomplete: " << img->GetImageStatus() << ".\n";
            }
            else {
                // Get chunk data
                ChunkData chunkData = img->GetChunkData();

                // Save the current bounding box chunk data
                InferenceBoundingBoxResult boundingBoxData = chunkData.GetInferenceBoundingBoxResult();

                // Lock the mutex
                bufferMutex->lock();

                // Clear the bounding box buffer
                boundingBoxBuffer->clear();

                // Push new bounding boxes to the buffer
                int numBoxes = boundingBoxData.GetBoxCount();
                for (int i = 0; i < numBoxes; i++) {
                    boundingBoxBuffer->push_back(boundingBoxData.GetBoxAt(i));
                }

                // Unlock the mutex
                bufferMutex->unlock();
            }
        }

        mCamera->EndAcquisition();
    }
    catch (Spinnaker::Exception & e) {
        cout << "Spinnaker exception caught: " << e.GetErrorMessage() << ".\n";
        return -1;
    }

    return 0;
}

void HikerCam::EndAcquisition(void) {
    endAcquistionSignal.store(true);
}

 void HikerCam::GetBoundingBoxData(vector<InferenceBoundingBox>& buf) {
     // Clear the buffer
     buf.clear();

     // Lock the mutex
     bufferMutex->lock();

     // Copy the shared buffer
     buf = *boundingBoxBuffer;

     // Unlock the mutex
     bufferMutex->unlock();
}

HikerCam::~HikerCam() {
    // Clear the camera list
    CameraList camList = mSystem->GetCameras();
    camList.Clear();
    mCamera = NULL;

    // Release the system
    mSystem->ReleaseInstance();
    mSystem = NULL;

    // Delete bounding box buffer and mutex
    delete bufferMutex;
    delete boundingBoxBuffer;
}

/************************** Private Functions **************************/

int HikerCam::EnableInference(INodeMap& nodeMap) {
    // Enable Inference
    CBooleanPtr inferenceEnable = nodeMap.GetNode("InferenceEnable");
    if (!IsAvailable(inferenceEnable) || !IsWritable(inferenceEnable)) {
        cout << "InferenceEnable is not available or writable.\n";
        return -1;
    }
    else {
        inferenceEnable->SetValue(true);
    }

    // Change InferenceNetworkTypeSelector to Detection
    CEnumerationPtr networkType = nodeMap.GetNode("InferenceNetworkTypeSelector");
    if (!IsAvailable(networkType) || !IsWritable(networkType)) {
        cout << "InferenceNetworkTypeSelector is not available or writable.\n";
        return -1;
    }
    else {
        CEnumEntryPtr detectionNetwork = networkType->GetEntryByName("Detection");
        if (!IsAvailable(detectionNetwork)) {
            cout << "Detection is not a valid enum entry.\n";
            return -1;
        }

        networkType->SetIntValue(detectionNetwork->GetValue());
    }

    // Set BoundingBoxThreshold to 0.60
    CFloatPtr boundingBoxThreshold = nodeMap.GetNode("InferenceBoundingBoxThreshold");
    if (!IsAvailable(boundingBoxThreshold) || !IsWritable(boundingBoxThreshold)) {
        cout << "InferenceBoundingBoxThreshold is not available or writable.\n";
        return -1;
    }
    else {
        boundingBoxThreshold->SetValue(0.60);
    }

    // Set TriggerSelector to FrameStart
    CEnumerationPtr triggerSelector = nodeMap.GetNode("TriggerSelector");
    if (!IsAvailable(triggerSelector) || !IsWritable(triggerSelector)) {
        cout << "TriggerSelector is not available or writable.\n";
        return -1;
    }
    else {
        CEnumEntryPtr frameStart = triggerSelector->GetEntryByName("FrameStart");
        if (!IsAvailable(frameStart)) {
            cout << "FrameStart is not a valid enum entry.\n";
            return -1;
        }

        triggerSelector->SetIntValue(frameStart->GetValue());
    }

    // Set TriggerMode to On
    CEnumerationPtr triggerMode = nodeMap.GetNode("TriggerMode");
    if (!IsAvailable(triggerMode) || !IsWritable(triggerMode)) {
        cout << "TriggerMode is not available or writable.\n";
        return -1;
    }
    else {
        CEnumEntryPtr onMode = triggerMode->GetEntryByName("On");
        if (!IsAvailable(onMode)) {
            cout << "On is not a valid enum entry.\n";
            return -1;
        }

        triggerMode->SetIntValue(onMode->GetValue());
    }

    // Set TriggerSource to InferenceReady
    CEnumerationPtr triggerSource = nodeMap.GetNode("TriggerSource");
    if (!IsAvailable(triggerSource) || !IsWritable(triggerSource)) {
        cout << "TriggerSource is not available or writable.\n";
        return -1;
    }
    else {
        CEnumEntryPtr infReady = triggerSource->GetEntryByName("InferenceReady");
        if (!IsAvailable(infReady)) {
            cout << "InferenceReady is not a valid enum entry.\n";
            return -1;
        }

        triggerSource->SetIntValue(infReady->GetValue());
    }

    return 0;
}



