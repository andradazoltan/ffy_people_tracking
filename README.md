# HikerCam

This project uses a [Firefly Deep Learning Camera](https://www.flir.ca/products/firefly-dl/) to perform people counting as people pass into and out of the frame. The initial application for this project was a device mounted at the head of hiking trails to track populations on the trail throughout the day, hence the name 'HikerCam', however it can be applied to any scenario. 

## General Design
The design has two major components that work together to count people:
* **CNN People Detection:** A MobileNet SSD network is loaded onto the camera and executes as the camera streams images. With every frame, the latest inference result is outputted in the form of bounding boxes surrounding any people in the frame. Other classes of bounding boxes may also be outputted, but are ignored in this application. A pre-trained network was used, and can be found [here](https://www.flir.ca/support-center/iis/machine-vision/application-note/neural-networks-supported-by-the-firefly-dl/) as the first option under Tested and Supported CNNs. 

* **Bounding Box Tracker:** Custom algorithm written to track bounding boxes as they travel throughout the frame and determine when the bounding boxes are no longer visible. The attempts at implmenting such a solution are outlined below.

## Tracking Solutions
* **Position Tracking:** The first attempt at tracking involved using only the position of the box. If the position between frames moved more than a certain threshold, then the box was deemed to be from a different person in the frame. However, this solution was much too simplistic and does not work for all speeds of people moving through the frame.

* **Kalman Filter:** The second attempt involved implementing a Kalman filter to predict the next state of each box and use the predicted state to determine if the observed state corresponded to the same box. The state of each box was represented by: position of the box, velocity of the box, size of the box. Although this solution worked better, there were still issues with it working consistently. One challenge was determining how to appropriately initialize the filter with a starting state vector, since the velocity varied depending on what proportion of the person was in the frame in the first reading. As well, the filter only underwent several iterations of updates before a person left the frame, so there were not enough readings to improve the accuracy of the filter before it was not needed anymore.

* **State Tracking:** The third attempt involved somewhat of a combination of the first two solutions. Rather than just using the position to differentiate between boxes, more variables were added to the state of a box. Similar to the Kalman filter, the position, velocity and size of the box were used. However in this solution, the filter was removed and instead replaced with a difference threshold between two readings. This solution worked consistenly with only one person passing through the frame. This implementation has not yet been tested with multiple people, so it is likely that the thresholds are too loose for such a scenario.

## Resources
* [People Counter Using OpenCV and dlib](https://www.pyimagesearch.com/2018/08/13/opencv-people-counter/)
* [Kalman Filter](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)
