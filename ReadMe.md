File Description:
This folder contains both old and new versions of code, currently using the new version of code
LaneDetectionAndOccupancy Mapubirdview_1 is the Chinese version
LaneDetectionAndOccupancy Mapubirdview_1_1 is the English version
In this code, the camera can recognize roads in real environments, including
Total Frame Processing Time
Receive
Bilateral Fitering
Threshold
Morphological
Average Brightness
Dynamic Threshold
These outputs can ultimately obtain the current ambient brightness, the total time for the camera to recognize a single frame, and the dynamic balance achieved



Responsibilities:
Before writing the code, Yicong Ren and I conducted simulations by writing the code separately, and finally synthesized the basic framework of the code
In this code, I am responsible for measuring the real-world application of the camera and finally obtaining the table



| ***\*HDR\****    |                              |                  |                              |
| ---------------- | ---------------------------- | ---------------- | ---------------------------- |
| ***\*Dlight\**** | ***\*Range of Threshold\**** | ***\*Dlight\**** | ***\*Range of Threshold\**** |
| 138              | 155                          | 57               | 68                           |
| 134              | 146                          | 51               | 61                           |
| 129              | 142                          | 47               | 57                           |
| 123              | 137                          | 43               | 53                           |
| 116              | 130                          | 39               | 48                           |
| 108              | 118                          | 35               | 42                           |
| 102              | 110                          | 31               | 36                           |
| 94               | 103                          | 27               | 31                           |
| 77               | 90                           | 23               | 26                           |
| 72               | 84                           | 19               | 22                           |
| 68               | 79                           | 16               | 19                           |
| 64               | 75                           | 13               | 17                           |

greythreshold = -0.0008 * avgBrightness^2 + 1.2008 * avgBrightness + 0.7402;