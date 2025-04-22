Note that this is an overall description of the section I have submitted

Personal Content refers to the content completed by individuals, while Group Content refers to the content completed by the entire Camera team

If you want to run the final result, you need to
1. Start and run in VM ware virtual machine environment

2. Enter Matlab, start Matlab and open the file
    LaneDetectionAndOccupancy Mapubirdview_1 (Chinese version) or
    LaneDetectionAndOccupancy Mapubirdview_1_1 （English version）

3. Enter ros2 launch multisense_ros multisense_1aunch.by to link the camera, ensuring that the camera is at a height of 1.2m and a top view angle of 30 degrees. This is the actual state that the camera should be in during use.

4. Enter ros2 run rqt_deconfigure rqt_deconfigure to open the settings and proceed with the setup until it meets the basic usage requirements

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

If successful, one can see

Total Frame Processing Time
Receive
Bilateral Fitering
Threshold
Morphological
Average Brightness
Dynamic Threshold

The data and corresponding images of the above items