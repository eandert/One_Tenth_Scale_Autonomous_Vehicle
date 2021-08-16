# Traxxis RC Car Control Master Node
Demonstration code for running a small vehicle intersection with figure 8 loop using SLAM.

Simulation:
To run the code in simulatuion mode (i.e. no real RC cars), simply run `python main.py`. Enter target speeds for each of the 2 vehicles, this should be between 0 and 3 (units is meters per second) otherwise the control system will not be able to keep the vehicles on the track. Once there is a speed entered for both, click the "start test" button. The pause test will set the target speeds to 0 to temporarily pause the test. You can also update the target speeds in real time.

1/10 Scale Vehicles:
To Install on physical Jetson hardware:
Download the SD card image and upload to 64 GB SD card. This will save days of building the correct libraries, etc. 
TODO: Place google drive link here for Jetseon TX2
Clone this repo on the Home/Projects/ folder.
TODO: Place google drive link here for Jetson Nano
Clone this repo on the Home/Projects/ folder.

In order to recognise the 1/10th scale vehicles you will need the retrained version of YoloV4 tiny. Be sure to download and add the following YoloV4 files into the darknet folder:

yolov4-tiny-cav.cfg, needs to be located in /cfg
cav.names, needs to be located in /data
cav.data, needs to be located in /data
yolov4-tiny-cav.weights, needs to be located in /weights
If you are interested in the data that was used to build this, you can find the 416x416 pictures converted for darknet here: Training Data. Over time this will increase in size as we add more data.

To run, type python main.py
