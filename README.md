# Traxxis 1/10 Scale RC Car Autonomous Vehicle Repositoy
Demonstration code for running a small vehicle intersection with figure 8 loop using SLAM.

Get Started Fast:
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://github.com/eandert/One_Tenth_Scale_Autonomous_Vehicle/blob/devel/colab-tutorial.ipynb)

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
 * [yolov4-tiny-cav.cfg](https://drive.google.com/file/d/1yMQntYWsVbJ8h7x0r0Hv5y34o2IGRVJ2/view?usp=sharing), needs to be located in /cfg
 * [cav.names](https://drive.google.com/file/d/1hP7bfu5Ei-5w1cub5-vZJ-96mx0LLVMY/view?usp=sharing), needs to be located in /data
 * [cav.data](https://drive.google.com/file/d/1jcEDFQ5n56Hq5tWJXXZ5N3Pg-p9qjq9t/view?usp=sharing), needs to be located in /data
 * [yolov4-tiny-cav.weights](https://drive.google.com/file/d/1g8r59Xcn5-n6jpc2ZjQdv-1jsLAiKN9w/view?usp=sharing), needs to be located in /weights

If you are interested in the data that was used to build this, you can find the 416x416 pictures converted for darknet here: [Training Data](https://drive.google.com/drive/folders/1pw01WHVJSjuO1fQmrj9-hd2wD756w0JB?usp=sharing). Over time this will increase in size as we add more data.

To run, type `python main.py`
