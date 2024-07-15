# Traxxis 1/10 Scale RC Car Autonomous Vehicle Repositoy
Demonstration code for running a small vehicle intersection with figure 8 loop using SLAM.

## Simulation:
To run the code in simulatuion mode (i.e. no real RC cars), simply run `pip install -r requirements.txt` and then `python main.py`. Enter target speeds for each of the 2 vehicles, this should be between 0 and 1 (units is meters per second) otherwise the control system will not be able to keep the vehicles on the track. Once there is a speed entered for both, click the "start test" button. The pause test will set the target speeds to 0 to temporarily pause the test. You can also update the target speeds in real time.

To run, type `python main.py`

## 1/10 Scale Vehicles:

### Preparing the Jeston Nano
Rather than re-inventing the wheel, I would suggest you follow the instructions outlined here to set up your Nvidia Jetson Nano for image processing. We will not need tensorflow, however we do require OpenCV and SkiKitLearn so make sure you get through that part. I find this guide is the best - you may be able to follow another one. https://www.pyimagesearch.com/2020/03/25/how-to-configure-your-nvidia-jetson-nano-for-computer-vision-and-deep-learning/

After installing all the necessary prerequisites including OpenCV and scikitlearn, we can now follow the instructions to install darknet:
https://jkjung-avt.github.io/yolov4/

Clone this repo on the ~/workspace/ folder.

In order to recognise the 1/10th scale vehicles you will need the retrained version of YoloV4 tiny. Be sure to download and add the following YoloV4 files into the darknet folder: 
 * [yolov4-tiny-cav.cfg](https://drive.google.com/file/d/1yMQntYWsVbJ8h7x0r0Hv5y34o2IGRVJ2/view?usp=sharing), needs to be located in /cfg
 * [cav.names](https://drive.google.com/file/d/1hP7bfu5Ei-5w1cub5-vZJ-96mx0LLVMY/view?usp=sharing), needs to be located in /data
 * [cav.data](https://drive.google.com/file/d/1jcEDFQ5n56Hq5tWJXXZ5N3Pg-p9qjq9t/view?usp=sharing), needs to be located in /data
 * [yolov4-tiny-cav.weights](https://drive.google.com/file/d/1g8r59Xcn5-n6jpc2ZjQdv-1jsLAiKN9w/view?usp=sharing), needs to be located in /weights

Now in order to run multiple vehicles you will have to modify the `routing.json` file with the respective IPs for the Connected Autonomous Vehicles (CAVs) and Connected Infrasrtucutre Sensors (CISs) and finally the Road Side Unit (RSU) that is going to act as the relay node for all of the parts. You can see it has some example IPs there already. I suggest using a separate router and setting static IPs for all devices so that this works.

Now we need to kick off the vehicles. First, place all the CAVs at their respective position specified in the config you are using for the test. Then turn them all on and wait for them to localize. Then on each of the CAVs run `startup.sh 0` where 0 is the CAV id listed in the `routing.json` file.

Finally, from the RSU (which must have a screen attached) `python main.py`. You should see the vehicles register with the RSU and begin reporting their locations. You can now hit play once all of the CAVs/CISs have connected and the test will begin.

** Retraining YOLO
If you are interested in the data that was used to build this, you can find the 416x416 pictures converted for darknet here: [Training Data](https://drive.google.com/drive/folders/1pw01WHVJSjuO1fQmrj9-hd2wD756w0JB?usp=sharing). Over time this will increase in size as we add more data.
