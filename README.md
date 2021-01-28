# LabNavDemo
 Demonstration code for running a small vehicle intersection with figure 8 loop using SLAM.
 
 To run the code in simulatuion mode (i.e. no real RC cars), simply run `python testrun.py`. Enter target speeds for each of the 2 vehicles, this should be between 0 and 3 (units is meters per second) otherwise the control system will not be able to keep the vehicles on the track. Once there is a speed entered for both, click the "start test" button. The pause test will set the target speeds to 0 to temporarily pause the test. You can also update the target speeds in real time.
