import vehicle_demo
import sys

global mainWin

# Initialize a simulator with the GUI set to on, cm map, .1s timestep, and vehicle spawn scale of 1 (default)
QTapp = vehicle_demo.QtWidgets.QApplication(sys.argv)
mainWin = vehicle_demo.MainWindow()
mainWin.show()

sys.exit(QTapp.exec_())