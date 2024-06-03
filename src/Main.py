#!/opt/ros/noetic/x64/python
from PyQt5 import QtWidgets, QtCore
import sys  # We need sys so that we can pass argv to QApplication
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal
from queue import Queue
from EMGThread import EMGThread
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
import math

class M1Thread(QThread):
    signal = pyqtSignal('PyQt_PyObject')

    def __init__(self, in_EMG, fs, nch):
        QThread.__init__(self)
        print('Initializing the M1 thread')
        #### Use the M1 thread input ####
        self.InEMG = in_EMG         # Get the raw EMG from queue

        #### Other Init ####
        self.timenow = 0                                             # Flag for time init
        self.fs = fs                                                 # Sampling frequency
        self.status = 1                                              # Flag for start/stop status init
        self.nch = nch                                               # Number of analog channels to read
        self.EMGPrev = [0] * self.nch                                # Vector of previous EMG values
        self.EMGFiltHist = [[0] * round(0.5 * self.fs) for _ in range(self.nch)]  # 0.5-second vector of EMG values
        self.alpha = (2*math.pi*2/self.fs)/(2*math.pi*2/self.fs+1)   # 2 Hz lowpass filter, rectified

        #### Set up ROS node ####
        rospy.init_node('reader_node')
        self.pub = []
        if rospy.has_param('reader_node/name_spaces'):
            self.ns = rospy.get_param('reader_node/name_spaces')
            print(self.ns)

        self.chp = round(self.nch / len(self.ns))
        self.mvcs = [[0] * self.chp] * len(self.ns)
        self.mves = [[0] * self.chp] * len(self.ns)

        for n in range(0, len(self.ns)):
            self.mvcs[n] = rospy.get_param('reader_node/' + self.ns[n] + '/mvc')
            self.mves[n] = rospy.get_param('reader_node/' + self.ns[n] + '/mve')
            self.pub.append(rospy.Publisher(self.ns[n] + '/emg_data', Float64MultiArray, queue_size=10))

        self.rate = rospy.Rate(self.fs)  # 400hz

    def stopM1(self):
        self.status = 1
        print('Stop')

    def startM1(self):
        self.status = 2
        print('Start')

    # run method gets called when we start the thread
    def run(self):
        while not rospy.is_shutdown():
            if self.status == 2:
                ########################
                #### EMG processing ####
                ########################
                #### 1. Get raw EMG data & publish ####
                EMGRaw = [0] * self.nch
                for x in range(0, self.nch):
                    EMGRaw[x] = self.InEMG.get()
                    self.EMGFiltHist[x].append(abs(EMGRaw[x]))  # 1-sec moving average
                    self.EMGFiltHist[x].pop(0)

                for n in range(0, len(self.pub)):
                    # RAW EMG
                    EMGMA = Float64MultiArray()  # emg values
                    EMGMA.layout.dim.append(MultiArrayDimension())
                    EMGMA.layout.dim[0].label = "emg"
                    EMGMA.layout.dim[0].size = self.chp
                    DAT = EMGRaw[n * self.chp:(n + 1) * self.chp]

                    # FILTERED EMG
                    EMGMA.layout.dim.append(MultiArrayDimension())
                    EMGMA.layout.dim[1].label = "emg_filtered"
                    EMGMA.layout.dim[1].size = self.chp
                    TEMPDAT = self.EMGFiltHist[n * self.chp:(n + 1) * self.chp]  # filtered EMG
                    for m in range(0, self.chp):
                        # filtered EMG
                        DAT.append(sum(TEMPDAT[m])/len(TEMPDAT[m]))

                    # COACTIVATION
                    # EMGMA.layout.dim.append(MultiArrayDimension())
                    # EMGMA.layout.dim[1].label = "coact"
                    # EMGMA.layout.dim[1].size = self.chp - 1
                    # TEMPDAT = self.EMGFiltHist[n * self.chp:(n + 1) * self.chp]
                    # for m in range(1, self.chp):
                    #     # coactivation between extensor and each flexor
                    #     COACT = min([self.mvcs[n][0]*(sum(TEMPDAT[0])/len(TEMPDAT[0]))/self.mves[n][0],
                    #                  self.mvcs[n][m]*(sum(TEMPDAT[m])/len(TEMPDAT[m]))/self.mves[n][m]])
                    #     DAT.append(COACT)

                    # Publish raw values
                    EMGMA.data = DAT
                    self.pub[n].publish(EMGMA)

                #### Update Time ####
                self.timenow = self.timenow + 1/self.fs

                # self.rate.sleep()

        print("Exit run")


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        self.setGeometry(500, 100, 200, 200)
        self.plot = False
        self.fs = 400
        self.nch = 8  # should be even number for designed plot colors (max is 8; 4x2 emg)

        ###########################
        ######  GUI Design   ######
        ###########################

        ####### Line 1 ########
        self.btn_connect = QtWidgets.QPushButton('Connect')
        self.btn_connect.clicked.connect(self.connectemg)

        self.btn_start = QtWidgets.QPushButton('Start')
        self.btn_start.clicked.connect(self.start)
        self.btn_start.setDisabled(True)

        self.btn_Exit = QtWidgets.QPushButton('Exit')
        self.btn_Exit.clicked.connect(self.Exit)
        self.btn_Exit.setDisabled(True)

        self.lh1 = QtWidgets.QHBoxLayout()
        self.lh1.addWidget(self.btn_connect)
        self.lh1.addWidget(self.btn_start)
        self.lh1.addWidget(self.btn_Exit)
        self.h_wid1 = QtWidgets.QWidget()
        self.h_wid1.setLayout(self.lh1)

        # create layout
        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.h_wid1)

        # add layout to widget and display
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        #### Status Init ####
        self.status = 0  # Connection status, 0 = Not Connected, 1 = Connected and start, 2 = Connected and stop

        #### Queue Init ####
        # Use Queue to send data from EMG thread to M1 thread. MainWindow (GUI thread) to coordinate.
        self.EMGQ = Queue()    # Raw EMG (2 x number of emg channels)

    ##########################################################
    #### Line1: Functions about connection and start/exit ####
    ##########################################################
    def connectemg(self):
        if self.status == 0:
            self.status = 1

            #### EMG connection ####
            try:
                self.EMGthread = EMGThread(self.EMGQ, self.fs, self.nch)
                print("EMG Connected!")
            except:
                print('No EMG Connection! Please check and restart.')    # If No EMG connection, the system will not work.

            #### M1 thread connection ####
            self.m1thread = M1Thread(self.EMGQ, self.fs, self.nch)

            #### Button activation ####
            self.btn_connect.setDisabled(True)
            self.btn_start.setDisabled(False)
            self.btn_Exit.setDisabled(False)

            print("Connect emg!")

    def start(self):
        if self.status == 1:
            #### Start M1 and EMG thread ####
            try:
                self.EMGthread.start()
            except:
                pass
            self.m1thread.start()
            #### Change button, press the same button again can stop ####
            self.btn_connect.setDisabled(True)
            self.btn_start.setText("Stop")
            self.status = 2
            self.m1thread.startM1()  # Update status after selection

        elif self.status == 2:
            #### Change back to start status
            self.status = 1
            self.m1thread.stopM1()
            self.btn_connect.setDisabled(True)
            self.btn_start.setText("Start")

    def Exit(self):
        print("Exit run")
        self.close()

def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()