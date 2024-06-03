##############################
## Collect EMG data by NI   ##
##############################
#!/opt/ros/noetic/x64/python
import nidaqmx
import numpy as np

class EMG_collector:
    def __init__(self, fs, nch):
        self.fs = fs
        self.nch = nch

    def initialize(self):
        self.task = nidaqmx.task.Task()
        chp = round((self.nch - 0) / 2)
        self.task.ai_channels.add_ai_voltage_chan(("Dev2/ai0:" + str(chp - 1)))  # m1_x
        self.task.ai_channels.add_ai_voltage_chan(("Dev2/ai4:" + str(4 + chp - 1)))  # m1_y

        ######## Sampling rate: 1500Hz (Default) ########
        self.task.timing.cfg_samp_clk_timing(rate=self.fs, sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS)

    def collect_EMG(self):
            self.data = self.task.read(number_of_samples_per_channel=1)
            temp = np.array(self.data)
            emg = [0] * self.nch
            for x in range(0, self.nch):
                emg[x] = temp[x, :]
            return emg