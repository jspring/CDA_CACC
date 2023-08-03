import numpy as np
import socket, time
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.widgets import Slider
import matplotlib.lines as lines
from matplotlib.collections import PatchCollection



ACCORD = 0
PRIUS = 1

TAURUS = 2

CRUISE = 0
ACC = 1
CACC = 2
CRUISE_2_ACC = 4
ACC_2_CRUISE = 5
CACC_2_ACC = 6
ACC_2_CACC = 7

####### THESE PARAMETERS HAVE TO BE DEFINED BEFORE RUNNING
ego_veh_id = TAURUS
setPointSpeedValue = 68.24  # In mph
selectedTimeGap = 1.35  # In sec
desired_control_mode = ACC
performance_factor = 0.5   # 0: Comfort     1: Performance
start_flag = True  # When False, forces to hold control system for time syncing
########

gap_choice = 3
state_change, gap_change = False, False
sync_requested, start_requested = False, False

class buttonCallback(object):
    def Cruise(self, event):
        print("Cruise")
    def ACC(self, event):
        global desired_control_mode
        desired_control_mode = ACC
        print("Requesting ACC")
    def CACC(self, event):
        global desired_control_mode
        desired_control_mode = CACC
        print("Requesting CACC")
    def Sync(self, event):
        global sync_requested
        sync_requested = True
    def Start(self, event):
        global start_requested
        start_requested = True


def sliderCallbackSetSpeed(value):
    global setPointSpeedValue
    setPointSpeedValue = value


def sliderCallbackTimeGap(value):
    global selectedTimeGap
    selectedTimeGap = value

def sliderCallbackPerformance(value):
    global performance_factor
    performance_factor = value


def setupSliders():
    sSlider = plt.axes([0.65, 0.92, 0.28, 0.05])
    hSlider = plt.axes([0.65, 0.84, 0.28, 0.05])
    pSlider = plt.axes([0.65, 0.76, 0.28, 0.05])
    s_slider = Slider(sSlider, 'Cruise speed', 25, 75, valinit=setPointSpeedValue)
    h_slider = Slider(hSlider, 'Time gap', 0.3, 1.7, valinit=selectedTimeGap)
    p_slider = Slider(pSlider, 'Perf. factor', 0.0, 1.0, valinit=performance_factor)
    return s_slider, h_slider, p_slider


def setupButtons():
    bCallback = buttonCallback()
    x0 = 0.28
    y0 = 0.7
    width = 0.12
    height = 0.12

    CC_im = plt.imread('./img/Cruise.png')
    ACC_im = plt.imread('./img/ACC.png')
    CACC_im = plt.imread('./img/CACC.png')
    Sync_im = plt.imread('./img/Sync.png')
    Start_im = plt.imread('./img/Start.png')

    CruiseAx = plt.axes([x0 - 2 * width, y0, width, height])
    CruiseAx.axis('off')
    bCruise = Button(CruiseAx, '', image=CC_im, color='white')
    bCruise.on_clicked(bCallback.Cruise)

    ACCAx = plt.axes([x0 - width / 2, y0, width, height])
    ACCAx.axis('off')
    bACC = Button(ACCAx, '', image=ACC_im, color='white')
    bACC.on_clicked(bCallback.ACC)

    CACCAx = plt.axes([x0 + width, y0, width, height])
    CACCAx.axis('off')
    bCACC = Button(CACCAx, '', image=CACC_im, color='white')
    bCACC.on_clicked(bCallback.CACC)

    SyncAx = plt.axes([x0 - 3* width / 2, y0 + 1.0*height, width*0.8, height*0.8])
    SyncAx.axis('off')
    bSyncyng = Button(SyncAx, '', image=Sync_im, color='white')
    bSyncyng.on_clicked(bCallback.Sync)

    StartAx = plt.axes([x0 - 0.2 * width, y0 + 1.0*height, width*0.8, height*0.8])
    StartAx.axis('off')
    bStart = Button(StartAx, '', image=Start_im, color='white')
    bStart.on_clicked(bCallback.Start)

    return bCruise, CruiseAx, bACC, ACCAx, bCACC, CACCAx, bSyncyng, SyncAx, bStart, StartAx


def setupFixedImages():
    lanes_ax = plt.axes([0.63, 0.45, 0.28, 0.25], frameon=True)
    lanes_ax.imshow(plt.imread('./img/lanes.png'))
    lanes_ax.axis('off')

    targetV_far_ax = plt.axes([0.719, 0.575, 0.1, 0.08], frameon=True)
    targetV_far_ax.imshow(plt.imread('./img/target.png'))
    targetV_far_ax.axis('off')

    targetV_ax = plt.axes([0.652, 0.5201, 0.23, 0.15], frameon=True)
    targetV_ax.imshow(plt.imread('./img/ACC_target.png'))
    targetV_ax.axis('off')

    targetVCACC_ax = plt.axes([0.652, 0.52, 0.23, 0.15], frameon=True)
    targetVCACC_ax.imshow(plt.imread('./img/CACC_target.png'))
    targetVCACC_ax.axis('off')

    x0 = 0.35
    y0 = 0.7
    width = 0.05
    height = 0.05

    Cruise_2_ACC_ax = plt.axes([0.165, 0.74, width, height], frameon=True)
    Cruise_2_ACC_ax.imshow(plt.imread('./img/right_arrow.png'))
    Cruise_2_ACC_ax.axis('off')

    ACC_2_Cruise_ax = plt.axes([0.165, 0.7401, width, height], frameon=True)
    ACC_2_Cruise_ax.imshow(plt.imread('./img/left_arrow.png'))
    ACC_2_Cruise_ax.axis('off')

    ACC_2_CACC_ax = plt.axes([0.3515, 0.74, width, height], frameon=True)
    ACC_2_CACC_ax.imshow(plt.imread('./img/right_arrow.png'))
    ACC_2_CACC_ax.axis('off')

    CACC_2_ACC_ax = plt.axes([0.3515, 0.7401, width, height], frameon=True)
    CACC_2_ACC_ax.imshow(plt.imread('./img/left_arrow.png'))
    CACC_2_ACC_ax.axis('off')

    v2v_ax = plt.axes([0.42, 0.83, 0.08, 0.08], frameon=True)
    v2v_ax.imshow(plt.imread('./img/v2v_av.png'))
    v2v_ax.axis('off')

    Cruise_2_ACC_ax.set_visible(False)
    ACC_2_Cruise_ax.set_visible(False)
    ACC_2_CACC_ax.set_visible(False)
    CACC_2_ACC_ax.set_visible(False)

    targetV_far_ax.set_visible(False)
    targetV_ax.set_visible(False)
    targetVCACC_ax.set_visible(False)

    v2v_ax.set_visible(False)

    return targetV_ax, targetV_far_ax, targetVCACC_ax, Cruise_2_ACC_ax, ACC_2_Cruise_ax, ACC_2_CACC_ax, CACC_2_ACC_ax, v2v_ax


def setUpFigures():
    axSpeed = fig.add_axes([0.06, 0.1, 0.4, 0.5])
    axSpeed.set_ylabel('Speed')
    fig.add_axes(axSpeed)

    axTimeGap = fig.add_axes([0.53, 0.1, 0.45, 0.3])
    axTimeGap.set_title('Time gap')
    fig.add_axes(axTimeGap)

    return axSpeed, axTimeGap


def update_plot(l, ax, x, y, update_artists):
    l.set_xdata(x)
    l.set_ydata(y)
    if update_artists:
        ax.draw_artist(ax.patch)
        ax.draw_artist(l)
        ax.relim()
        ax.autoscale_view(True, True, True)

####################################################################################################################
## PROGRAM STARTS HERE

timeSpan = 20  # seconds time span
sampleFreq = 5

fig = plt.figure()

plt.style.use('seaborn')

bCruise, ax_bCruise, bACC, ax_bACC, bCACC, ax_bCACC, bSyncyng, ax_bSync, bStart, ax_bStart = setupButtons()
sSlider, hSlider, pSlider = setupSliders()
axSpeed, axTimeGap = setUpFigures()
targV, targV_far, targVCACC_ax, Cruise_2_ACC_ax, ACC_2_Cruise_ax, ACC_2_CACC_ax, CACC_2_ACC_ax, v2v_ax = setupFixedImages()

line1, = axSpeed.plot(np.zeros((sampleFreq*timeSpan,)))
line1_1 = lines.Line2D(xdata=np.zeros((sampleFreq*timeSpan)), ydata=np.zeros((sampleFreq*timeSpan)), color='r')
axSpeed.add_line(line1_1)
line1_2 = lines.Line2D(xdata=np.zeros((sampleFreq*timeSpan)), ydata=np.zeros((sampleFreq*timeSpan)), color='k')
axSpeed.add_line(line1_2)
line2, = axTimeGap.plot(np.zeros((sampleFreq*timeSpan)))
line2_1 = lines.Line2D(xdata=np.zeros((sampleFreq*timeSpan)), ydata=np.zeros((sampleFreq*timeSpan)), color='r')
axTimeGap.add_line(line2_1)
line2_2 = lines.Line2D(xdata=np.zeros((sampleFreq*timeSpan)), ydata=np.zeros((sampleFreq*timeSpan)), color='k')
axTimeGap.add_line(line2_2)

plt.ion()  # Set interactive mode ON, so matplotlib will not be blocking the window
fig.show()
fig.canvas.draw()

sSlider.on_changed(sliderCallbackSetSpeed)
hSlider.on_changed(sliderCallbackTimeGap)
pSlider.on_changed(sliderCallbackPerformance)

print("ego_veh_id: " + str(ego_veh_id))

if ego_veh_id == PRIUS:
    veh_id = './img/prius.png'
    ip_address = '172.16.0.127'
elif ego_veh_id == ACCORD:
    veh_id = './img/accord.png'
    ip_address = '172.16.0.120'
elif ego_veh_id == TAURUS:
    veh_id = './img/prius.png'
    ip_address = '172.16.0.128'

ip_address = '172.16.0.120'

print("ip_address: " + str(ip_address))

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((ip_address, 4000))

#  SYNCHRONIZATION ROUTINE

rx_data_int = [0, 0]

sync_from_server = False
start_from_server = False
print("Sync routine " + str(start_flag))
execution = 0

if start_flag:
    ax_bSync.images[0].set_data(plt.imread('./img/Synced.png'))
    ax_bStart.images[0].set_data(plt.imread('./img/Started.png'))
    data_tx = str(int(True)) + ' ' + str(int(True)) + ' '
    print("Sending: " + data_tx)
    s.sendall(data_tx.encode('ascii'))
    receivedData = s.recv(1024).decode()
    print("Received1: " + receivedData)
    parsed_rx_data = receivedData.split(' ')
    rx_data_int = [int(numeric_string) for numeric_string in parsed_rx_data]
    print("Sync_server1:" + str(rx_data_int[0]) + " Start_server:" + str(rx_data_int[1]))

else:
    while not start_flag:
        data_tx = str(int(sync_requested)) + ' ' + str(int(start_requested)) + ' '
        print("Sending: " + data_tx)
        s.sendall(data_tx.encode('ascii'))
        receivedData = s.recv(1024).decode()
        print("Received2: " + receivedData)
        parsed_rx_data = receivedData.split(' ')
        rx_data_int = [int(numeric_string) for numeric_string in parsed_rx_data]

        print("Sync_server2:" + str(rx_data_int[0]) + " Start_server:" + str(rx_data_int[1]))

        sync_from_server = rx_data_int[0]
        start_from_server = rx_data_int[1]
        if sync_requested and not sync_from_server:
            ax_bSync.images[0].set_data(plt.imread('./img/Syncing.png'))
        if start_requested and not start_from_server:
            ax_bStart.images[0].set_data(plt.imread('./img/Starting.png'))
        if sync_from_server:
            ax_bSync.images[0].set_data(plt.imread('./img/Synced.png'))
        if start_from_server:
            ax_bStart.images[0].set_data(plt.imread('./img/Started.png'))
            start_flag = True

        fig.canvas.update()
        fig.canvas.flush_events()
        time.sleep(0.05)

# RECURSIVE LOOP

current_control_mode = CRUISE
state_change = True
target_flag = False
v2v_flag = True
t1 = time.time()

rx_data_int = [1, 1, 0, 0, 2000000, 500000, 600000, 700000, 800000, 1000000, 1100000, 1200000]

t = np.linspace(-timeSpan, 0, sampleFreq*timeSpan)
speed_1 = np.zeros((sampleFreq*timeSpan, ))
speed_2 = np.zeros((sampleFreq*timeSpan, ))
speed_3 = np.zeros((sampleFreq*timeSpan, ))
hgap = np.zeros((sampleFreq*timeSpan, ))
hgap_ACC_set = np.zeros((sampleFreq*timeSpan, ))
hgap_CACC_set = np.zeros((sampleFreq*timeSpan, ))
t1 = time.time()
first_time = True
frame = 0

while True:
    data_tx = str(desired_control_mode) + ' ' + \
              str(0) + ' '\
              + str(selectedTimeGap*100000) + ' ' +\
              str(setPointSpeedValue/2.25*100000) + ' ' +\
              str(performance_factor*100000) + ' '
    print(data_tx)
    s.sendall(data_tx.encode('ascii'))
    receivedData = s.recv(1024).decode()
    parsed_rx_data = receivedData.split(' ')
    # print(parsed_rx_data)
    rx_data_int = [int(numeric_string) for numeric_string in parsed_rx_data]
    rx_data = {}
    rx_data["my_pip"] = np.maximum(1, np.minimum(rx_data_int[0], 3))
    rx_data["target_valid"] = rx_data_int[1]
    rx_data["current_control_mode"] = rx_data_int[2]
    rx_data["v2v_available"] = bool(rx_data_int[3])
    rx_data["ego_speed"] = rx_data_int[4] / 100000
    rx_data["measured_tgap"] = np.minimum(rx_data_int[5], 6) / 100000
    rx_data["ACC_tgap"] = rx_data_int[6] / 100000
    rx_data["CACC_tgap"] = rx_data_int[7] / 100000
    rx_data["veh_1_speed"] = rx_data_int[8] / 100000
    rx_data["veh_2_speed"] = rx_data_int[9] / 100000
    rx_data["veh_3_speed"] = rx_data_int[10] / 100000
    rx_data["time"] = rx_data_int[11] / 100000
    frame = frame+0.05
    # rx_data["time"] = frame

    if current_control_mode != rx_data["current_control_mode"] or first_time:
        state_change = True
        current_control_mode = rx_data["current_control_mode"]

    if state_change or first_time:
        ax_bACC.images[0].set_data(plt.imread('./img/ACC.png'))
        ax_bCACC.images[0].set_data(plt.imread('./img/CACC.png'))
        ax_bCruise.images[0].set_data(plt.imread('./img/Cruise.png'))
        if current_control_mode == CACC:
            ax_bCACC.images[0].set_data(plt.imread('./img/CACC_on.png'))
            Cruise_2_ACC_ax.set_visible(False)
            ACC_2_Cruise_ax.set_visible(False)
            ACC_2_CACC_ax.set_visible(False)
            CACC_2_ACC_ax.set_visible(False)
            targV_far.set_visible(False)
            targV.set_visible(False)
            targVCACC_ax.set_visible(bool(rx_data["target_valid"]))
        if current_control_mode == ACC:
            ax_bACC.images[0].set_data(plt.imread('./img/ACC_on.png'))
            Cruise_2_ACC_ax.set_visible(False)
            ACC_2_Cruise_ax.set_visible(False)
            ACC_2_CACC_ax.set_visible(False)
            CACC_2_ACC_ax.set_visible(False)
            targV_far.set_visible(False)
            targVCACC_ax.set_visible(False)
            targV.set_visible(bool(rx_data["target_valid"]))
        if current_control_mode == CRUISE:
            ax_bCruise.images[0].set_data(plt.imread('./img/Cruise_on.png'))
            Cruise_2_ACC_ax.set_visible(False)
            ACC_2_Cruise_ax.set_visible(False)
            ACC_2_CACC_ax.set_visible(False)
            CACC_2_ACC_ax.set_visible(False)
            targVCACC_ax.set_visible(False)
            targV.set_visible(False)
            targV_far.set_visible(bool(rx_data["target_valid"]))
        if current_control_mode == CRUISE_2_ACC:
            Cruise_2_ACC_ax.set_visible(True)
            ACC_2_Cruise_ax.set_visible(False)
            ACC_2_CACC_ax.set_visible(False)
            CACC_2_ACC_ax.set_visible(False)
            targV_far.set_visible(False)
            targVCACC_ax.set_visible(False)
            targV.set_visible(bool(rx_data["target_valid"]))
        if current_control_mode == ACC_2_CRUISE:
            Cruise_2_ACC_ax.set_visible(False)
            ACC_2_Cruise_ax.set_visible(True)
            ACC_2_CACC_ax.set_visible(False)
            CACC_2_ACC_ax.set_visible(False)
            targVCACC_ax.set_visible(False)
            targV.set_visible(False)
            targV_far.set_visible(bool(rx_data["target_valid"]))
        if current_control_mode == ACC_2_CACC:
            Cruise_2_ACC_ax.set_visible(False)
            ACC_2_Cruise_ax.set_visible(False)
            ACC_2_CACC_ax.set_visible(True)
            CACC_2_ACC_ax.set_visible(False)
            targV_far.set_visible(False)
            targV.set_visible(False)
            targVCACC_ax.set_visible(bool(rx_data["target_valid"]))
        if current_control_mode == CACC_2_ACC:
            Cruise_2_ACC_ax.set_visible(False)
            ACC_2_Cruise_ax.set_visible(False)
            ACC_2_CACC_ax.set_visible(False)
            CACC_2_ACC_ax.set_visible(True)
            targV_far.set_visible(False)
            targVCACC_ax.set_visible(False)
            targV.set_visible(bool(rx_data["target_valid"]))
    state_change = False

    if target_flag ^ bool(rx_data["target_valid"]) or first_time:
        if current_control_mode == CRUISE or current_control_mode == ACC_2_CRUISE:
            targV_far.set_visible(bool(rx_data["target_valid"]))
            targVCACC_ax.set_visible(False)
            targV.set_visible(False)
        if current_control_mode == ACC or current_control_mode == CRUISE_2_ACC or current_control_mode == CACC_2_ACC:
            targV_far.set_visible(False)
            targVCACC_ax.set_visible(False)
            targV.set_visible(bool(rx_data["target_valid"]))
        if current_control_mode == CACC or current_control_mode == ACC_2_CACC:
            targV_far.set_visible(False)
            targV.set_visible(False)
            targVCACC_ax.set_visible(bool(rx_data["target_valid"]))

    target_flag = bool(rx_data["target_valid"])

    if v2v_flag ^ rx_data["v2v_available"] or first_time:
        v2v_ax.set_visible(rx_data["v2v_available"])
    v2v_flag = rx_data["v2v_available"]

    t = np.roll(t, -1)
    t[-1] = rx_data["time"]
    speed_1 = np.roll(speed_1, -1)
    speed_1[-1] = rx_data["veh_1_speed"]
    speed_2 = np.roll(speed_2, -1)
    speed_2[-1] = rx_data["veh_2_speed"]
    speed_3 = np.roll(speed_3, -1)
    speed_3[-1] = rx_data["veh_3_speed"]
    hgap = np.roll(hgap, -1)
    hgap[-1] = rx_data["measured_tgap"]
    hgap_ACC_set = np.roll(hgap_ACC_set, -1)
    hgap_ACC_set[-1] = rx_data["ACC_tgap"]
    hgap_CACC_set = np.roll(hgap_CACC_set, -1)
    hgap_CACC_set[-1] = rx_data["CACC_tgap"]

    update_plot(line1, axSpeed, t, speed_1, False)
    update_plot(line1_1, axSpeed, t, speed_2, False)
    update_plot(line1_2, axSpeed, t, speed_3, True)
    update_plot(line2, axTimeGap, t, hgap, True)
    update_plot(line2_1, axTimeGap, t, hgap_ACC_set, True)
    update_plot(line2_2, axTimeGap, t, hgap_CACC_set, True)

#    fig.canvas.update()
    fig.canvas.flush_events()
    first_time = False
    time.sleep(0.2)

    # print(1 / (time.time() - t1))
    # t1 = time.time()

s.close()
