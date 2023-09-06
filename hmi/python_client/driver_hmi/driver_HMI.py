import numpy as np
import socket, time
from datetime import datetime
#import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.widgets import Slider
import matplotlib.image as image
#from scipy.ndimage.interpolation import shift
import json
import pickle
import matplotlib.lines as lines
from matplotlib.collections import PatchCollection

PRIUS = 0
CAMRY = 1
LEAF = 2

CRUISE = 0
ACC = 1
CACC = 2
CRUISE_2_ACC = 4
ACC_2_CRUISE = 5
CACC_2_ACC = 6
ACC_2_CACC = 7

####### THESE PARAMETERS HAVE TO BE DEFINED BEFORE RUNNING
ego_veh_id = PRIUS 
setPointSpeedValue = 68.24
desired_control_mode = CACC
gap_choice = 3
performance_factor = 0.3
start_flag = True  # When False, forces to hold control system for time syncing
############################

state_change, gap_change = False, False
sync_requested, start_requested = False, False

class buttonCallback(object):
    def Cruise(self, event):
        print("Cruise")
    def ACC(self, event):
        global desired_control_mode
        desired_control_mode = ACC
        print("ACC requested")
    def CACC(self, event):
        global desired_control_mode
        desired_control_mode = CACC
        print("CACC requested")
    def gap_up_arrow(self, event):
        global gap_choice
        global gap_change
        gap_choice = np.minimum(gap_choice + 1, 5)
        gap_change = True
    def gap_down_arrow(self, event):
        global gap_choice
        global gap_change
        gap_choice = np.maximum(gap_choice - 1, 1)
        gap_change = True
    def Sync(self, event):
        global sync_requested
        sync_requested = True
    def Start(self, event):
        global start_requested
        start_requested = True

def setupButtons():
    bCallback = buttonCallback()
    x0 = 0.5
    y0 = 0.7
    width = 0.18
    height = 0.18

    CC_im = plt.imread('./img/Cruise.png')
    ACC_im = plt.imread('./img/ACC.png')
    CACC_im = plt.imread('./img/CACC.png')
    GUp_im = plt.imread('./img/up_arrow.png')
    GDown_im = plt.imread('./img/down_arrow.png')
    Sync_im = plt.imread('./img/Sync.png')
    Start_im = plt.imread('./img/Start.png')

    CruiseAx = plt.axes([x0-2*width, y0, width, height])
    CruiseAx.axis('off')
    bCruise = Button(CruiseAx, '', image=CC_im, color='white')
    bCruise.on_clicked(bCallback.Cruise)

    ACCAx = plt.axes([x0-width/2, y0, width, height])
    ACCAx.axis('off')
    bACC = Button(ACCAx, '', image=ACC_im, color='white')
    bACC.on_clicked(bCallback.ACC)

    CACCAx = plt.axes([x0 + width, y0, width, height])
    CACCAx.axis('off')
    bCACC = Button(CACCAx, '', image=CACC_im, color='white')
    bCACC.on_clicked(bCallback.CACC)

    GapUpArrowAx = plt.axes([0.78, 0.45, 0.2, 0.2])
    GapUpArrowAx.axis('off')
    bGapUpArrow = Button(GapUpArrowAx , '', image=GUp_im, color='white')
    bGapUpArrow.on_clicked(bCallback.gap_up_arrow)

    GapDownArrowAx = plt.axes([0.78, 0.1, 0.2, 0.2])
    GapDownArrowAx.axis('off')
    bGapDownArrow = Button(GapDownArrowAx, '', image=GDown_im, color='white')
    bGapDownArrow.on_clicked(bCallback.gap_down_arrow)

    SyncAx = plt.axes([x0-2.1*width, y0 - 1.0 * height, width * 0.7, height * 0.7])
    SyncAx.axis('off')
    bSyncyng = Button(SyncAx, '', image=Sync_im, color='white')
    bSyncyng.on_clicked(bCallback.Sync)

    StartAx = plt.axes([x0-2.1*width, y0 - 2.0 * height, width * 0.7, height * 0.7])
    StartAx.axis('off')
    bStart = Button(StartAx, '', image=Start_im, color='white')
    bStart.on_clicked(bCallback.Start)

    return bCruise, CruiseAx, bACC, ACCAx, bCACC, CACCAx, bGapUpArrow, bGapDownArrow, bSyncyng, SyncAx, bStart, StartAx

def setupFixedImages(vehicle_id):
    path_img = plt.imread('./img/path.png')
    path_img[:, :, -1] = 0.6
    path_logo_ax = plt.axes([0.02, 0.85, 0.15, 0.15], frameon=True)
    path_logo_ax.imshow(path_img)
    path_logo_ax.axis('off')

    vehicle_img = plt.imread(vehicle_id)
    vehicle_img[:, :, -1] = 0.6
    vehicle_ax = plt.axes([0.81, 0.85, 0.15, 0.15], frameon=True)
    vehicle_ax.imshow(vehicle_img)
    vehicle_ax.axis('off')

    lanes_ax = plt.axes([0.2, 0.1, 0.6, 0.4], frameon=True)
    lanes_ax.imshow(plt.imread('./img/lanes.png'))
    lanes_ax.axis('off')

    v2v_lanes_ax = plt.axes([0.01, 0.01, 0.1, 0.85], frameon=True)
    v2v_lanes_ax.imshow(plt.imread('./img/v2v_lanes.png'))
    v2v_lanes_ax.axis('off')

    targetV_far_ax = plt.axes([0.5-0.05, 0.32, 0.1, 0.2], frameon=True)
    targetV_far_ax.imshow(plt.imread('./img/target.png'))
    targetV_far_ax.axis('off')

    targetV_ax = plt.axes([0.5-0.18, 0.25, 0.36, 0.28], frameon=True)
    targetV_ax.imshow(plt.imread('./img/ACC_target.png'))
    targetV_ax.axis('off')

    targetVCACC_ax = plt.axes([0.5 - 0.2, 0.22, 0.4, 0.3], frameon=True)
    targetVCACC_ax.imshow(plt.imread('./img/CACC_target.png'))
    targetVCACC_ax.axis('off')

    GapLevel_ax = plt.axes([0.73, 0.273, 0.30, 0.18], frameon=True)
    GapLevel_ax.imshow(plt.imread('img/3.png'))
    GapLevel_ax.axis('off')

    Cruise_2_ACC_ax = plt.axes([0.315, 0.746, 0.1, 0.1], frameon=True)
    Cruise_2_ACC_ax.imshow(plt.imread('./img/right_arrow.png'))
    Cruise_2_ACC_ax.axis('off')

    ACC_2_Cruise_ax = plt.axes([0.315, 0.7461, 0.1, 0.1], frameon=True)
    ACC_2_Cruise_ax.imshow(plt.imread('./img/left_arrow.png'))
    ACC_2_Cruise_ax.axis('off')

    ACC_2_CACC_ax = plt.axes([0.585, 0.7461, 0.1, 0.1], frameon=True)
    ACC_2_CACC_ax.imshow(plt.imread('./img/right_arrow.png'))
    ACC_2_CACC_ax.axis('off')

    CACC_2_ACC_ax = plt.axes([0.585, 0.746, 0.1, 0.1], frameon=True)
    CACC_2_ACC_ax.imshow(plt.imread('./img/left_arrow.png'))
    CACC_2_ACC_ax.axis('off')

    v2v_ax = plt.axes([0.731, 0.886, 0.08, 0.08], frameon=True)
    v2v_ax.imshow(plt.imread('./img/v2v_av.png'))
    v2v_ax.axis('off')

    Cruise_2_ACC_ax.set_visible(False)
    ACC_2_Cruise_ax.set_visible(False)
    ACC_2_CACC_ax.set_visible(False)
    CACC_2_ACC_ax.set_visible(False)
    v2v_ax.set_visible(False)
    targetVCACC_ax.set_visible(False)
    targetV_ax.set_visible(False)
    targetV_far_ax.set_visible(False)
    return targetV_ax, targetV_far_ax, targetVCACC_ax, GapLevel_ax, Cruise_2_ACC_ax, ACC_2_Cruise_ax, ACC_2_CACC_ax, CACC_2_ACC_ax, v2v_ax

def setupStringVehicles():
    x0 = 0.034
    y0 = 0.6
    width = 0.05
    height = 0.15

    ego = plt.imread('./img/ego_veh.png')
    non_com_veh = plt.imread('./img/non_com_veh.png')
    com_veh = plt.imread('./img/com_veh.png')

    veh1_ax = plt.axes([x0, y0, width, height])
    veh1_ax.axis('off')
    bveh1 = Button(veh1_ax, '', image=ego, color='white')

    veh2_ax = plt.axes([x0, y0 - 1.5*height, width, height])
    veh2_ax.axis('off')
    bveh2 = Button(veh2_ax, '', image=com_veh, color='white')

    veh3_ax = plt.axes([x0, y0 - 3* height, width, height])
    veh3_ax.axis('off')
    bveh3 = Button(veh3_ax, '', image=com_veh, color='white')

    veh1_ax.set_visible(True)
    veh2_ax.set_visible(False)
    veh3_ax.set_visible(False)
    return bveh1, bveh2, bveh3, veh1_ax, veh2_ax, veh3_ax
ego_veh_id = PRIUS;
fig = plt.figure()
if ego_veh_id == PRIUS:
    veh_id = './img/prius.png'
    ip_address = '172.16.0.127'
elif ego_veh_id == LEAF:
    veh_id = './img/leaf.jpg'
    ip_address = '172.16.0.120'
elif ego_veh_id == CAMRY:
    veh_id = './img/camry.jpg'
    ip_address = '172.16.0.128'

targV, targV_far, targVCACC_ax, GapLevel_ax, Cruise_2_ACC_ax, ACC_2_Cruise_ax, ACC_2_CACC_ax, CACC_2_ACC_ax, v2v_ax = setupFixedImages(veh_id)
bCruise, ax_bCruise, bACC, ax_bACC, bCACC, ax_bCACC, bGapUpArrow, bGapDownArrow, bSyncyng, ax_bSync, bStart, ax_bStart = setupButtons()
bveh1, bveh2, bveh3, veh1_ax, veh2_ax, veh3_ax = setupStringVehicles()

plt.ion()
fig.show()
fig.canvas.draw()

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
    ax_bSync.images[0].set_data(plt.imread('./img/Synced.png'))
    ax_bStart.images[0].set_data(plt.imread('./img/Started.png'))
    data_tx = str(int(True)) + ' ' + str(int(True)) + ' '
    print("Sending: " + data_tx)
    s.sendall(data_tx.encode('ascii'))
    receivedData = s.recv(1024).decode()
    print("Received: " + receivedData)
    parsed_rx_data = receivedData.split(' ')
    rx_data_int = [int(numeric_string) for numeric_string in parsed_rx_data]
    print("Sync_server:" + str(rx_data_int[0]) + " Start_server:" + str(rx_data_int[1]))
else:
    while not start_flag:
        data_tx = str(int(sync_requested)) + ' ' + str(int(start_requested)) + ' '
        print("Sending: " + data_tx)
        s.sendall(data_tx.encode('ascii'))
        receivedData = s.recv(1024).decode()
        print("Received: " + receivedData)
        parsed_rx_data = receivedData.split(' ')
        rx_data_int = [int(numeric_string) for numeric_string in parsed_rx_data]

        print("Sync_server: " + str(rx_data_int[0]) + " Start_server: " + str(rx_data_int[1]))

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

        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.05)

#  RECURSIVE LOOP

current_control_mode = CRUISE
state_change = True
target_flag = False
v2v_flag = False
t1 = time.time()
rx_data_int = [3, 1, 1, 1, 3, 4, 5, 6, 7, 8, 9, 10]
first_time = True

while True:
    data_tx = str(desired_control_mode) + ' ' + str(gap_choice) + ' ' + str(0) + ' ' + str(setPointSpeedValue/2.25*100000) + ' ' + str(performance_factor*100000) + ' '
    s.sendall(data_tx.encode('ascii'))
    receivedData = s.recv(1024).decode()
    parsed_rx_data = receivedData.split(' ')
    rx_data_int = [int(numeric_string) for numeric_string in parsed_rx_data]
    print(rx_data_int)
    rx_data={}
    rx_data["my_pip"] = np.maximum(1, np.minimum(rx_data_int[0], 3))
    rx_data["target_valid"] = rx_data_int[1]
    rx_data["current_control_mode"] = rx_data_int[2]
    rx_data["v2v_available"] = bool(rx_data_int[3])
    rx_data["ego_speed"] = rx_data_int[4]/100000
    rx_data["measured_tgap"] = rx_data_int[5]/100000
    rx_data["ACC_tgap"] = rx_data_int[6]/100000
    rx_data["CACC_tgap"] = rx_data_int[7]/100000
    rx_data["veh_1_speed"] = rx_data_int[8]/100000
    rx_data["veh_2_speed"] = rx_data_int[9]/100000
    rx_data["veh_3_speed"] = rx_data_int[10] / 100000
    rx_data["time"] = rx_data_int[11] / 100000

    if current_control_mode != rx_data["current_control_mode"] or first_time:
        state_change = True
        current_control_mode = rx_data["current_control_mode"]

    if state_change or first_time:
        ax_bACC.images[0].set_data(plt.imread('./img/ACC.png'))
        ax_bCACC.images[0].set_data(plt.imread('./img/CACC.png'))
        ax_bCruise.images[0].set_data(plt.imread('./img/Cruise.png'))
        if current_control_mode == ACC:
            ax_bACC.images[0].set_data(plt.imread('./img/ACC_on.png'))
            Cruise_2_ACC_ax.set_visible(False)
            ACC_2_Cruise_ax.set_visible(False)
            ACC_2_CACC_ax.set_visible(False)
            CACC_2_ACC_ax.set_visible(False)

            targV_far.set_visible(False)
            targVCACC_ax.set_visible(False)
            targV.set_visible(bool(rx_data["target_valid"]))

            veh1_ax.images[0].set_data(plt.imread('./img/non_com_veh.png'))
            veh2_ax.images[0].set_data(plt.imread('./img/ego_veh.png'))
            veh2_ax.set_visible(True)
            veh3_ax.set_visible(False)
        if current_control_mode == CRUISE:
            ax_bCruise.images[0].set_data(plt.imread('./img/Cruise_on.png'))
            Cruise_2_ACC_ax.set_visible(False)
            ACC_2_Cruise_ax.set_visible(False)
            ACC_2_CACC_ax.set_visible(False)
            CACC_2_ACC_ax.set_visible(False)

            targV_far.set_visible(bool(rx_data["target_valid"]))
            targVCACC_ax.set_visible(False)
            targV.set_visible(False)

            veh1_ax.images[0].set_data(plt.imread('./img/ego_veh.png'))
            veh2_ax.set_visible(False)
            veh3_ax.set_visible(False)
        if current_control_mode == CRUISE_2_ACC:
            Cruise_2_ACC_ax.set_visible(True)
            ACC_2_Cruise_ax.set_visible(False)
            ACC_2_CACC_ax.set_visible(False)
            CACC_2_ACC_ax.set_visible(False)

            targV_far.set_visible(False)
            targVCACC_ax.set_visible(False)
            targV.set_visible(bool(rx_data["target_valid"]))

            veh1_ax.images[0].set_data(plt.imread('./img/non_com_veh.png'))
            veh2_ax.images[0].set_data(plt.imread('./img/ego_veh.png'))
            veh2_ax.set_visible(True)
            veh3_ax.set_visible(False)
        if current_control_mode == ACC_2_CRUISE:
            Cruise_2_ACC_ax.set_visible(False)
            ACC_2_Cruise_ax.set_visible(True)
            ACC_2_CACC_ax.set_visible(False)
            CACC_2_ACC_ax.set_visible(False)

            targV_far.set_visible(bool(rx_data["target_valid"]))
            targVCACC_ax.set_visible(False)
            targV.set_visible(False)

            veh1_ax.images[0].set_data(plt.imread('./img/ego_veh.png'))
            veh2_ax.set_visible(False)
            veh3_ax.set_visible(False)
        if current_control_mode == ACC_2_CACC:
            Cruise_2_ACC_ax.set_visible(False)
            ACC_2_Cruise_ax.set_visible(False)
            ACC_2_CACC_ax.set_visible(True)
            CACC_2_ACC_ax.set_visible(False)

            targV_far.set_visible(False)
            targV.set_visible(False)
            targVCACC_ax.set_visible(bool(rx_data["target_valid"]))

            if(rx_data["my_pip"] == 2):
                veh1_ax.images[0].set_data(plt.imread('./img/com_veh.png'))
                veh2_ax.images[0].set_data(plt.imread('./img/ego_veh.png'))
                veh2_ax.set_visible(True)
                veh3_ax.set_visible(False)
            elif(rx_data["my_pip"] == 3):
                veh1_ax.images[0].set_data(plt.imread('./img/com_veh.png'))
                veh2_ax.images[0].set_data(plt.imread('./img/com_veh.png'))
                veh3_ax.images[0].set_data(plt.imread('./img/ego_veh.png'))
                veh2_ax.set_visible(True)
                veh3_ax.set_visible(True)
        if current_control_mode == CACC_2_ACC:
            Cruise_2_ACC_ax.set_visible(False)
            ACC_2_Cruise_ax.set_visible(False)
            ACC_2_CACC_ax.set_visible(False)
            CACC_2_ACC_ax.set_visible(True)

            targV_far.set_visible(False)
            targVCACC_ax.set_visible(False)
            targV.set_visible(bool(rx_data["target_valid"]))

            veh1_ax.images[0].set_data(plt.imread('./img/non_com_veh.png'))
            veh2_ax.images[0].set_data(plt.imread('./img/ego_veh.png'))
            veh2_ax.set_visible(True)
            veh3_ax.set_visible(False)
        if current_control_mode == CACC:
            ax_bCACC.images[0].set_data(plt.imread('./img/CACC_on.png'))
            Cruise_2_ACC_ax.set_visible(False)
            ACC_2_Cruise_ax.set_visible(False)
            ACC_2_CACC_ax.set_visible(False)
            CACC_2_ACC_ax.set_visible(False)
            targV.set_visible(False)
            targV_far.set_visible(False)
            targVCACC_ax.set_visible(bool(rx_data["target_valid"]))
            if rx_data["my_pip"] == 2:
                veh1_ax.images[0].set_data(plt.imread('./img/com_veh.png'))
                veh2_ax.images[0].set_data(plt.imread('./img/ego_veh.png'))
                veh2_ax.set_visible(True)
                veh3_ax.set_visible(False)
            elif rx_data["my_pip"] == 3:
                veh1_ax.images[0].set_data(plt.imread('./img/com_veh.png'))
                veh2_ax.images[0].set_data(plt.imread('./img/com_veh.png'))
                veh3_ax.images[0].set_data(plt.imread('./img/ego_veh.png'))
                veh2_ax.set_visible(True)
                veh3_ax.set_visible(True)
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
            targVCACC_ax.set_visible(bool(rx_data["target_valid"]))
            targV.set_visible(False)

    target_flag = bool(rx_data["target_valid"])

    if v2v_flag ^ rx_data["v2v_available"] or first_time:
        v2v_ax.set_visible(rx_data["v2v_available"])
    v2v_flag = rx_data["v2v_available"]

    if gap_change:
        if gap_choice == 1:
            GapLevel_ax.images[0].set_data(plt.imread('./img/1.png'))
        elif gap_choice == 2:
            GapLevel_ax.images[0].set_data(plt.imread('./img/2.png'))
        elif gap_choice == 3:
            GapLevel_ax.images[0].set_data(plt.imread('./img/3.png'))
        elif gap_choice == 4:
            GapLevel_ax.images[0].set_data(plt.imread('./img/4.png'))
        elif gap_choice == 5:
            GapLevel_ax.images[0].set_data(plt.imread('./img/5.png'))
        gap_change = False

    fig.canvas.draw()#update()
    fig.canvas.flush_events()
    first_time = False
    # time.sleep(0.1)
    #print(1 / np.maximum(0.00001,(time.time() - t1)))
    # t1 = time.time()

s.close()

