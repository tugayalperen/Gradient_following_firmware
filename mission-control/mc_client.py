import threading
import numpy as np
import logging
import time
import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.swarm import CachedCfFactory
import pygame
from pygame.locals import *
import matplotlib.pyplot as plt
from matplotlib import cm, colors
from matplotlib import patches
from PyP100 import PyP100
import os
import json
import argparse

## A, B --> 35 secs
## C, D --> 65 secs
## E --> 40 secs

experiment_name = "Experiment_1"
if not os.path.isdir(experiment_name):
    os.mkdir(experiment_name)
    print("The directory is created.")
else:
    print("The experiment directory already exists!!q!")
    exit()

# This initializes the connection to the bulb, create as many as needed for your setup
p1 = PyP100.P100("00.00.0.000", "your.email@gmail.com", "your_password")

p_array = [p1]
p1.handshake()
p1.login()


# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

num_cf = 5
max_num_cf = 5
max_datapoint = 30000
data = []
id_list = []
log_all = np.zeros([max_datapoint*num_cf, 4])
steps_last = np.zeros([1, num_cf])
light_states_all = np.zeros([max_datapoint, 6])
steps = np.zeros(max_num_cf)
xs = np.zeros(max_num_cf)
ys = np.zeros(max_num_cf)
hs = np.zeros(max_num_cf)
ls = np.zeros(max_num_cf)

global config
config_file = "./config.json"
with open(config_file, "r") as f:
    config = json.loads(f.read())

def display(str):
    screen.fill((255, 128, 0))
    text = font.render(str, True, (255, 255, 255), (159, 182, 205))
    textRect = text.get_rect()
    textRect.centerx = screen.get_rect().centerx
    textRect.centery = screen.get_rect().centery

    screen.blit(text, textRect)
    pygame.display.update()

def decode_aggdata(noncoded):
    x = int(np.floor(noncoded/np.power(10, 7)))/10
    noncoded = noncoded - x*10*np.power(10, 7)
    y = int(np.floor(noncoded / np.power(10, 5))) / 10
    noncoded = noncoded - y * 10 * np.power(10, 5)
    h = int(np.floor(noncoded / np.power(10, 3))) / 10
    noncoded = noncoded - h * 10 * np.power(10, 3)
    l = noncoded
    return x, y, h, l


def logg_swarm(scf):
    log_config = LogConfig(name="synthLog", period_in_ms=500)
    log_config.add_variable("synthLog.agg_data", "int32_t")
    log_config.add_variable("synthLog.agent_id", "uint8_t")

    with SyncLogger(scf, log_config) as logger:
        for entry in logger:
            agg_data = entry[1]["synthLog.agg_data"]
            agent_id = entry[1]["synthLog.agent_id"]
            x, y, h, l = decode_aggdata(agg_data)
            log_all[int((agent_id-1) + steps[agent_id-1]*max_num_cf), :] = x, y, h, l
            xs[int(agent_id-1)] = x
            ys[int(agent_id-1)] = y
            hs[int(agent_id-1)] = h
            ls[int(agent_id-1)] = l
            steps[agent_id-1] = steps[agent_id-1] + 1


def _take_off(scf):
    cf = scf.cf
    cf.param.set_value("fmodes.if_takeoff", 1)
    time.sleep(0.1)


def take_off(swarm):
    swarm.parallel_safe(_take_off)


def _land(scf):
    cf = scf.cf
    cf.param.set_value("fmodes.if_terminate",1)
    time.sleep(0.1)


def land(swarm):
    swarm.parallel_safe(_land)


def _update_param(scf):
    global config
    cf = scf.cf
    #####################
    sigma_base = config["sigma_base"]
    sigma_var = config["sigma_var"]
    k1 = config["k1"]
    k2 = config["k2"]
    alpha = config["alpha"]
    beta = config["beta"]
    kappa = config["kappa"]
    light_max = config["light_max"]
    light_min = config["light_min"]
    flight_height = config["flight_height"]
    umax = config["umax"]
    wmax = config["wmax"]
    u_add = config["u_add"]
    update_params = 1
    #####################
    cf.param.set_value("flockParams.sb_param", sigma_base)
    cf.param.set_value("flockParams.sv_param", sigma_var)
    cf.param.set_value("flockParams.k1_param", k1)
    cf.param.set_value("flockParams.k2_param", k2)
    cf.param.set_value("flockParams.alpha_param", alpha)
    cf.param.set_value("flockParams.beta_param", beta)
    cf.param.set_value("flockParams.kappa_param", kappa)
    cf.param.set_value("flockParams.lmx_param", light_max)
    cf.param.set_value("flockParams.lmn_param", light_min)
    cf.param.set_value("flockParams.fh_param", flight_height)
    cf.param.set_value("flockParams.umax_param", umax)
    cf.param.set_value("flockParams.wmax_param", wmax)
    cf.param.set_value("flockParams.u_add_param", u_add)
    cf.param.set_value("flockParams.update_params", update_params)
    print("update finished")


def update_param(swarm):
    swarm.parallel_safe(_update_param)

def log_swarm(swarm):
    swarm.parallel_safe(logg_swarm)


if __name__ == "__main__":

    uris = {
        'radio://0/100/2M/E7E7E7E701',
        'radio://0/100/2M/E7E7E7E702',
        'radio://0/100/2M/E7E7E7E703',
        'radio://0/100/2M/E7E7E7E704',
        'radio://0/100/2M/E7E7E7E705',
    }

    size_x = 8.0
    size_y = 6.0
    norm = colors.Normalize(vmin=50.0, vmax=600.0, clip=True)
    mapper = cm.ScalarMappable(norm=norm, cmap=cm.inferno)

    plt.ion()
    plt.show()

    # cell_corners_x = np.array([0.0, 2.33, 4.66, 4.66, 2.33, 0.0])
    # cell_corners_y = np.array([0.0, 0.0, 0.0, 2.375, 2.375, 2.375])
    # light_centers_x = np.array([1.7, 3.0, 5.1, 5.1, 3.5, 1.7])
    # light_centers_y = np.array([1.4, 1.3, 1.7, 3.5, 3.5, 3.2])
    light_centers_x = np.array([1.18, 2.96, 5.0, 5.2, 3.37, 1.31])
    light_centers_y = np.array([0.99, 1.1, 1.5, 3.5, 3.3, 2.75])
    light_radius = 2.33
    # cell_width = 2.33
    # cell_height = 2.375
    cell_states = [False, False, False, False, False, False]
    for i in range(len(p_array)):
        p_array[i].turnOff()
        cell_states[i] = False
        time.sleep(0.1)

    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption('Python numbers')
    screen.fill((255, 128, 0))
    font = pygame.font.Font(None, 45)
    done = False
    start_seq = False
    obstacle_exp = False
    seq_duration = 100.0
    custom_seq_1 = np.array([1, 6, 5, 2, 3, 4])
    # custom_seq_2 = np.array([1, 5, 3, 4, 2, 6])
    custom_seq_2 = np.array([1, 5, 3])
    # custom_seq_2 = np.array([4, 2, 6])
    custom_seq_3 = np.array([[1, 6], [2, 5], [3, 4]])
    custom_seq_obs = np.array([3, 4, 5])
    # custom_seq_durations_6 = np.array([45.0, 35.0, 120.0])
    custom_seq_durations_6 = np.array([90.0, 20.0, 90.0])
    # custom_seq_durations_6 = np.array([5.0, 3.0, 5.0])
    seq_dir = 1
    seq_obs = True

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')
    display_msg = "No key is pressed!"
    last = time.time()
    last_pressed = time.time()
    last_seq_time = time.time() - seq_duration
    # obs_pos = np.array([5.1, 2.5])
    obs_pos = np.array([4.52, 2.76])
    with Swarm(uris, factory=factory) as swarm:
        thread_1 = threading.Thread(target=log_swarm, args=([swarm]))
        thread_1.start()
        log_index = 0
        while not done:
            pygame.event.pump()
            keys = pygame.key.get_pressed()
            display(display_msg)
            if time.time() - last > 0.5:
                last = time.time()
                ##PLOT##
                plt.axis([-1, size_x, -1, size_y])
                rect = patches.Rectangle((0.0, 0.0), 7.0, 4.75, linewidth=5, edgecolor='r', facecolor='none')
                ax = plt.gca()
                ax.add_patch(rect)
                for c in range(len(cell_states)):
                    if cell_states[c]:
                        # light_rect = patches.Rectangle((cell_corners_x[c], cell_corners_y[c]), cell_width, cell_height,
                        #                                linewidth=5, edgecolor='orange', facecolor='orange', alpha=0.5)
                        light_cent = plt.Circle((light_centers_x[c], light_centers_y[c]), 2.33/2, color='orange', alpha=0.5)
                        ax.add_patch(light_cent)
                if obstacle_exp:
                    obstacle = plt.Circle((obs_pos[0], obs_pos[1]), 0.4, color='red', alpha=0.3)
                    ax.add_patch(obstacle)
                plt.scatter(xs, ys, color=mapper.to_rgba(ls))
                plt.scatter(np.mean(xs), np.mean(ys), color='red')
                print("X-Com is: ", np.mean(xs), " ||| Y-Com is: ", np.mean(ys))
                plt.quiver(xs, ys, np.cos(hs), np.sin(hs))
                plt.draw()
                plt.pause(0.001)
                plt.clf()
                light_states_all[log_index, :] = cell_states
                steps_last = steps
                log_index += 1
                ###
            if keys[K_ESCAPE]:
                done = True
            if keys[K_t] and ((time.time() - last_pressed) > 0.5):
                take_off(swarm)
                display_msg = "Takeoff command is sent!"
                last_pressed = time.time()
            elif keys[K_l] and ((time.time() - last_pressed) > 0.5):
                land(swarm)
                display_msg = "Land command is sent!"
                last_pressed = time.time()
            elif keys[K_u] and ((time.time() - last_pressed) > 0.5):
                with open(config_file, "r") as f:
                    config = json.loads(f.read())
                    print("New config file loaded")
                time.sleep(0.5)
                update_param(swarm)
                display_msg = "Parameters are updated!"
                last_pressed = time.time()
            elif keys[K_q] and ((time.time() - last_pressed) > 0.5):
                display_msg = "Terminate the experiment!"
                last_pressed = time.time()
                np.save(experiment_name + "/log_data_1.npy", log_all)
                np.save(experiment_name + "/steps_last.npy", steps_last)
                np.save(experiment_name + "/lights_states_all.npy", light_states_all)
                exit()
            elif keys[K_o] and ((time.time() - last_pressed) > 0.5) and not start_seq:
                start_seq = True
                seq_num = config["seq_initial"]
                seq_type = config["seq_type"]
                print(seq_num)
                last_pressed = time.time()
            elif start_seq:
                display_msg = "Bulb" + str(seq_num-1) + "is on!"
                if (time.time()-last_seq_time) > seq_duration and not seq_obs:
                    last_seq_time = time.time()
                    print("seq num is: ", seq_num)
                    for i in range(len(p_array)):
                        if cell_states[i]:
                            p_array[i].turnOff()
                            cell_states[i] = False
                            time.sleep(0.1)
                    time.sleep(0.1)
                    if seq_type == 1:
                        p_array[seq_num-1].turnOn()
                        cell_states[seq_num-1] = True
                        seq_num += 1
                        if seq_num > 6:
                            seq_num = 1
                        elif seq_num < 1:
                            seq_num = 6
                    elif seq_type == 2:
                        p_array[seq_num-1].turnOn()
                        cell_states[seq_num-1] = True
                        seq_num -= 1
                        if seq_num > 6:
                            seq_num = 1
                        elif seq_num < 1:
                            seq_num = 6
                    elif seq_type == 3:
                        p_array[custom_seq_1[seq_num-1]-1].turnOn()
                        cell_states[custom_seq_1[seq_num-1]-1] = True
                        seq_num += 1
                        if seq_num > 3:
                            seq_num = 1
                        elif seq_num < 1:
                            seq_num = 3
                    elif seq_type == 4:
                        p_array[custom_seq_2[seq_num-1]-1].turnOn()
                        cell_states[custom_seq_2[seq_num-1]-1] = True
                        seq_num += 1
                        if seq_num > 3:
                            seq_num = 1
                        elif seq_num < 1:
                            seq_num = 3
                    elif seq_type == 5:
                        for i in range(2):
                            _bulb = custom_seq_3[seq_num-1][i]
                            p_array[_bulb-1].turnOn()
                            cell_states[_bulb-1] = True
                            time.sleep(0.2)
                        seq_num = seq_num + seq_dir
                        if seq_num > 3:
                            seq_num = 2
                            seq_dir = -1
                        elif seq_num < 1:
                            seq_num = 2
                            seq_dir = 1
                elif (time.time()-last_seq_time) > custom_seq_durations_6[seq_num-1] and seq_obs:
                    last_seq_time = time.time()
                    print("seq num is: ", seq_num)
                    for i in range(len(p_array)):
                        if cell_states[i]:
                            p_array[i].turnOff()
                            cell_states[i] = False
                            time.sleep(0.1)
                    time.sleep(0.1)
                    print("here")
                    if seq_type == 6:
                        p_array[custom_seq_obs[seq_num-1]-1].turnOn()
                        cell_states[custom_seq_obs[seq_num-1]-1] = True
                        seq_num += 1
                        if seq_num > 3:
                            seq_num = 1
                        elif seq_num < 1:
                            seq_num = 3

            elif keys[K_1] and ((time.time() - last_pressed) > 0.5):
                display_msg = "Bulb 1 is on!"
                for i in range(len(p_array)):
                    if cell_states[i]:
                        p_array[i].turnOff()
                        cell_states[i] = False
                        time.sleep(0.1)
                time.sleep(0.1)
                p_array[0].turnOn()
                cell_states[0] = True
            elif keys[K_2] and ((time.time() - last_pressed) > 0.5):
                display_msg = "Bulb 2 is on!"
                for i in range(len(p_array)):
                    if cell_states[i]:
                        p_array[i].turnOff()
                        cell_states[i] = False
                        time.sleep(0.1)
                time.sleep(0.1)
                p_array[1].turnOn()
                cell_states[1] = True
            elif keys[K_3] and ((time.time() - last_pressed) > 0.5):
                display_msg = "Bulb 3 is on!"
                for i in range(len(p_array)):
                    if cell_states[i]:
                        p_array[i].turnOff()
                        cell_states[i] = False
                        time.sleep(0.1)
                time.sleep(0.1)
                p_array[2].turnOn()
                cell_states[2] = True
            elif keys[K_4] and ((time.time() - last_pressed) > 0.5):
                display_msg = "Bulb 4 is on!"
                for i in range(len(p_array)):
                    if cell_states[i]:
                        p_array[i].turnOff()
                        cell_states[i] = False
                        time.sleep(0.1)
                time.sleep(0.1)
                p_array[3].turnOn()
                cell_states[3] = True
            elif keys[K_5] and ((time.time() - last_pressed) > 0.5):
                display_msg = "Bulb 5 is on!"
                for i in range(len(p_array)):
                    if cell_states[i]:
                        p_array[i].turnOff()
                        cell_states[i] = False
                        time.sleep(0.1)
                time.sleep(0.1)
                p_array[4].turnOn()
                cell_states[4] = True
            elif keys[K_6] and ((time.time() - last_pressed) > 0.5):
                display_msg = "Bulb 6 is on!"
                for i in range(len(p_array)):
                    if cell_states[i]:
                        p_array[i].turnOff()
                        cell_states[i] = False
                        time.sleep(0.1)
                time.sleep(0.1)
                p_array[5].turnOn()
                cell_states[5] = True
            elif keys[K_7] and ((time.time() - last_pressed) > 0.5):
                display_msg = "Bulb 1 and 6 are on!"
                for i in range(len(p_array)):
                    if cell_states[i]:
                        p_array[i].turnOff()
                        cell_states[i] = False
                        time.sleep(0.1)
                time.sleep(0.1)
                p_array[0].turnOn()
                time.sleep(0.1)
                p_array[5].turnOn()
                cell_states[0] = True
                cell_states[5] = True
            elif keys[K_8] and ((time.time() - last_pressed) > 0.5):
                display_msg = "Bulb 2 and 5 are on!"
                for i in range(len(p_array)):
                    if cell_states[i]:
                        p_array[i].turnOff()
                        cell_states[i] = False
                        time.sleep(0.1)
                time.sleep(0.1)
                p_array[1].turnOn()
                time.sleep(0.1)
                p_array[4].turnOn()
                cell_states[1] = True
                cell_states[4] = True
            elif keys[K_9] and ((time.time() - last_pressed) > 0.5):
                display_msg = "Bulb 3 and 4 is on!"
                for i in range(len(p_array)):
                    if cell_states[i]:
                        p_array[i].turnOff()
                        cell_states[i] = False
                        time.sleep(0.1)
                p_array[2].turnOn()
                time.sleep(0.1)
                p_array[3].turnOn()
                time.sleep(0.1)
                cell_states[2] = True
                cell_states[3] = True
            elif keys[K_0] and ((time.time() - last_pressed) > 0.5):
                display_msg = "All bulbs are off!"
                for i in range(len(p_array)):
                    if cell_states[i]:
                        p_array[i].turnOff()
                        cell_states[i] = False
                        time.sleep(0.1)
