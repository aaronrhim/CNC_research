import time
import os

import numpy as np
import tensorflow as tf
import pybullet as p
import threading

from packages import Environment
from packages import Tables

def main():
    device = tf.config.experimental.list_physical_devices('GPU')
    if not device:
        print('No GPUs detected. Running with CPU.')
    else:
        print('Using GPU')

    env = Environment(gui=True)
    env.reset()

    gui_thread = threading.Thread(
        target=Tables.launch_gui,
        args=(env.joints, env.ur5),
        daemon=True
    )
    gui_thread.start()
    
    while True:
        p.stepSimulation()
        time.sleep(1. / 240)  # match physics timestep

if __name__ == '__main__':
    main()