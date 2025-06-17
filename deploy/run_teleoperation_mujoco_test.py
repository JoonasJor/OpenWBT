from config import Config
from controllers.controller import Runner_handle_mujoco #, Runner_offline_mujoco

import time
import cv2
from multiprocessing import shared_memory
from datetime import datetime
import os

from teleop.robot_control.robot_arm_ik import G1_29_ArmIK

import numpy as np
import torch

torch.set_printoptions(precision=3)
np.set_printoptions(precision=3)

arm_ik = G1_29_ArmIK()

def get_output_dir():
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join("record_mujoco_images", timestamp)
    os.makedirs(output_dir, exist_ok=True)
    return output_dir


def save_images(shm_name, shape, dtype, interval=1.0 / 25.):
    shm = shared_memory.SharedMemory(name=shm_name)
    img = np.ndarray(shape, dtype=dtype, buffer=shm.buf)

    output_dir = get_output_dir()
    counter = 0

    try:
        while True:
            img_copy = img.copy()
            filename = os.path.join(output_dir, f"{counter:04d}.png")
            cv2.imwrite(filename, img_copy)
            counter += 1
            time.sleep(interval)
    except KeyboardInterrupt:
        pass
    finally:
        shm.close()


def deploy_handle_mujoco(args):
    import mujoco.viewer
    import mujoco
    config_path = f"deploy/configs/{args.config}"
    print(config_path)
    config = Config(config_path)

    # Initialize DDS communication
    runner = Runner_handle_mujoco(config, args=args)
    current_mode = "SQUAT"
    print('Squat mode!')
    print('Press 1 to start the locomotion mode!')

    with mujoco.viewer.launch_passive(runner.m, runner.d) as viewer:
        runner.last_control_timestamp = time.time()
        while True:
            if current_mode == "LOCOMOTION":
                runner.run_loco(manual=True)
                viewer.sync()
                if runner.transfer_to_squat:
                    current_mode = "SQUAT"
                    print('Squat mode!')
                    print('Press 1 to start the locomotion mode!')

            elif current_mode == "SQUAT":
                runner.run_squat(manual=True)
                viewer.sync()
                if runner.transfer_to_loco:
                    current_mode = "LOCOMOTION"
                    print('Locomotion mode!')
                    print('Press 2 to start the squat mode!')

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--config",
                        type=str,
                        help="config file name in the configs folder",
                        default="run_loco_squat_grasp.yaml")
    parser.add_argument("--save_data", action="store_true", help="whether saving the mujoco data")
    parser.add_argument("--save_data_dir", type=str, help="where to save the data", default="./save_mujoco_data")
    parser.add_argument("--save_image", action="store_true", help="whether saving the mujoco image")
    args = parser.parse_args()

    deploy_handle_mujoco(args)
