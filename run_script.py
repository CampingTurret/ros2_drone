import subprocess
from pathlib import Path
import numpy as np
import time

PATH_microxrcedds = Path(__file__).parents[1]
PATH_PX4 = Path(__file__).parents[1]
PATH_Commander = Path(__file__).parent

def start_px4():
    # Command: make px4_sitl gz_x500
    print("make px4_sitl gz_x500")
    p = subprocess.Popen(
        ["make", "px4_sitl", "gz_x500"],
        cwd=PATH_PX4,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(2)
    return p

def start_uxrcedss():
    # Command: MicroXRCEAgent udp4 -p 8888
    print("MicroXRCEAgent udp4 -p 8888")
    p = subprocess.Popen(
        ["MicroXRCEAgent", "udp4", "-p", "8888"],
        cwd=PATH_microxrcedds,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(2)
    return p

def repo_pull():
    # Command: git pull in PATH_Commander
    print("git pull")
    subprocess.run(
        ["git", "pull"],
        cwd=PATH_Commander,
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(1)

def build_docker():
    # Command: ./docker/dockerize_amd64.sh
    print("./docker/dockerize_amd64.sh")
    subprocess.run(
        ["./docker/dockerize_amd64.sh"],
        cwd=PATH_PX4,
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(1)


import subprocess
import time
import os

def start_commander(axis, amplitude, hover_thrust, filename):
    # 1. Start the docker_run.sh script (silent)
    subprocess.Popen(
        ["sudo","./docker_run.sh"],
        cwd=PATH_Commander,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    # 2. Wait for container to start
    time.sleep(2)

    # 3. Find the container ID of the ros_imav container
    result = subprocess.run(
        ["sudo","docker", "ps", "-q", "--filter", "ancestor=elijahanghw/ros_imav:latest"],
        capture_output=True,
        text=True
    )
    container_id = result.stdout.strip()

    if not container_id:
        raise RuntimeError("Commander container did not start")

    # 4. Build the ros2 command
    ros2_cmd = ["sudo","docker", "exec", "-it", container_id,
                "ros2", "run", "mission_commander", "attitude_step", str(axis), str(amplitude), str(hover_thrust), str(filename)]

    # 5. Run the ROS2 command inside the container WITH terminal output
    return subprocess.Popen(
        ros2_cmd,
        cwd=PATH_Commander
    )


def export_run():
    pass

def compile_runs(folder):
    pass

def clear_runs(folder):
    pass

def one_run(axis, amplitude, file_name):

    #Preperation
    repo_pull()
    build_docker()
    
    #Start processes
    processes = {}
    processes["uxrce"] = start_uxrcedss()
    processes["px4"] = start_px4()
    processes["commander"] = start_commander(axis, amplitude, 0.7, file_name)


    #Wait
    time.sleep(120)

    #End processes


    #Recover data
    export_run()

def test_sweep(search_runs:int, search_axis:str, max_amplitude: float, min_amplitude :float|None = None):
    if min_amplitude is None:
        if max_amplitude > 0:
            min_amplitude = - max_amplitude
        else:
            raise ValueError("Max amplitude cannot be less or equal then 0 when min_amplitude is not assigned")
    for axis in search_axis:
        clear_runs(axis)
        for i in np.linspace(min_amplitude, max_amplitude, search_runs):
            one_run(axis, i, f"{axis}_{int(i*100)}_run")
        compile_runs(axis)


