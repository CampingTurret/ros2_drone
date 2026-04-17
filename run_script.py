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
        cwd=PATH_Commander,
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    time.sleep(1)


def start_commander_container():
    # Build docker run command
    subprocess.run(["sudo", "docker", "rm", "-f", "ros_imav_container"])

    cmd = [
        "sudo", "docker", "run", "-dit",
        "--name", "ros_imav_container",
        "--network=host",
        "--privileged",
        "--device", "/dev/gpiomem4",
        "--device", "/dev/mem",
        "-v", f"{PATH_Commander}/logs:/root/logs",
        "elijahanghw/ros_imav:latest"
    ]

    # Start container
    result = subprocess.run(cmd, capture_output=True, text=True)
    container_id = result.stdout.strip()
    time.sleep(1)

    if not container_id:
        raise RuntimeError("Failed to start ros_imav_container")

    return container_id


def start_commander(axis, amplitude, hover_thrust, filename):
    container_id = start_commander_container()

    ros2_cmd = [
        "sudo", "docker", "exec", "-it", container_id,
        "ros2", "run", "mission_commander", "attitude_step",
        str(axis), str(amplitude), str(hover_thrust), str(filename)
    ]

    return subprocess.Popen(ros2_cmd), container_id


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


if __name__ == "__main__":
    print("Starting")
    one_run("roll", 0.17, "test")