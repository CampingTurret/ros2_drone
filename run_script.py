import subprocess
from pathlib import Path
import numpy as np
import time
import json
import shutil
import pandas as pd
import json
import os, signal

PATH_microxrcedds = Path(__file__).parents[1] / "Micro-XRCE-DDS-Agent"
PATH_PX4 = Path(__file__).parents[1] / "PX4-Autopilot"
PATH_Commander = Path(__file__).parent

def start_px4():
    # Command: make px4_sitl gz_x500
    print("make px4_sitl gz_x500")
    p = subprocess.Popen(
        ["make", "px4_sitl", "gz_x500"],
        cwd=PATH_PX4,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    time.sleep(5)
    return p

def start_uxrcedss():
    # Command: MicroXRCEAgent udp4 -p 8888
    print("MicroXRCEAgent udp4 -p 8888")
    p = subprocess.Popen(
        ["MicroXRCEAgent", "udp4", "-p", "8888"],
        cwd=PATH_microxrcedds,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    time.sleep(5)
    return p

def kill_process_tree(p):
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        p.wait(timeout=5)
    except Exception:
        os.killpg(os.getpgid(p.pid), signal.SIGKILL)


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

    cmd_str = (
    f"ros2 run mission_commander attitude_step "
    f"{axis} {amplitude} {hover_thrust} {filename}"
    )
    
    ros2_cmd = [
        "sudo", "docker", "exec", "-it", container_id,
        "bash", "-i", "-c", cmd_str
    ]

    return subprocess.Popen(ros2_cmd, preexec_fn=os.setsid), container_id

import shutil
import json
from datetime import datetime

def export_run(axis: str, filename: str, amplitude: float, hover_thrust: float):
    src = PATH_Commander / "logs" / f"{filename}.csv"
    dst_dir = PATH_Commander / "exports" / axis
    dst_dir.mkdir(parents=True, exist_ok=True)

    dst_csv = dst_dir / f"{filename}.csv"
    shutil.copy(src, dst_csv)

    meta = {
        "axis": axis,
        "amplitude": amplitude,
        "hover_thrust": hover_thrust,
        "filename": filename,
        "timestamp": datetime.now().isoformat()
    }

    with open(dst_dir / f"{filename}.json", "w") as f:
        json.dump(meta, f, indent=4)

    print(f"Exported run {dst_csv}")

def compile_runs(axis: str):
    export_path = PATH_Commander / "exports" / axis
    compiled_path = PATH_Commander / "compiled"
    compiled_path.mkdir(parents=True, exist_ok=True)

    compiled = {}

    for csv_file in sorted(export_path.glob("*.csv")):
        run_name = csv_file.stem

        # Load metadata
        meta_file = export_path / f"{run_name}.json"
        if not meta_file.exists():
            print(f"Missing metadata for {run_name}")
            continue

        with open(meta_file) as f:
            meta = json.load(f)

        # Load CSV
        df = pd.read_csv(csv_file)

        if "stage" not in df.columns:
            print(f"Warning: no 'stage' column in {csv_file.name}")
            continue

        # Filter to stage == 2
        df2 = df[df["stage"] == 2].copy()

        # Build JSON‑friendly structure
        compiled[run_name] = {
            "axis": meta["axis"],
            "amplitude": meta["amplitude"],
            "hover_thrust": meta["hover_thrust"],
            "timestamp": meta["timestamp"],
            "columns": list(df2.columns),
            "data": df2.values.tolist()
        }

        print(f"Added run {run_name} with {len(df2)} stage‑2 samples")

    # Save JSON
    out_file = compiled_path / f"{axis}_compiled.json"
    with open(out_file, "w") as f:
        json.dump(compiled, f, indent=4)

    print(f"Saved compiled JSON → {out_file}")

    return compiled

def clear_runs(axis: str):
    export_path = PATH_Commander / "exports" / axis
    if export_path.exists():
        shutil.rmtree(export_path)
    export_path.mkdir(parents=True, exist_ok=True)
    print(f"Cleared exported runs for axis {axis}")


def one_run(axis, amplitude, file_name):

    #Preperation
    ensure_sudo()
    repo_pull()
    build_docker()
    
    #Start processes
    processes = {}
    processes["uxrce"] = start_uxrcedss()
    processes["px4"] = start_px4()
    processes["commander"], _ = start_commander(axis, amplitude, 0.8, file_name)


    #Wait
    time.sleep(120)


    kill_process_tree(processes["px4"])
    kill_process_tree(processes["uxrce"])
    kill_process_tree(processes["commander"])

    #End processes
    #processes["px4"].terminate()
    #processes["px4"].wait(timeout=5)
    #processes["uxrce"].terminate()
    #processes["uxrce"].wait(timeout=5)
    #processes["commander"].kill()
    subprocess.run(["sudo", "docker", "stop", "ros_imav_container"])


    #Recover data
    export_run(axis, file_name, amplitude, 0.8)

def test_sweep(search_runs:int, search_axis:list[str], max_amplitude: float, min_amplitude :float|None = None):
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

def ensure_sudo():
    print("Requesting sudo password...")
    subprocess.run(["sudo", "-v"])  # triggers password prompt early

import subprocess, threading, time

def keep_sudo_alive():
    while True:
        subprocess.run(["sudo", "-v"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(60)  # refresh every minute

# Start refresher thread
threading.Thread(target=keep_sudo_alive, daemon=True).start()



if __name__ == "__main__":
    print("Starting")
    ensure_sudo()
    threading.Thread(target=keep_sudo_alive, daemon=True).start()
    one_run("roll", 10, "test")
    one_run("roll", 10, "test2")
    one_run("roll", 10, "test3")
    one_run("roll", 10, "test4")
    one_run("roll", 10, "test5")
    compile_runs("roll")