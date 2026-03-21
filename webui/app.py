#!/usr/bin/env python3
"""
LiDAR Scanner Webinterface
Flask app for controlling LIO-SAM recording on Jetson TX2
Compatible with Python 3.5
"""

import os
import subprocess
import signal
import glob
import time
import threading
import io
from datetime import datetime
from flask import Flask, render_template, jsonify, send_file, Response

app = Flask(__name__)

DATA_DIR = "/media/nvidia/SSD1/lidar_data"
BAGS_DIR = os.path.join(DATA_DIR, "bags")
MAPS_DIR = os.path.join(DATA_DIR, "maps")
ROS_ENV = "/home/nvidia/miniforge3/envs/ros-noetic"

recording_process = None
slam_process = None
recording_start_time = None
recording_lock = threading.Lock()


def get_ros_env():
    env = os.environ.copy()
    env["CONDA_PREFIX"] = ROS_ENV
    env["PATH"] = "{}/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin".format(ROS_ENV)
    env["ROS_DISTRO"] = "noetic"
    env["ROS_ROOT"] = "{}/opt/ros/noetic/share/ros".format(ROS_ENV)
    env["ROS_PACKAGE_PATH"] = "{}/opt/ros/noetic/share".format(ROS_ENV)
    env["PYTHONPATH"] = "{0}/opt/ros/noetic/lib/python3/dist-packages:{0}/lib/python3.8/site-packages".format(ROS_ENV)
    env["LD_LIBRARY_PATH"] = "{0}/opt/ros/noetic/lib:{0}/lib".format(ROS_ENV)
    env["ROS_MASTER_URI"] = "http://localhost:11311"
    env["ROS_HOSTNAME"] = "localhost"
    return env


def run_cmd(cmd, timeout=2):
    try:
        result = subprocess.run(
            cmd, env=get_ros_env(), timeout=timeout,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        return result.returncode == 0
    except Exception:
        return False


def is_ros_running():
    return run_cmd(["rostopic", "list"], timeout=2)


def is_recording():
    global recording_process
    with recording_lock:
        if recording_process is None:
            return False
        return recording_process.poll() is None


def get_bags():
    bags = []
    for path in sorted(glob.glob(os.path.join(BAGS_DIR, "*.bag")), reverse=True):
        stat = os.stat(path)
        bags.append({
            "name": os.path.basename(path),
            "size_mb": round(stat.st_size / 1024.0 / 1024.0, 1),
            "mtime": datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M"),
        })
    return bags


def get_disk_info():
    try:
        stat = os.statvfs(DATA_DIR)
        free_gb = round(stat.f_frsize * stat.f_bavail / 1e9, 1)
        total_gb = round(stat.f_frsize * stat.f_blocks / 1e9, 1)
        used_pct = round(100 * (1 - float(stat.f_bavail) / stat.f_blocks))
        return {"free_gb": free_gb, "total_gb": total_gb, "used_pct": used_pct}
    except Exception:
        return {"free_gb": 0, "total_gb": 0, "used_pct": 0}


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    ros_ok = is_ros_running()
    rec = is_recording()
    duration = None
    if rec and recording_start_time:
        duration = int(time.time() - recording_start_time)
    imu_ok = False
    lidar_ok = False
    if ros_ok:
        imu_ok = run_cmd(["rostopic", "hz", "/imu/data", "--window=1"], timeout=1)
        lidar_ok = run_cmd(["rostopic", "hz", "/os_cloud_node/points", "--window=1"], timeout=1)
    return jsonify({
        "ros": ros_ok,
        "recording": rec,
        "duration_s": duration,
        "imu": imu_ok,
        "lidar": lidar_ok,
        "disk": get_disk_info(),
        "time": datetime.now().strftime("%H:%M:%S"),
    })


@app.route("/api/start", methods=["POST"])
def api_start():
    global recording_process, recording_start_time
    with recording_lock:
        if is_recording():
            return jsonify({"ok": False, "msg": "Already recording"})
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = os.path.join(BAGS_DIR, "scan_{}.bag".format(ts))
        try:
            recording_process = subprocess.Popen(
                ["rosbag", "record", "-O", bag_path,
                 "/imu/data", "/os_cloud_node/points", "/os_cloud_node/imu",
                 "/gps/fix", "/tf", "/tf_static"],
                env=get_ros_env(),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            recording_start_time = time.time()
            bname = os.path.basename(bag_path)
            return jsonify({"ok": True, "msg": "Recording started: {}".format(bname), "bag": bname})
        except Exception as e:
            return jsonify({"ok": False, "msg": str(e)})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    global recording_process, recording_start_time
    with recording_lock:
        if not is_recording():
            return jsonify({"ok": False, "msg": "Not recording"})
        try:
            recording_process.send_signal(signal.SIGINT)
            recording_process.wait(timeout=10)
            recording_process = None
            recording_start_time = None
            return jsonify({"ok": True, "msg": "Recording stopped"})
        except Exception as e:
            recording_process = None
            recording_start_time = None
            return jsonify({"ok": False, "msg": str(e)})


@app.route("/api/bags")
def api_bags():
    return jsonify(get_bags())


@app.route("/api/download/<filename>")
def api_download(filename):
    path = os.path.join(BAGS_DIR, filename)
    if not os.path.exists(path) or not filename.endswith(".bag"):
        return jsonify({"error": "Not found"}), 404
    return send_file(path, as_attachment=True)


@app.route("/api/snapshot")
def api_snapshot():
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return Response(b"", mimetype="image/png")

    WIDTH, HEIGHT, SCALE = 400, 400, 20
    img = Image.new("RGB", (WIDTH, HEIGHT), (20, 20, 30))
    draw = ImageDraw.Draw(img)

    for i in range(0, WIDTH, SCALE * 5):
        draw.line([(i, 0), (i, HEIGHT)], fill=(40, 40, 55), width=1)
    for i in range(0, HEIGHT, SCALE * 5):
        draw.line([(0, i), (WIDTH, i)], fill=(40, 40, 55), width=1)

    cx, cy = WIDTH // 2, HEIGHT // 2
    draw.line([(cx - 10, cy), (cx + 10, cy)], fill=(255, 80, 80), width=2)
    draw.line([(cx, cy - 10), (cx, cy + 10)], fill=(255, 80, 80), width=2)

    if is_ros_running():
        draw.text((10, 10), "ROS online", fill=(100, 255, 100))
    else:
        draw.text((10, 10), "ROS offline", fill=(255, 100, 100))

    draw.line([(10, HEIGHT - 20), (10 + SCALE * 5, HEIGHT - 20)], fill=(200, 200, 200), width=2)
    draw.text((10, HEIGHT - 15), "5m", fill=(200, 200, 200))
    draw.text((WIDTH - 80, HEIGHT - 15), datetime.now().strftime("%H:%M:%S"), fill=(150, 150, 150))

    buf = io.BytesIO()
    img.save(buf, "PNG")
    buf.seek(0)
    return send_file(buf, mimetype="image/png", cache_timeout=0)


@app.route("/api/launch_ros", methods=["POST"])
def api_launch_ros():
    env = get_ros_env()
    try:
        subprocess.Popen(["roscore"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)
        return jsonify({"ok": True, "msg": "ROS master gestartet"})
    except Exception as e:
        return jsonify({"ok": False, "msg": str(e)})


@app.route("/api/stop_ros", methods=["POST"])
def api_stop_ros():
    try:
        subprocess.call(["pkill", "-f", "roscore"])
        subprocess.call(["pkill", "-f", "rosmaster"])
        subprocess.call(["pkill", "-f", "roslaunch"])
        subprocess.call(["pkill", "-f", "rosbag"])
        return jsonify({"ok": True, "msg": "ROS gestoppt"})
    except Exception as e:
        return jsonify({"ok": False, "msg": str(e)})


if __name__ == "__main__":
    os.makedirs(BAGS_DIR, exist_ok=True)
    os.makedirs(MAPS_DIR, exist_ok=True)
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
