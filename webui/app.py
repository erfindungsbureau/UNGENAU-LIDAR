#!/usr/bin/env python3
"""
UNGENAU LiDAR Scanner - Web Interface
Jetson TX2 / ROS Melodic / Ouster OS1-64 / SBG Ellipse 2
"""
import os, re, subprocess, signal, glob, time, threading, json
from datetime import datetime
from flask import Flask, render_template, jsonify, send_file, Response, request

app = Flask(__name__, template_folder="templates", static_folder="static")

DATA_DIR  = "/media/nvidia/SSD/lidar_data"
BAGS_DIR  = os.path.join(DATA_DIR, "bags")
MAPS_DIR  = os.path.join(DATA_DIR, "maps")
ROS_SETUP = "/opt/ros/melodic/setup.bash"

OUSTER_IP   = "192.168.2.2"
OUSTER_PORT = 7501

def ouster_cmd(cmd, timeout=3):
    import socket as _sock, time as _time
    try:
        s = _sock.socket(_sock.AF_INET, _sock.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect((OUSTER_IP, OUSTER_PORT))
        s.sendall((cmd + "\n").encode())
        _time.sleep(0.4)
        data = b""
        s.settimeout(1)
        try:
            while True:
                c = s.recv(4096)
                if not c: break
                data += c
        except: pass
        s.close()
        return data.decode().strip()
    except Exception as e:
        return "error: " + str(e)

def ouster_set_mode(mode):
    r1 = ouster_cmd("set_config_param operating_mode " + mode)
    r2 = ouster_cmd("reinitialize")
    return r1, r2

WS_SETUP  = "/home/nvidia/catkin_ws/devel/setup.bash"

recording_process    = None
recording_start_time = None
recording_lock       = threading.Lock()

slam_process   = None
slam_lock      = threading.Lock()
slam_stop_timer = None
slam_stop_lock  = threading.Lock()
slam_engine    = "lio_sam"   # "lio_sam" | "fast_lio"

SLAM_LAUNCHES = {
    "lio_sam":  "/home/nvidia/UNGENAU-LIDAR/ros/launch/slam_lio_sam.launch",
    "fast_lio": "/home/nvidia/UNGENAU-LIDAR/ros/launch/slam_fast_lio_sbg.launch",
}

export_process  = None
export_lock     = threading.Lock()
export_timeout  = 1800  # 30 Minuten max (HD Export kann lang dauern)
export_xyz_path = None   # aktuell geschriebene XYZ-Datei
export_bag_size = 0      # Bag-Groesse als Schaetzbasis

# ─── ARM / DISARM ─────────────────────────────────────────────────────────────
arm_state       = "disarmed"   # disarmed | arming | armed | disarming
arm_lock        = threading.Lock()
arm_auto_timer  = None
arm_timer_lock  = threading.Lock()
arm_timer_start = None         # Zeitpunkt ARM gesetzt (fuer Countdown)

# Diagnose-Modus: deaktiviert automatisches DISARM (Startup-Safety + 120s-Timeout).
# Datei-basiert (nicht nur In-Memory), damit der Modus auch ueberlebt wenn Flask
# waehrend Tests mehrfach neu gestartet wird (genau dafuer gedacht).
DIAGNOSTIC_FLAG_FILE = "/tmp/diagnostic_mode"

def is_diagnostic_mode():
    return os.path.exists(DIAGNOSTIC_FLAG_FILE)


def _cancel_arm_timer():
    global arm_auto_timer
    with arm_timer_lock:
        if arm_auto_timer:
            arm_auto_timer.cancel()
            arm_auto_timer = None

def _start_arm_timer():
    global arm_auto_timer, arm_timer_start
    _cancel_arm_timer()
    if is_diagnostic_mode():
        return
    with arm_timer_lock:
        arm_timer_start = time.time()
        def _do_auto_disarm():
            global arm_state, arm_auto_timer
            with arm_lock:
                if arm_state != "armed":
                    return
                arm_state = "disarming"
            with arm_timer_lock:
                arm_auto_timer = None
            t = threading.Thread(target=_disarm_sequence, daemon=True)
            t.start()
        arm_auto_timer = threading.Timer(200.0, _do_auto_disarm)
        arm_auto_timer.daemon = True
        arm_auto_timer.start()

def _stop_ouster_driver():
    patterns = ["ouster_ros", "os_nodelet", "relay_lidar", "relay_imu",
                "launch/ouster.launch", "ouster_restamp"]
    for p in patterns:
        subprocess.run(["pkill", "-f", p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)

# Zeitbasis des Ouster: "ros" = Jetson-Uhr (bisher), "pps" = SBG-PPS (+ ouster_restamp.py).
# Datei statt Variable, damit die Wahl einen Neustart von Flask/Jetson ueberlebt.
TIME_MODE_FILE = "/home/nvidia/config/time_mode"

def get_time_mode():
    try:
        with open(TIME_MODE_FILE) as f:
            return "pps" if f.read().strip() == "pps" else "ros"
    except OSError:
        return "ros"

def _start_ouster_driver():
    mode = get_time_mode()
    if mode == "ros" and "SYNC_PULSE_IN" in ouster_cmd("get_config_param active timestamp_mode"):
        # Sensor steht noch vom PPS-Betrieb auf SYNC_PULSE_IN -> zurueck auf interne Uhr
        ouster_cmd("set_config_param timestamp_mode TIME_FROM_INTERNAL_OSC")
        ouster_cmd("reinitialize")
    env = {**os.environ, "ROS_MASTER_URI": "http://localhost:11311", "ROS_HOSTNAME": "localhost"}
    ros_src = ("source /opt/ros/melodic/setup.bash && "
               "source /home/nvidia/catkin_ws/devel/setup.bash && ")
    subprocess.Popen(
        ["bash", "-c", ros_src +
         "roslaunch /home/nvidia/UNGENAU-LIDAR/ros/launch/ouster.launch time_mode:=" + mode +
         " >> /tmp/ouster.log 2>&1"],
        env=env)
    if mode == "pps":
        subprocess.Popen(
            ["bash", "-c", ros_src +
             "python /home/nvidia/UNGENAU-LIDAR/scripts/ouster_restamp.py >> /tmp/ouster_restamp.log 2>&1"],
            env=env)

def _arm_sequence():
    """Laeuft im Hintergrund: LiDAR einschalten → warten → SLAM starten → warten → armed."""
    global arm_state, slam_process
    ouster_set_mode("NORMAL")
    _stop_ouster_driver()
    time.sleep(1)
    _start_ouster_driver()
    # Warten auf lidar_ok (max 60s; nach Zeitmodus-Wechsel konfiguriert sich der Ouster ~35s neu)
    for _ in range(60):
        with arm_lock:
            if arm_state != "arming":
                return
        if _bridge_status().get("lidar_ok"):
            break
        time.sleep(1)
    else:
        with arm_lock:
            arm_state = "disarmed"
        return
    # SLAM starten
    if not is_slam_process_running():
        launch = SLAM_LAUNCHES.get(slam_engine, SLAM_LAUNCHES["lio_sam"])
        cmd = ("source /opt/ros/melodic/setup.bash && "
               "source /home/nvidia/catkin_ws/devel/setup.bash && "
               "roslaunch " + launch)
        proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdin=subprocess.DEVNULL,
            stdout=open("/tmp/slam.log", "w"),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        with slam_lock:
            slam_process = proc
    # Warten auf slam_ok (max 40s)
    for _ in range(40):
        with arm_lock:
            if arm_state != "arming":
                return
        if _bridge_status().get("slam_ok"):
            break
        time.sleep(1)
    else:
        with arm_lock:
            arm_state = "disarmed"
        return
    with arm_lock:
        arm_state = "armed"
    _start_arm_timer()

def _disarm_sequence():
    """Stoppt Aufnahme, SLAM, ouster_ros Driver und setzt LiDAR auf STANDBY."""
    global arm_state, recording_process, recording_start_time
    # Aufnahme stoppen
    if is_recording():
        try:
            os.killpg(os.getpgid(recording_process.pid), signal.SIGINT)
            recording_process.wait(timeout=5)
        except Exception:
            pass
        with recording_lock:
            recording_process    = None
            recording_start_time = None
    # SLAM stoppen
    kill_slam_all()
    # ouster_ros stoppen
    _stop_ouster_driver()
    # LiDAR STANDBY
    ouster_set_mode("STANDBY")
    with arm_lock:
        arm_state = "disarmed"

def _startup_disarm():
    """Beim Flask-Start: LiDAR STANDBY sicherstellen (uebersprungen im Diagnose-Modus)."""
    time.sleep(5)
    if is_diagnostic_mode():
        return
    ouster_set_mode("STANDBY")
    _stop_ouster_driver()

threading.Thread(target=_startup_disarm, daemon=True).start()

# ─── Hilfsfunktionen ──────────────────────────────────────────────────────────

def _bridge_status():
    try:
        with open("/tmp/bridge_status.json") as f:
            return json.load(f)
    except Exception:
        return {"lidar_ok": False, "slam_ok": False, "still": False}

def is_lidar_publishing():
    return _bridge_status().get("lidar_ok", False)

def is_slam_active():
    """Prüft ob cloud_registered publiziert (Wahrheit über bridge_status)."""
    return _bridge_status().get("slam_ok", False)

def is_recording():
    with recording_lock:
        return recording_process is not None and recording_process.poll() is None

def is_slam_process_running():
    """Prüft ob ein slam.launch Prozess läuft (Flask-intern oder extern)."""
    with slam_lock:
        if slam_process is not None and slam_process.poll() is None:
            return True
    # Auch extern gestartete SLAM-Prozesse erkennen
    r = subprocess.run(["pgrep", "-f", "UNGENAU-LIDAR/ros/launch/slam_"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return r.returncode == 0

def is_export_running():
    with export_lock:
        return export_process is not None and export_process.poll() is None

def kill_slam_all():
    """Beendet alle SLAM-Prozesse sauber, egal wie gestartet."""
    global slam_process
    patterns = [
        "UNGENAU-LIDAR/ros/launch/slam_",
        "fastlio_mapping",
        "laserMapping",
        "lio_sam_imuPreintegration",
        "lio_sam_imageProjection",
        "lio_sam_featureExtraction",
        "lio_sam_mapOptmization",
        "imu_filter_node",
        "imu_relay",
    ]
    # SIGINT (sauber)
    for p in patterns:
        subprocess.run(["pkill", "-SIGINT", "-f", p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3)
    # SIGKILL (falls nötig)
    for p in patterns:
        subprocess.run(["pkill", "-9", "-f", p], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with slam_lock:
        slam_process = None

def get_bags():
    bags = []
    for path in sorted(glob.glob(os.path.join(BAGS_DIR, "*.bag")), reverse=True):
        s    = os.stat(path)
        base = os.path.basename(path)
        stem = base[:-4]  # ohne .bag
        xyz_path  = os.path.join(BAGS_DIR, stem + ".xyz")
        hires_path = os.path.join(BAGS_DIR, stem + "_hires.xyz")
        bags.append({
            "name":       base,
            "size_mb":    round(s.st_size / 1024.0 / 1024.0, 1),
            "mtime":      datetime.fromtimestamp(s.st_mtime).strftime("%Y-%m-%d %H:%M"),
            "has_xyz":    os.path.exists(xyz_path),
            "has_hires":  os.path.exists(hires_path),
            "xyz_mb":     round(os.stat(xyz_path).st_size  / 1024.0 / 1024.0, 1) if os.path.exists(xyz_path)   else 0,
            "hires_mb":   round(os.stat(hires_path).st_size / 1024.0 / 1024.0, 1) if os.path.exists(hires_path) else 0,
        })
    return bags

def get_disk_info():
    try:
        s = os.statvfs(DATA_DIR)
        free_gb  = round(s.f_frsize * s.f_bavail / 1e9, 1)
        total_gb = round(s.f_frsize * s.f_blocks / 1e9, 1)
        used_pct = round(100 * (1 - float(s.f_bavail) / s.f_blocks))
        return {"free_gb": free_gb, "total_gb": total_gb, "used_pct": used_pct}
    except Exception:
        return {"free_gb": 0, "total_gb": 0, "used_pct": 0}

# ── USB ──────────────────────────────────────────────────────────────────────
usb_copy_state = {"active": False, "file": None, "copied": 0, "total": 0, "done": False, "error": None}
usb_copy_lock  = threading.Lock()

def get_usb_mount():
    try:
        r = subprocess.run(["lsblk", "-J", "-o", "NAME,TYPE,MOUNTPOINT,HOTPLUG,TRAN,LABEL,SIZE"],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        import json as _json
        d = _json.loads(r.stdout)
        for dev in d.get("blockdevices", []):
            if dev.get("tran") != "usb":
                continue
            # Partitionen pruefen
            for child in dev.get("children", []) or []:
                mp = child.get("mountpoint")
                if mp:
                    return {"mount": mp, "label": child.get("label") or dev.get("label") or "USB",
                            "size": child.get("size", "?")}
            # Direkt gemountet
            mp = dev.get("mountpoint")
            if mp:
                return {"mount": mp, "label": dev.get("label") or "USB", "size": dev.get("size", "?")}
    except Exception:
        pass
    return None

def _copy_file_to_usb(src, dst_dir, filename):
    global usb_copy_state
    dst = os.path.join(dst_dir, filename)
    size = os.path.getsize(src)
    with usb_copy_lock:
        usb_copy_state.update({"active": True, "file": filename, "copied": 0,
                               "total": size, "done": False, "error": None})
    try:
        with open(src, "rb") as fin, open(dst, "wb") as fout:
            while True:
                chunk = fin.read(1024 * 1024)
                if not chunk:
                    break
                fout.write(chunk)
                with usb_copy_lock:
                    usb_copy_state["copied"] += len(chunk)
        with usb_copy_lock:
            usb_copy_state["done"]   = True
            usb_copy_state["active"] = False
    except Exception as e:
        with usb_copy_lock:
            usb_copy_state["error"]  = str(e)
            usb_copy_state["active"] = False
        try: os.remove(dst)
        except: pass


@app.route("/api/usb/status")
def api_usb_status():
    usb = get_usb_mount()
    if not usb:
        return jsonify({"connected": False})
    try:
        s = os.statvfs(usb["mount"])
        free_gb = round(s.f_frsize * s.f_bavail / 1e9, 1)
    except:
        free_gb = 0
    with usb_copy_lock:
        state = dict(usb_copy_state)
    return jsonify({"connected": True, "mount": usb["mount"], "label": usb["label"],
                    "size": usb["size"], "free_gb": free_gb, "copy": state})


@app.route("/api/usb/copy/<filename>", methods=["POST"])
def api_usb_copy(filename):
    if "/" in filename:
        return jsonify({"ok": False, "msg": "Ungueltig"}), 400
    with usb_copy_lock:
        if usb_copy_state["active"]:
            return jsonify({"ok": False, "msg": "Kopiervorgang laeuft bereits"})
    usb = get_usb_mount()
    if not usb:
        return jsonify({"ok": False, "msg": "Kein USB-Stick erkannt"})
    src = os.path.join(BAGS_DIR, filename)
    if not os.path.exists(src):
        return jsonify({"ok": False, "msg": "Datei nicht gefunden"}), 404
    t = threading.Thread(target=_copy_file_to_usb,
                         args=(src, usb["mount"], filename), daemon=True)
    t.start()
    return jsonify({"ok": True, "file": filename, "dest": usb["mount"]})


def sanitize_name(name):
    name = re.sub(r'[^\w\- ]', '', name).strip().replace(' ', '_')
    return name[:80] if name else None

def export_watchdog(proc, xyz_path, timeout):
    """Background-Thread: killt Export nach Timeout, löscht unfertige Datei."""
    start = time.time()
    while proc.poll() is None:
        if time.time() - start > timeout:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
            # Unfertige XYZ-Datei entfernen
            try:
                if os.path.exists(xyz_path):
                    os.remove(xyz_path)
            except Exception:
                pass
            break
        time.sleep(5)
    global export_process, export_xyz_path
    with export_lock:
        export_process  = None
        export_xyz_path = None


# ─── Routes ───────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    from flask import make_response
    r = make_response(render_template("lidar_viewer.html"))
    r.headers["Cache-Control"] = "no-cache, no-store"
    return r


@app.route("/api/diagnostic/enable", methods=["POST"])
def api_diagnostic_enable():
    open(DIAGNOSTIC_FLAG_FILE, "w").close()
    _cancel_arm_timer()
    return jsonify({"ok": True, "diagnostic_mode": True,
                     "msg": "Diagnose-Modus aktiv: kein Auto-DISARM (Startup-Safety + 120s-Timeout deaktiviert)"})


@app.route("/api/diagnostic/disable", methods=["POST"])
def api_diagnostic_disable():
    try:
        os.remove(DIAGNOSTIC_FLAG_FILE)
    except FileNotFoundError:
        pass
    return jsonify({"ok": True, "diagnostic_mode": False,
                     "msg": "Diagnose-Modus deaktiviert: Auto-DISARM wieder normal aktiv"})


@app.route("/api/lidar/off", methods=["POST"])
def api_lidar_off():
    if is_recording():
        return jsonify({"ok": False, "msg": "Scan laeuft — zuerst stoppen"})
    # SLAM stoppen
    kill_slam_all()
    # LiDAR in Standby
    r1, r2 = ouster_set_mode("STANDBY")
    ok = "error" not in r1.lower()
    if ok:
        msg = "LiDAR wird gestoppt..."
    elif "timed out" in r1 or "No route" in r1 or "refused" in r1:
        msg = "LiDAR nicht erreichbar — laeuft er gerade?"
    else:
        msg = r1.replace("error: ", "")
    return jsonify({"ok": ok, "msg": msg, "set": r1, "reinit": r2})


@app.route("/api/lidar/on", methods=["POST"])
def api_lidar_on():
    # Sensor aus STANDBY holen (falls erreichbar)
    r1, r2 = ouster_set_mode("NORMAL")
    sensor_ok = "error" not in r1.lower()

    # ouster_ros Driver immer neu starten (stirbt nach STANDBY).
    # WICHTIG: pkill in einem EIGENEN Subprocess-Aufruf, getrennt vom roslaunch-Befehl.
    # Grund: wenn beide im selben "bash -c"-String stehen, enthaelt die cmdline dieses
    # bash-Prozesses selbst den Suchstring "ouster_ros" (aus dem spaeteren roslaunch-Teil)
    # -> pkill toetet dann seinen eigenen Elternprozess mitten im Start (race condition).
    # Läuft im Hintergrund-Thread, damit der Request sofort zurückkehrt (wie zuvor).
    def _restart_ouster():
        _stop_ouster_driver()
        time.sleep(1)
        _start_ouster_driver()
    threading.Thread(target=_restart_ouster, daemon=True).start()
    msg = "LiDAR startet (~15s)..."
    return jsonify({"ok": True, "msg": msg, "set": r1, "reinit": r2})


@app.route("/api/time_mode", methods=["GET", "POST"])
def api_time_mode():
    if request.method == "POST":
        mode = (request.get_json(silent=True) or {}).get("mode", "")
        if mode not in ("ros", "pps"):
            return jsonify({"ok": False, "msg": "Unbekannter Zeitmodus"})
        if arm_state != "disarmed" or is_recording():
            return jsonify({"ok": False, "msg": "Erst DISARM, dann Zeitmodus wechseln"})
        os.makedirs(os.path.dirname(TIME_MODE_FILE), exist_ok=True)
        with open(TIME_MODE_FILE, "w") as f:
            f.write(mode + "\n")
        if is_lidar_publishing():  # laufenden Treiber sofort im neuen Modus neu starten
            def _restart():
                _stop_ouster_driver()
                time.sleep(1)
                _start_ouster_driver()
            threading.Thread(target=_restart, daemon=True).start()
    try:
        locked = bool(json.loads(ouster_cmd("get_time_info"))["sync_pulse_in"]["locked"])
    except Exception:
        locked = None
    return jsonify({"ok": True, "mode": get_time_mode(), "pps_locked": locked})


@app.route("/api/lidar/mode")
def api_lidar_mode():
    r = ouster_cmd("get_config_param active")
    try:
        import json as _json
        d = _json.loads(r)
        return jsonify({"ok": True, "mode": d.get("operating_mode", "?")})
    except:
        return jsonify({"ok": False, "mode": "unknown"})


@app.route("/api/status")
def api_status():
    rec      = is_recording()
    duration = int(time.time() - recording_start_time) if rec and recording_start_time else None
    lidar    = is_lidar_publishing()
    imu      = os.path.exists("/tmp/imu.json")
    slam_act = is_slam_active()
    return jsonify({
        "ros":          True,
        "lidar":        lidar,
        "imu":          imu,
        "recording":    rec,
        "duration_s":   duration,
        "slam":         is_slam_process_running(),
        "slam_active":  slam_act,
        "scan_ready":   lidar and imu and slam_act,
        "imu_still":    _bridge_status().get("still", False),
        "arm_state":    arm_state,
        "arm_secs_left": max(0, int(200 - (time.time() - arm_timer_start))) if arm_timer_start and arm_state == "armed" else None,
        "export_running": is_export_running(),
        "slam_engine":  slam_engine,
        "diagnostic_mode": is_diagnostic_mode(),
        "disk":         get_disk_info(),
        "time":         datetime.now().strftime("%H:%M:%S"),
    })


@app.route("/api/scan_preview")
def api_scan_preview():
    try:
        with open("/tmp/scan_2d.json") as f:
            data = f.read()
        resp = Response(data, mimetype="application/json")
        resp.headers["Cache-Control"] = "no-cache, no-store"
        return resp
    except Exception:
        return jsonify({"pts": [], "t": 0})


@app.route("/api/imu")
def api_imu():
    try:
        with open("/tmp/imu.json") as f:
            data = f.read()
        resp = Response(data, mimetype="application/json")
        resp.headers["Cache-Control"] = "no-cache, no-store"
        return resp
    except Exception:
        return jsonify({"roll": 0, "pitch": 0, "yaw": 0})


@app.route("/api/record/start", methods=["POST"])
def api_record_start():
    global recording_process, recording_start_time
    if is_recording():
        return jsonify({"ok": False, "msg": "Läuft bereits"})
    if not is_slam_active():
        return jsonify({"ok": False, "msg": "SLAM nicht aktiv – warten bis SLAM bereit"})
    os.makedirs(BAGS_DIR, exist_ok=True)
    ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_path = os.path.join(BAGS_DIR, "scan_{}.bag".format(ts))
    try:
        env = os.environ.copy()
        env.setdefault("ROS_MASTER_URI", "http://localhost:11311")
        cmd = (
            "source {ros} && "
            "rosbag record -O {bag} "
            "/ouster/points /ouster/imu /imu/data "  # /imu/data = SBG, fuer SLAM-Neuberechnung am Computer
            "/lio_sam/mapping/cloud_registered "
            "/lio_sam/mapping/odometry "
            "/tf /tf_static"
        ).format(ros=ROS_SETUP, bag=bag_path)
        proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
            preexec_fn=os.setsid,
        )
        with recording_lock:
            recording_process    = proc
            recording_start_time = time.time()
        return jsonify({"ok": True, "bag": os.path.basename(bag_path)})
    except Exception as e:
        return jsonify({"ok": False, "msg": str(e)})


@app.route("/api/record/stop", methods=["POST"])
def api_record_stop():
    global recording_process, recording_start_time
    if not is_recording():
        return jsonify({"ok": False, "msg": "Nicht aktiv"})
    try:
        os.killpg(os.getpgid(recording_process.pid), signal.SIGINT)
        recording_process.wait(timeout=10)
    except Exception:
        try:
            os.killpg(os.getpgid(recording_process.pid), signal.SIGKILL)
        except Exception:
            pass
    with recording_lock:
        recording_process    = None
        recording_start_time = None
    return jsonify({"ok": True})


@app.route("/api/slam/start", methods=["POST"])
def api_slam_start():
    global slam_process, slam_engine
    if is_slam_process_running():
        return jsonify({"ok": False, "msg": "SLAM läuft bereits"})
    data   = request.get_json(silent=True) or {}
    engine = data.get("engine", slam_engine)
    if engine in SLAM_LAUNCHES:
        slam_engine = engine
    launch = SLAM_LAUNCHES.get(slam_engine, SLAM_LAUNCHES["lio_sam"])
    try:
        cmd = (
            "source /opt/ros/melodic/setup.bash && "
            "source /home/nvidia/catkin_ws/devel/setup.bash && "
            "roslaunch " + launch
        )
        proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdin=subprocess.DEVNULL,
            stdout=open("/tmp/slam.log", "w"),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        with slam_lock:
            slam_process = proc
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "msg": str(e)})


@app.route("/api/slam/engine", methods=["GET", "POST"])
def api_slam_engine():
    global slam_engine
    if request.method == "POST":
        data   = request.get_json(silent=True) or {}
        engine = data.get("engine", "")
        if engine in SLAM_LAUNCHES:
            slam_engine = engine
            return jsonify({"ok": True, "engine": slam_engine})
        return jsonify({"ok": False, "msg": "Unbekannte Engine"})
    return jsonify({"engine": slam_engine})


@app.route("/api/slam/stop", methods=["POST"])
def api_slam_stop():
    if is_recording():
        return jsonify({"ok": False, "msg": "Scan laeuft - zuerst Aufnahme stoppen"})
    global slam_stop_timer
    with slam_stop_lock:
        if slam_stop_timer:
            slam_stop_timer.cancel()
            slam_stop_timer = None
    kill_slam_all()
    return jsonify({"ok": True})


@app.route("/api/slam/schedule_stop", methods=["POST"])
def api_slam_schedule_stop():
    global slam_stop_timer
    with slam_stop_lock:
        if slam_stop_timer:
            slam_stop_timer.cancel()
        def _do_stop():
            global slam_stop_timer
            kill_slam_all()
            with slam_stop_lock:
                slam_stop_timer = None
        slam_stop_timer = threading.Timer(30.0, _do_stop)
        slam_stop_timer.daemon = True
        slam_stop_timer.start()
    return jsonify({"ok": True, "stop_in_s": 30})


@app.route("/api/slam/cancel_stop", methods=["POST"])
def api_slam_cancel_stop():
    global slam_stop_timer
    with slam_stop_lock:
        if slam_stop_timer:
            slam_stop_timer.cancel()
            slam_stop_timer = None
    return jsonify({"ok": True})


@app.route("/api/bags")
def api_bags():
    return jsonify(get_bags())


@app.route("/api/bags/<filename>", methods=["DELETE"])
def api_delete_bag(filename):
    if "/" in filename or not filename.endswith(".bag"):
        return jsonify({"ok": False, "msg": "Ungültig"}), 400
    path = os.path.join(BAGS_DIR, filename)
    if not os.path.exists(path):
        return jsonify({"ok": False, "msg": "Nicht gefunden"}), 404
    os.remove(path)
    # zugehörige XYZ-Datei auch löschen
    xyz = path.replace(".bag", ".xyz")
    if os.path.exists(xyz):
        os.remove(xyz)
    return jsonify({"ok": True})


@app.route("/api/bags/<filename>/rename", methods=["POST"])
def api_rename_bag(filename):
    if "/" in filename or not filename.endswith(".bag"):
        return jsonify({"ok": False, "msg": "Ungültig"}), 400
    data     = request.get_json() or {}
    new_name = sanitize_name(data.get("name", ""))
    if not new_name:
        return jsonify({"ok": False, "msg": "Kein Name"}), 400
    if not new_name.endswith(".bag"):
        new_name += ".bag"
    src = os.path.join(BAGS_DIR, filename)
    dst = os.path.join(BAGS_DIR, new_name)
    if not os.path.exists(src):
        return jsonify({"ok": False, "msg": "Nicht gefunden"}), 404
    os.rename(src, dst)
    # XYZ-Datei mitumbenennen falls vorhanden
    src_xyz = src.replace(".bag", ".xyz")
    dst_xyz = dst.replace(".bag", ".xyz")
    if os.path.exists(src_xyz):
        os.rename(src_xyz, dst_xyz)
    return jsonify({"ok": True, "name": new_name})


def _start_export(bag_path, xyz_path, script, sec_arg):
    """Startet Export-Prozess und Watchdog-Thread. Gibt (proc, err) zurueck."""
    global export_process, export_xyz_path, export_bag_size
    export_xyz_path = xyz_path
    export_bag_size = os.path.getsize(bag_path) if os.path.exists(bag_path) else 0
    cmd = (
        "source {ros} && "
        "python2 {script} {bag} {xyz} 1 {sec}"
        " > /tmp/export.log 2>&1"
    ).format(ros=ROS_SETUP, script=script, bag=bag_path, xyz=xyz_path, sec=sec_arg)
    try:
        proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdin=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        with export_lock:
            export_process = proc
        t = threading.Thread(
            target=export_watchdog,
            args=(proc, xyz_path, export_timeout),
            daemon=True,
        )
        t.start()
        return proc, None
    except Exception as e:
        return None, str(e)


@app.route("/api/bags/<filename>/export", methods=["POST"])
def api_export_bag(filename):
    if "/" in filename or not filename.endswith(".bag"):
        return jsonify({"ok": False, "msg": "Ungueltig"}), 400
    if is_export_running():
        return jsonify({"ok": False, "msg": "Export laeuft bereits"})
    bag_path = os.path.join(BAGS_DIR, filename)
    if not os.path.exists(bag_path):
        return jsonify({"ok": False, "msg": "Nicht gefunden"}), 404
    data    = request.get_json(silent=True) or {}
    max_sec = data.get("max_sec", "")
    sec_arg = str(float(max_sec)) if max_sec else ""
    xyz_name = filename.replace(".bag", ".xyz")
    xyz_path = os.path.join(BAGS_DIR, xyz_name)
    if os.path.exists(xyz_path):
        os.remove(xyz_path)
    proc, err = _start_export(bag_path, xyz_path, "/home/nvidia/UNGENAU-LIDAR/scripts/bag_to_xyz.py", sec_arg)
    if err:
        return jsonify({"ok": False, "msg": err})
    return jsonify({"ok": True, "file": xyz_name})


@app.route("/api/bags/<filename>/export_hires", methods=["POST"])
def api_export_bag_hires(filename):
    if "/" in filename or not filename.endswith(".bag"):
        return jsonify({"ok": False, "msg": "Ungueltig"}), 400
    if is_export_running():
        return jsonify({"ok": False, "msg": "Export laeuft bereits"})
    bag_path = os.path.join(BAGS_DIR, filename)
    if not os.path.exists(bag_path):
        return jsonify({"ok": False, "msg": "Nicht gefunden"}), 404
    data    = request.get_json(silent=True) or {}
    max_sec = data.get("max_sec", "")
    sec_arg = str(float(max_sec)) if max_sec else ""
    xyz_name = filename.replace(".bag", "_hires.xyz")
    xyz_path = os.path.join(BAGS_DIR, xyz_name)
    if os.path.exists(xyz_path):
        os.remove(xyz_path)
    proc, err = _start_export(bag_path, xyz_path, "/home/nvidia/UNGENAU-LIDAR/scripts/bag_to_xyz_hires.py", sec_arg)
    if err:
        return jsonify({"ok": False, "msg": err})
    return jsonify({"ok": True, "file": xyz_name})


@app.route("/api/export/progress")
def api_export_progress():
    with export_lock:
        running  = export_process is not None and export_process.poll() is None
        xyz_path = export_xyz_path
        bag_size = export_bag_size
    bytes_written = 0
    if xyz_path and os.path.exists(xyz_path):
        bytes_written = os.path.getsize(xyz_path)
    # Schaetze Gesamtgroesse: HD ~35% der Bag-Groesse, Standard ~0.6%
    is_hires = xyz_path and "_hires" in xyz_path if xyz_path else False
    ratio = 0.35 if is_hires else 0.006
    est_total = int(bag_size * ratio) if bag_size else 0
    pct = min(99, int(bytes_written * 100.0 / est_total)) if est_total > 0 else 0
    return jsonify({
        "running":       running,
        "bytes_written": bytes_written,
        "mb_written":    round(bytes_written / 1024.0 / 1024.0, 1),
        "est_total_mb":  round(est_total / 1024.0 / 1024.0, 1),
        "pct":           pct,
        "file":          os.path.basename(xyz_path) if xyz_path else None,
    })


@app.route("/api/processes")
def api_processes():
    """Listet laufende Konvertierungsprozesse (bag_to_xyz*)."""
    allowed = ["bag_to_xyz"]
    procs = []
    try:
        r = subprocess.run(["ps", "aux"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        for line in r.stdout.decode().splitlines():
            if not any(p in line for p in allowed):
                continue
            if "grep" in line or "ps aux" in line:
                continue
            parts = line.split(None, 10)
            if len(parts) < 11:
                continue
            pid  = int(parts[1])
            # Kommandozeile aus /proc lesen fuer Dateinamen
            try:
                with open("/proc/%d/cmdline" % pid, "rb") as f:
                    cmd_args = f.read().split(b"")
                cmd_str = " ".join(a.decode("utf-8", errors="replace") for a in cmd_args if a)
            except:
                cmd_str = parts[10]
            # Input/Output Dateinamen extrahieren
            bag_name = xyz_name = ""
            for arg in cmd_str.split():
                if arg.endswith(".bag"):
                    bag_name = os.path.basename(arg)
                elif arg.endswith(".xyz"):
                    xyz_name = os.path.basename(arg)
            is_hires = "hires" in cmd_str
            procs.append({
                "pid":      pid,
                "cpu":      parts[2],
                "mem":      parts[3],
                "elapsed":  parts[9],
                "bag":      bag_name,
                "xyz":      xyz_name,
                "hires":    is_hires,
            })
    except Exception as e:
        pass
    return jsonify(procs)


@app.route("/api/processes/<int:pid>/kill", methods=["POST"])
def api_kill_process(pid):
    # Sicherheit: nur bag_to_xyz Prozesse erlaubt
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as f:
            cmdline = f.read().replace(b"\x00", b" ").decode("utf-8", errors="replace")
        if "bag_to_xyz" not in cmdline:
            return jsonify({"ok": False, "msg": "Nicht ein Konvertierungsprozess"})
        try:
            os.killpg(os.getpgid(pid), signal.SIGKILL)
        except ProcessLookupError:
            os.kill(pid, signal.SIGKILL)
        # Export-State zuruecksetzen
        global export_process, export_xyz_path
        with export_lock:
            export_process  = None
            export_xyz_path = None
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "msg": str(e)})


@app.route("/api/export/cancel", methods=["POST"])
def api_export_cancel():
    global export_process
    with export_lock:
        if export_process is None or export_process.poll() is not None:
            return jsonify({"ok": False, "msg": "Kein Export aktiv"})
        try:
            os.killpg(os.getpgid(export_process.pid), signal.SIGKILL)
        except Exception:
            pass
        export_process = None
    return jsonify({"ok": True})


@app.route("/api/download/<filename>")
def api_download(filename):
    if "/" in filename:
        return jsonify({"error": "Ungültig"}), 400
    allowed_ext = (".bag", ".xyz", ".e57", ".pcd", ".las")
    if not any(filename.endswith(e) for e in allowed_ext):
        return jsonify({"error": "Nicht erlaubt"}), 403
    path = os.path.join(BAGS_DIR, filename)
    if not os.path.exists(path):
        return jsonify({"error": "Nicht gefunden"}), 404
    return send_file(path, as_attachment=True)


@app.route("/api/sysinfo")
def api_sysinfo():
    import shutil, re as _re
    try:
        disk       = shutil.disk_usage("/media/nvidia/SSD")
        disk_used  = round(disk.used  / (1024**3), 1)
        disk_total = round(disk.total / (1024**3), 1)
        disk_pct   = round(disk.used  / disk.total * 100, 1)
    except Exception:
        disk_used = disk_total = disk_pct = 0
    try:
        cpu = round(float(open("/proc/loadavg").read().split()[0]), 2)
    except Exception:
        cpu = 0.0
    def read_cpu_stats():
        lines = [l for l in open("/proc/stat").readlines() if _re.match(r"cpu[0-9]", l)]
        result = []
        for l in lines:
            v = list(map(int, l.split()[1:]))
            idle  = v[3] + (v[4] if len(v) > 4 else 0)
            total = sum(v)
            result.append((total, idle))
        return result
    s1 = read_cpu_stats()
    time.sleep(0.5)
    s2 = read_cpu_stats()
    cpu_per = []
    for (t1, i1), (t2, i2) in zip(s1, s2):
        dt = t2 - t1
        cpu_per.append(round((1 - (t2 - t1 - (i2 - i1)) / dt) * 100, 1) if dt else 0.0)
    try:
        temp = round(int(open("/sys/class/thermal/thermal_zone0/temp").read().strip()) / 1000.0, 1)
    except Exception:
        temp = 0.0
    return jsonify({
        "disk_used_gb":  disk_used,
        "disk_total_gb": disk_total,
        "disk_pct":      disk_pct,
        "cpu_load":      cpu,
        "cpu_per":       cpu_per,
        "temp_c":        temp,
    })


@app.route("/api/arm/scan_active", methods=["POST"])
def api_arm_scan_active():
    """Scan wurde gestartet — Auto-DISARM-Timer abbrechen."""
    _cancel_arm_timer()
    return jsonify({"ok": True})


@app.route("/api/arm", methods=["POST"])
def api_arm():
    global arm_state
    with arm_lock:
        if arm_state in ("arming", "armed"):
            return jsonify({"ok": False, "msg": "Bereits " + arm_state})
        arm_state = "arming"
    t = threading.Thread(target=_arm_sequence, daemon=True)
    t.start()
    return jsonify({"ok": True, "state": "arming"})


@app.route("/api/disarm", methods=["POST"])
def api_disarm():
    global arm_state
    _cancel_arm_timer()
    with arm_lock:
        if arm_state == "disarmed":
            return jsonify({"ok": True, "state": "disarmed"})
        arm_state = "disarming"
    t = threading.Thread(target=_disarm_sequence, daemon=True)
    t.start()
    return jsonify({"ok": True, "state": "disarming"})


if __name__ == "__main__":
    os.makedirs(BAGS_DIR, exist_ok=True)
    os.makedirs(MAPS_DIR, exist_ok=True)
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
