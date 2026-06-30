#!/usr/bin/env python3
from flask import Flask, render_template, jsonify, Response
import json, os

app = Flask(__name__, template_folder="templates", static_folder="static")

@app.route("/")
def index():
    return render_template("lidar_viewer.html")

@app.route("/api/points")
def get_points():
    try:
        with open("/tmp/pts.json") as f:
            data = f.read()
    except:
        data = "[]"
    resp = Response(data, mimetype="application/json")
    resp.headers["Cache-Control"] = "no-cache, no-store"
    return resp

@app.route("/api/imu")
def get_imu():
    return jsonify({"roll": 0, "pitch": 0, "yaw": 0})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
