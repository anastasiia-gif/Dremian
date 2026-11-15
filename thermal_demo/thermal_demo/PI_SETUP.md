# Running the demo on a Raspberry Pi (web UI)

The same code runs on your laptop and on the Pi. The Pi runs a small web server;
you watch the live detection in a browser on your laptop over the network. This
suits a headless Pi Zero 2W (no monitor needed on the Pi).

## 0. What you need
- A Raspberry Pi running Raspberry Pi OS (Lite is fine — no desktop needed).
- The Pi and your laptop on the **same network** (WiFi or Ethernet).
- This `thermal_demo` folder copied onto the Pi.

## 1. Copy the folder to the Pi
From your laptop (replace the IP with your Pi's address):
```bash
scp -r thermal_demo  pi@<pi-ip>:~/
```
or just put it on the SD card / a USB stick.

## 2. Install the dependencies (one time, on the Pi)
SSH into the Pi (`ssh pi@<pi-ip>`), then:
```bash
cd ~/thermal_demo
sudo apt update && sudo apt install -y python3-pip
pip3 install -r requirements.txt --break-system-packages
```
> On Pi OS Lite, `opencv-python-headless` is the right package (no GUI libraries
> needed because the video goes to the browser, not a window on the Pi).

## 3. Run it
```bash
python3 server.py
```
You'll see:
```
 Thermal detection web UI is running.
   On this machine : http://localhost:8000
   On the network  : http://<this-device-ip>:8000
```

## 4. Watch it from your laptop
Find the Pi's IP (run `hostname -I` on the Pi), then on your laptop open a browser:
```
http://<pi-ip>:8000
```
You'll see the live thermal feed, the TARGET LOCK box, and the latency read-outs.

## 5. Measure latency on the Pi (the number that matters)
The web UI shows live `algorithm` latency. For a written record, also run the
headless harness on the Pi:
```bash
python3 run_demo.py            # prints + writes latency_report.txt on the Pi
python3 selftest.py            # reproducibility check on the Pi
```
The **algorithm latency measured on the Pi** is your real, defensible figure —
this is the whole point of putting it on the Pi. (The end-to-end figure still
excludes the real camera-capture chain until a camera is attached; see below.)

## 6. Later: real camera instead of synthetic frames
When you have the Caddx camera + capture path working as a V4L2 device on the Pi,
implement `get_frame()` in the `V4L2ThermalSource` stub in `frame_source.py`
(open `/dev/video0`, return a grayscale uint8 frame), then in `server.py` and
`run_demo.py` swap `SyntheticThermalSource(...)` for `V4L2ThermalSource(...)`.
Nothing else changes — and now the capture-stage timing is real, giving a true
end-to-end latency on the Pi.

## Troubleshooting
- **Browser shows the page but no video**: OpenCV didn't install. Re-run the
  pip install step; check `python3 -c "import cv2"` works on the Pi.
- **Can't reach the page from the laptop**: confirm both devices are on the same
  network and use the Pi's actual IP (`hostname -I`). The server listens on all
  interfaces (0.0.0.0), so a firewall on the laptop is the usual culprit.
- **Slow / low frame rate on Pi Zero 2W**: lower the rate with
  `python3 server.py --fps 12`. The detection still runs full-speed; only the
  streamed preview is throttled.
