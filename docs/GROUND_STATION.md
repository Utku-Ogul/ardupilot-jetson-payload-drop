# Watching the Annotated Video Feed

The companion can stream the **annotated** camera view (detection boxes and the state
overlay drawn on top) to the ground as RTP/UDP H.264, encoded in hardware.

This is a separate stream from the camera's own raw RTSP feed. It carries what the
detector actually sees, which is what you want while debugging a mission.

---

## 1. Enable it

In `config.json` (section `streaming`):

```json
"streaming": {
  "STREAM_ENABLED": true,
  "STREAM_HOST": "<ground-station-ip>",
  "STREAM_PORT": 5600,
  "STREAM_WIDTH": 960,
  "STREAM_HEIGHT": 540,
  "STREAM_BITRATE": 2000000
}
```

`STREAM_HOST` is your ground station's IP; port 5600 is the QGroundControl and Mission
Planner default. Verify with `python3 tools/show_config.py --changed`.

Then launch through **`run.sh`**. Streaming needs the GStreamer-enabled OpenCV; under a
bare `python3` it disables itself silently (flight is unaffected) and logs one warning.

On success the log shows:

```
Live stream ON -> udp://<ground-station-ip>:5600  960x540@15fps  2000 kbps  (IDR=15 frames)
```

---

## 2. Network

Connect the ground station to your video link's LAN port and give it a static address on
the same subnet. Check your radio's manual for addresses it reserves for itself.

Verify reachability first:

```bash
ping <companion-ip>      # the companion
```

---

## 3. Viewing

### Mission Planner / QGroundControl (easiest)

Video widget → Source = **UDP h.264 Video Stream** → Port **5600**. The image appears once
the publisher starts.

### GStreamer (lowest latency)

Linux:

```bash
gst-launch-1.0 -v udpsrc port=5600 \
  caps="application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264,payload=(int)96" \
  ! rtpjitterbuffer latency=0 drop-on-latency=true \
  ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

Windows (on an NVIDIA GPU, swap `avdec_h264` for `d3d11h264dec`):

```cmd
gst-launch-1.0.exe -v udpsrc port=5600 ^
  caps="application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264,payload=(int)96" ^
  ! rtpjitterbuffer latency=0 drop-on-latency=true ^
  ! rtph264depay ! avdec_h264 ! videoconvert ! d3dvideosink sync=false
```

### VLC or ffplay (via an SDP file)

Create `stream.sdp`, replacing the address with your ground station's IP:

```
v=0
o=- 0 0 IN IP4 <ground-station-ip>
s=SUAS Annotated
c=IN IP4 <ground-station-ip>
t=0 0
m=video 5600 RTP/AVP 96
a=rtpmap:96 H264/90000
```

```bash
vlc --network-caching=0 --no-audio stream.sdp
ffplay -fflags nobuffer -flags low_delay -framedrop \
       -protocol_whitelist file,rtp,udp -i stream.sdp
```

### Windows firewall

If nothing appears:

```cmd
netsh advfirewall firewall add rule name="SUAS5600" dir=in action=allow protocol=UDP localport=5600
```

---

## 4. Test the path before flying

Run this on the companion — it needs none of the flight code:

```bash
gst-launch-1.0 videotestsrc is-live=true \
  ! video/x-raw,width=1280,height=720,framerate=30/1 \
  ! nvvidconv ! nvv4l2h264enc insert-sps-pps=1 idrinterval=15 bitrate=4000000 \
  ! rtph264pay config-interval=1 pt=96 mtu=1400 \
  ! udpsink host=<ground-station-ip> port=5600 sync=false
```

A colour test pattern on the ground station means the path works. Then set
`"STREAM_ENABLED": true` in `config.json` and use the real stream.

---

## 5. Latency and bandwidth

Expect roughly 150–300 ms end to end (hardware encode + radio + decode). To keep it there,
the receiver must not buffer: `sync=false`, `--network-caching=0`, ffplay `-flags
low_delay`.

If the raw camera feed shares the same radio link, bandwidth can run out. Reduce:

```json
"STREAM_WIDTH": 640,
"STREAM_HEIGHT": 360,
"STREAM_BITRATE": 800000,
"STREAM_FPS": 10
```

The most reliable option is not to send the raw camera feed to the ground at all — the
annotated stream is usually more useful anyway.

The publisher is constant-bitrate with a leaky queue, so latency does not accumulate, and
all of its errors are swallowed internally: **streaming can never affect telemetry, IBVS
or the payload path.**
