# Models

This directory holds the detector weights.

| File | Tracked in git | Purpose |
|---|---|---|
| `v14_best.pt` | **no** (`.gitignore`) | YOLO weights used by the detector (`MODEL_FILE`) |
| `v14_best_960.engine` | **no** (`.gitignore`) | TensorRT FP16 engine for `IMGSZ=960`, built on the target device |
| `train_colab.ipynb` | yes | Training notebook |

**The weights are not in the repository.** Copy them onto the Jetson by hand, e.g.

```bash
scp v14_best.pt <user>@<jetson-ip>:<repo>/models/
```

The flown model is a YOLO11m-P2 (an extra P2 head for small targets) fine-tuned at
`imgsz=640`, two classes. It runs full-frame at `IMGSZ=960` with a TensorRT engine.

## Expected classes

The tracker's logic keys off the class **label strings**, not the indices:

```python
CLASS_NAMES = {0: "mannequin", 1: "tent"}     # suas/config.py -> detector
```

The detector compares the loaded model's class names against this and logs a loud warning
if they differ, because silently renamed or reordered classes break target clustering,
per-class target selection, IBVS class matching and the payload identity gate.

To use different classes:

1. Update `CLASS_NAMES` in `suas/config.py`.
2. Update `PAYLOAD_TARGETS` in `suas/config.py`, which maps each
   class label to a servo pin, a motor channel and a release angle. A class with no entry
   there is detected and visited but never triggers the payload.

`tests/test_config.py` verifies that the two agree.

## Training your own

Open `train_colab.ipynb` in Google Colab. Set the three paths in its settings cell (Drive
folder, `data.yaml`, run name), point it at your own dataset and run it top to bottom.

Or train locally with Ultralytics:

```bash
yolo detect train data=your_dataset.yaml model=yolo11n.pt imgsz=640 epochs=100
```

Copy the resulting `runs/detect/train/weights/best.pt` here and point `MODEL_FILE` at it.

## TensorRT engine

`USE_TENSORRT` is `true` by default. The engine has a fixed input shape, so its file name
carries the working size: `<MODEL_FILE stem>_<IMGSZ>.engine` (e.g. `v14_best_960.engine`).
Changing `IMGSZ` therefore looks for a different engine; if it does not exist the detector
logs a warning and falls back to the `.pt` (about 2x slower on the Jetson).

Build it **on the target device** — an engine is specific to the JetPack, CUDA and GPU
version it was compiled for:

```bash
python3 tools/export_tensorrt.py
```

Never copy an engine between machines, and never commit one.
