# Models

This directory holds the detector weights.

| File | Tracked in git | Purpose |
|---|---|---|
| `best.pt` | yes | YOLO weights used by the detector |
| `best.engine` | **no** (`.gitignore`) | TensorRT FP16 engine, built on the target device |
| `train_colab.ipynb` | yes | Training notebook |

## Expected classes

The tracker's logic keys off the class **label strings**, not the indices:

```python
"CLASS_NAMES": { "0": "mannequin", "1": "tent" }     // config.json -> detector
```

The detector compares the loaded model's class names against this and logs a loud warning
if they differ, because silently renamed or reordered classes break target clustering,
per-class target selection, IBVS class matching and the payload identity gate.

To use different classes:

1. Update `CLASS_NAMES` in `config.json` (section `detector`).
2. Update `PAYLOAD_TARGETS` in `config.json` (section `payload_hardware`), which maps each
   class label to a servo pin, a motor channel and a release angle. A class with no entry
   there is detected and visited but never triggers the payload.

`python3 tools/show_config.py --check` verifies that the two agree.

## Training your own

Open `train_colab.ipynb` in Google Colab. Set the three paths in its settings cell (Drive
folder, `data.yaml`, run name), point it at your own dataset and run it top to bottom.

Or train locally with Ultralytics:

```bash
yolo detect train data=your_dataset.yaml model=yolo11n.pt imgsz=640 epochs=100
```

Copy the resulting `runs/detect/train/weights/best.pt` here.

## Large-file note

`best.pt` is around 39 MB. GitHub warns above 50 MB and rejects files above 100 MB. If
your weights are larger, either use [Git LFS](https://git-lfs.com/) or attach them to a
GitHub Release and document the download step here instead of committing the file.

## TensorRT engine

Only needed when `USE_TENSORRT` is `true` in `config.json`. It is **off by default**,
because with tiling disabled a single full-frame `.pt` inference is the faster path.

Build it **on the target device** — an engine is specific to the JetPack, CUDA and GPU
version it was compiled for:

```bash
python3 tools/export_tensorrt.py
```

Never copy an engine between machines, and never commit one.
