# CoreS3 Edge-AI Visual Inspection

Reproducibility package for the manuscript **“Resource-Aware Camera-Domain Adaptation for Portable Edge-AI Visual Inspection”**, prepared for the *IEEE Open Journal of the Computer Society*.

## What this repository contains

- **Firmware** for M5Stack CoreS3 / ESP32-S3: QVGA acquisition, RGB conversion, ESPDL inference, filtering, counting, LCD visualization, local storage, CSV history, and HTTP reporting.
- **Training material** for the baseline and camera-reinforced detector.
- **ESPDL quantization workflow** and helper scripts.
- **Model artifacts**: PyTorch, ONNX, and final ESPDL model.
- **Project-generated CoreS3 data**:
  - 21 manually labeled camera-domain adaptation images.
  - 22 frozen held-out CoreS3 validation images and labels.
  - representative final detections and on-device timing evidence.
- **Results** for validation metrics, camera-domain diagnostics, device response time, and post-detection memory.
- **Paper source** and figures.

## Key reported results

| Metric | Baseline | Camera-reinforced |
|---|---:|---:|
| Precision | 0.898 | 0.995 |
| Recall | 0.776 | 0.891 |
| mAP@0.50 | 0.920 | 0.959 |
| mAP@0.50:0.95 | 0.585 | 0.570 |

Five documented device runs show end-to-end response times of **12.121–13.557 s**. Immediately after detection returns, the firmware reports **11,919 B free internal RAM** and **6,858,372 B free PSRAM**. These are post-detection values, not peak inference memory.

## Repository layout

```text
firmware/                  CoreS3 application source
training/                  training/export/evaluation material
quantization/              ESPDL conversion notebook + helpers
models/                    baseline and final model artifacts
data/cores3_adaptation/    21 manually labeled target-camera images
data/cores3_frozen_validation/ 22 frozen held-out validation images
data/on_device_evidence/   final detections and screen photographs
results/                   CSV summaries used in the paper
docs/                      methodology and model-evolution notes
paper/                     OJCS LaTeX source and figures
external_data/             provenance/redistribution note
```

## Firmware configuration

The public package intentionally contains **no Wi-Fi credentials**.

```bash
cp firmware/src/project_config.example.h firmware/src/project_config.h
```

Then edit `project_config.h` locally.

## Data roles

The 21 images in `cores3_adaptation` were used to construct the reinforced training set and must not be treated as an independent test. The 22 images in `cores3_frozen_validation` were kept outside gradient updates and augmentation and were used as the frozen held-out validation split for both reported checkpoints.

## Runtime instrumentation

The detector call is synchronous and blocking with respect to the calling task. Therefore, the firmware can record heap state immediately before/after the detection routine, but the reported memory values do not represent the transient peak while `model->run()` is active. The CoreS3 does not expose native inference-level power/energy telemetry; external electrical instrumentation would be required for those measurements.

## Reproduce summary checks

```bash
python scripts/verify_dataset.py
python scripts/summarize_runtime.py
```

## External data

Some original training/calibration imagery in the research archive has undocumented redistribution terms and is therefore not included here. See `external_data/README.md`.

## Citation

See `CITATION.cff`. Replace the repository URL placeholder after publishing this package on GitHub.

## Acknowledgment

The authors would like to express their gratitude to Universidad Indoamérica for its support of this research through the “Tecnologías de la Industria 4.0 en Educación, Salud, Empresa e Industria” project.
