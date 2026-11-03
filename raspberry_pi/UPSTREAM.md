# Tracking sources

The runtime uses the installed `ultralytics==8.4.159` implementation, rather than
vendoring or rewriting its Kalman filter and global motion compensation.

- [Ultralytics tracking integration at v8.4.159](https://github.com/ultralytics/ultralytics/blob/v8.4.159/ultralytics/trackers/track.py)
- [BoT-SORT implementation](https://github.com/ultralytics/ultralytics/blob/v8.4.159/ultralytics/trackers/bot_sort.py)
- [ByteTrack implementation](https://github.com/ultralytics/ultralytics/blob/v8.4.159/ultralytics/trackers/byte_tracker.py)
- [Official BoT-SORT configuration](https://github.com/ultralytics/ultralytics/blob/v8.4.159/ultralytics/cfg/trackers/botsort.yaml)
- [Ultralytics track documentation](https://docs.ultralytics.com/modes/track/)
- Original algorithms: [BoT-SORT](https://github.com/NirAharon/BoT-SORT), [ByteTrack](https://github.com/FoundationVision/ByteTrack).

Ultralytics and the supplied custom 13-class YOLO11n NCNN model identify their license as AGPL-3.0;
see [upstream license](https://github.com/ultralytics/ultralytics/blob/v8.4.159/LICENSE).
The local tracker YAMLs configure the official implementation with a 0.35 new
track threshold, a 0.1 low-confidence association threshold, sparse optical
flow for BoT-SORT, and no extra appearance model. `tracking.py` adds wire budget,
session isolation, immutable class identity and elapsed-time expiry policies.

The default `best_ncnn_model` was supplied locally by the user, exported with
Ultralytics 8.3.221 at fixed 320x320 input. Its original four model data files are
copied unchanged. The runtime remains pinned to 8.4.159. `model_config.py` maps its
13 local class IDs to the existing FPGA COCO-80 wire IDs and validates the supplied
`classes.json`; tracker association uses the original model class IDs. An active
re-acquisition alias has priority over replacement candidates and refreshes its
geometry from reliable current observations. This remains geometric recovery,
not an appearance/ReID embedding model.

The v8.4.159 tracker has an inference-step Kalman clock. This package permits
ordinary dropped camera frames, expires IDs by sampled elapsed time, and resets
the namespace after a sample gap over 0.6 seconds. It does not claim a variable
time-step Kalman implementation or validated identity through full occlusion.
