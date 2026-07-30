Place the exported Ultralytics YOLO NCNN model here as best_ncnn_model/
(the folder produced by `yolo export model=best.pt format=ncnn half=True`,
containing model.ncnn.param, model.ncnn.bin, and metadata.yaml).

A plain PyTorch best.pt is also supported: pass its path with the
model_path launch argument to override the default.

Requires the `ncnn` Python package to be installed alongside ultralytics
(pip install ncnn) wherever buoy_detector_node runs inference.
