from ultralytics import YOLO

# Load a model
model = YOLO("/home/guilh/R4F_EVAbot_Navigation/lib/Yolo_Tensorrt/r4f_yolov8m_seg.pt")  # load a custom trained model

# Export the model
success = model.export(format='onnx',
                        imgsz=640,
                        optimize=False,
                        half=True,
                        int8=False,
                        dynamic=False,
                        opset=11) # export the model to onnx format FP16
assert success