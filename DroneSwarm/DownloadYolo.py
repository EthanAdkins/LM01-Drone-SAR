import os
import torch

if __name__ == '__main__':
    print("Downloading YOLOV5")

    cwd = os.getcwd()
    yoloPT = os.path.join(str(cwd), 'noiseModel.pt')

    # Model is loaded global to be used by service functions
    try:
        print("Loading model")
        MODEL = torch.hub.load('ultralytics/yolov5', 'custom', path=yoloPT, trust_repo=True)
        print("Model loaded")
        isModelLoaded = True
    except Exception as error:
        print(error)
        isModelLoaded = False

    if (isModelLoaded):
        print("YOLOV5 Downloaded Succesfully!")
    else:
        print("YOLOV5 Failed to Download!")