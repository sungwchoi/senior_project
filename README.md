## Senior Project
--------------------
This is private back up repository for Senior project. 

## Installation
--------------------
```bash 

# Gitclone
cd ~/senior_project 
git clone https://github.com/ultralytics/yolov5 

# Installation required libraries
cd yolov5
pip install -r requirements.txt numpy==1.24.4 

# Implementation
python3 detect.py --weights yolov5n.pt --source 0 

python3 detect.py --weights best_v<version_number>.pt --source 0   
```

## Documentation  
--------------------
 
Initial version is best.pt_v0.
best.pt_v1 is the version with --epoch 200 and freeze 10.

For the yolov8n:

Inside the ~/senior_project

```bash
python3 detect_v8.py
``` 
