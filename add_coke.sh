#!/bin/bash
rosrun object_recognition_core object_add.py -n "coke" -d "A universal can of coke" -- commit

rosrun object_recognition_core mesh_add.py 3e6bb2f77b7e8280f7206475ef003449 $(find pcl_recognition)/data/coke.stl