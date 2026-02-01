#!/bin/bash

# Source directory
SRC_DIR="./navigation_materials"

echo "Deploying navigation materials from $SRC_DIR..."

# 1. Copy .env to cabot/ and cabot/cabot-navigation/
if [ -f "$SRC_DIR/.env" ]; then
    cp "$SRC_DIR/.env" ./
    cp "$SRC_DIR/.env" ./cabot-navigation/
    echo "Copied .env to ./ and ./cabot-navigation/"
else
    echo "Warning: $SRC_DIR/.env not found"
fi

# 2. Copy SOCNAV_V2.prms and SOCNAV_V2.tch to cabot/SNGNN2D-v2/model
DEST_DIR_2="./SNGNN2D-v2/model"
mkdir -p "$DEST_DIR_2"
if [ -f "$SRC_DIR/SOCNAV_V2.prms" ]; then
    cp "$SRC_DIR/SOCNAV_V2.prms" "$DEST_DIR_2/"
    echo "Copied SOCNAV_V2.prms to $DEST_DIR_2/"
else
    echo "Warning: $SRC_DIR/SOCNAV_V2.prms not found"
fi

if [ -f "$SRC_DIR/SOCNAV_V2.tch" ]; then
    cp "$SRC_DIR/SOCNAV_V2.tch" "$DEST_DIR_2/"
    echo "Copied SOCNAV_V2.tch to $DEST_DIR_2/"
else
    echo "Warning: $SRC_DIR/SOCNAV_V2.tch not found"
fi

# 3. Copy yolov8*.pt to cabot/cabot-navigation/VLM-Social-Nav-ROS2/src/vsn_yolo_ros/models
DEST_DIR_3="./cabot-navigation/VLM-Social-Nav-ROS2/src/vsn_yolo_ros/models"
mkdir -p "$DEST_DIR_3"
if ls "$SRC_DIR"/yolov8*.pt 1> /dev/null 2>&1; then
    cp "$SRC_DIR"/yolov8*.pt "$DEST_DIR_3/"
    echo "Copied yolov8*.pt to $DEST_DIR_3/"
else
    echo "Warning: No yolov8*.pt files found in $SRC_DIR"
fi

# 4. Copy ranking_distillation_model.pth to VLM_Rank_Nav/distillation/checkpoints/
DEST_DIR_4="./VLM_Rank_Nav/distillation/checkpoints"
mkdir -p "$DEST_DIR_4"
if [ -f "$SRC_DIR/ranking_distillation_model.pth" ]; then
    cp "$SRC_DIR/ranking_distillation_model.pth" "$DEST_DIR_4/"
    echo "Copied ranking_distillation_model.pth to $DEST_DIR_4/"
else
    echo "Warning: $SRC_DIR/ranking_distillation_model.pth not found"
fi

echo "Done."
