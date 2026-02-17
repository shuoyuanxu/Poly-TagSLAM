#!/bin/bash
# ============================================================
# LOO Cross-Validation Runner for AprilSLAM
# 
# For each LOO split CSV:
#   1. Copy it to config/TotalStation_to_LIO_3D.csv
#   2. Play the rosbag to feed data into the SLAM node
#   3. Copy the output afteroptimisation.csv to results/
# ============================================================

set -e

# ── Paths ─────────────────────────────────────────────────────────────────────
PACKAGE_PATH=$(rospack find aprilslamcpp)
LOO_DIR="/home/shuoyuan/catkin_aereh_ws/src/aprilslamcpp/config/loo_splits"
SURVEY_CSV="$PACKAGE_PATH/config/TotalStation_to_LIO_3D.csv"  # what the node reads
OUTPUT_CSV="$PACKAGE_PATH/config/afteroptimisation.csv"
RESULTS_DIR="$PACKAGE_PATH/config/loo_results"
BAG_FILE="/media/shuoyuan/CrucialX9/PolyTunnel_haygrove_Feb2026/tfcorrection/hard2_tffix.bag"
BAG_RATE=10
LAUNCH_FILE="run_calibration.launch"

mkdir -p "$RESULTS_DIR"

# ── Backup original survey CSV ────────────────────────────────────────────────
SURVEY_BACKUP="${SURVEY_CSV}.bak"
if [ ! -f "$SURVEY_BACKUP" ]; then
    cp "$SURVEY_CSV" "$SURVEY_BACKUP"
    echo "[INFO] Backed up original survey CSV to $SURVEY_BACKUP"
fi

# ── Find all LOO split files ──────────────────────────────────────────────────
LOO_FILES=($(ls "$LOO_DIR"/loo_*.csv | sort))
TOTAL=${#LOO_FILES[@]}

if [ "$TOTAL" -eq 0 ]; then
    echo "[ERROR] No LOO CSV files found in $LOO_DIR"
    exit 1
fi

echo "[INFO] Found $TOTAL LOO splits. Starting runs..."
echo "============================================================"

# ── Main Loop ─────────────────────────────────────────────────────────────────
for i in "${!LOO_FILES[@]}"; do
    LOO_FILE="${LOO_FILES[$i]}"
    BASENAME=$(basename "$LOO_FILE" .csv)
    RUN_NUM=$(printf "%02d" $((i + 1)))

    echo ""
    echo "[$RUN_NUM/$TOTAL] Running LOO split: $BASENAME"
    echo "------------------------------------------------------------"

    # 1. Swap in the LOO CSV as the survey landmarks file
    cp "$LOO_FILE" "$SURVEY_CSV"
    echo "  [1/4] Copied LOO split -> $SURVEY_CSV"

    # 2. Launch the SLAM node in its own process group so we can kill everything cleanly
    setsid roslaunch aprilslamcpp $LAUNCH_FILE &
    LAUNCH_PID=$!
    echo "  [2/4] Launched SLAM node (PID $LAUNCH_PID), waiting 5s to settle..."
    sleep 5

    # 3. Play the rosbag and wait for it to finish
    echo "  [3/4] Playing rosbag at ${BAG_RATE}x speed..."
    rosbag play "$BAG_FILE" -r $BAG_RATE
    echo "  [3/4] Rosbag finished."

    # 4. Give node a moment to flush/save output CSV, then kill the whole process group
    sleep 3
    kill -- -$LAUNCH_PID 2>/dev/null || true
    wait $LAUNCH_PID 2>/dev/null || true
    echo "  [2/4] SLAM node stopped."

    # 5. Save output
    RESULT_FILE="$RESULTS_DIR/${BASENAME}_result.csv"
    if [ -f "$OUTPUT_CSV" ]; then
        cp "$OUTPUT_CSV" "$RESULT_FILE"
        echo "  [4/4] Result saved -> $RESULT_FILE"
    else
        echo "  [4/4] WARNING: Output CSV not found at $OUTPUT_CSV — skipping save."
    fi

    echo "  Done with run $RUN_NUM."
done

# ── Restore original survey CSV ───────────────────────────────────────────────
cp "$SURVEY_BACKUP" "$SURVEY_CSV"
echo ""
echo "============================================================"
echo "[INFO] Restored original survey CSV."
echo "[INFO] All $TOTAL LOO runs complete. Results in: $RESULTS_DIR"
echo "============================================================"
