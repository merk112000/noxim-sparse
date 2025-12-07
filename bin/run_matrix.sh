#!/bin/bash
# Helper script to run Noxim with different sparse matrices
# Usage: ./run_matrix.sh <matrix_name> [simulation_time]
#   matrix_name: web-Stanford, roadNet-PA, filter3D, cit-HepTh, roadNet-TX, web-NotreDame
#   simulation_time: optional, defaults to 10000

# Set library paths
export LD_LIBRARY_PATH=/projects/befp/dm68/local/systemc-2.3.3/lib-linux64:/projects/befp/dm68/local/yaml-cpp/lib64:$LD_LIBRARY_PATH

# Available matrices
AVAILABLE_MATRICES=(
    "web-Stanford"
    "roadNet-PA"
    "filter3D"
    "cit-HepTh"
    "roadNet-TX"
    "web-NotreDame"
)

if [ $# -lt 1 ]; then
    echo "Usage: $0 <matrix_name> [simulation_time]"
    echo "Available matrices:"
    for matrix in "${AVAILABLE_MATRICES[@]}"; do
        echo "  - $matrix"
    done
    exit 1
fi

MATRIX_NAME=$1
SIM_TIME=${2:-10000}

# Construct trace directory path
TRACE_DIR="../other/suitesparse_traces/${MATRIX_NAME}_16"

# Check if directory exists
if [ ! -d "$TRACE_DIR" ]; then
    echo "Error: Trace directory not found: $TRACE_DIR"
    echo "Available matrices:"
    ls -1 ../other/suitesparse_traces/
    exit 1
fi

echo "Running Noxim with matrix: $MATRIX_NAME"
echo "Trace directory: $TRACE_DIR"
if [ $# -ge 2 ]; then
    echo "Simulation time: $SIM_TIME cycles (overridden)"
else
    echo "Using all parameters from YAML config"
fi
echo "-------------------------------------------"

CONFIG_FILE="../config_examples/trace_based_sparse.yaml"
TEMP_CONFIG="/tmp/noxim_temp_config_$$.yaml"

# Always create temp config to update trace_dir
if [ $# -ge 2 ]; then
    # Override both trace_dir and simulation_time
    sed "s|trace_dir:.*|trace_dir: $TRACE_DIR|" "$CONFIG_FILE" | \
    sed "s|simulation_time:.*|simulation_time: $SIM_TIME|" > "$TEMP_CONFIG"
else
    # Only override trace_dir, keep all other YAML params
    sed "s|trace_dir:.*|trace_dir: $TRACE_DIR|" "$CONFIG_FILE" > "$TEMP_CONFIG"
fi

# Run simulation
./noxim -config "$TEMP_CONFIG" 2>&1 | grep -E "(Compute PE.*loaded|% Total|% Global|% Max|% Network|Memory Tile Statistics|MemTile|Requests|Heartbeat|\[PE )"

# Cleanup
rm -f "$TEMP_CONFIG"

echo "-------------------------------------------"
echo "Simulation completed for matrix: $MATRIX_NAME"
