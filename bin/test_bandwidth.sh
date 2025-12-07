#!/bin/bash
# Bandwidth test script: Compare PIR=0.5 vs PIR=1.0 with 4-flit packets

export LD_LIBRARY_PATH=/projects/befp/dm68/local/systemc-2.3.3/lib-linux64:/projects/befp/dm68/local/yaml-cpp/lib64:$LD_LIBRARY_PATH

echo "=================================================================="
echo "Bandwidth Test: 4-flit packets with INTERVAL=$1"
echo "=================================================================="
echo ""

# Test 1: PIR=0.5 (1 packet every 2 cycles = 2 flits/cycle with 4-flit packets)
echo "TEST 1: PIR=0.5 (1 packet every 2 cycles)"
echo "------------------------------------------------------------------"
timeout 120 ./noxim -config ../config_examples/bandwidth_test_2cycles.yaml 2>&1 | grep -E "Total received|Average delay|MemTile\[.*\] Statistics:|Link Utilization:|Egress \(RESPs\):" | head -40
echo ""
echo ""

# Test 2: PIR=1.0 (1 packet every cycle = 4 flits/cycle with 4-flit packets)  
echo "TEST 2: PIR=1.0 (1 packet every cycle)"
echo "------------------------------------------------------------------"
timeout 120 ./noxim -config ../config_examples/bandwidth_test_1cycle.yaml 2>&1 | grep -E "Total received|Average delay|MemTile\[.*\] Statistics:|Link Utilization:|Egress \(RESPs\):" | head -40
echo ""
echo ""

echo "=================================================================="
echo "SUMMARY:"
echo "- With PIR=0.5: Expect ~50% utilization with INTERVAL=2, ~25% with INTERVAL=1"
echo "- With PIR=1.0: Expect ~100% utilization with INTERVAL=1, ~50% with INTERVAL=2"
echo "- INTERVAL=$1 currently set in MemoryController.h"
echo "=================================================================="
