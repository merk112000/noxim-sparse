# Trace-Driven Traffic Mode Implementation Summary

## Overview
This document summarizes the modifications made to Noxim to support trace-driven traffic for sparse matrix workloads (SparseNoC project).

## New Feature: TRAFFIC_TRACE_BASED Mode

### What It Does
- Reads pre-generated trace files (one per PE) containing sparse matrix communication patterns
- Deterministically injects packets at specific cycles according to the trace
- Each trace event specifies: cycle, source PE, destination PE, and feature ID
- Supports sparse matrix partitioning across multiple PEs for GNN workloads

### Trace File Format
```
# Lines starting with # or % are comments
<cycle> <src_router_id> <dst_router_id> <feature_id>
```

Example:
```
# web-Stanford sparse matrix trace for PE 0
100 0 5 42
150 0 3 17
200 0 12 93
```

## Files Modified

### 1. GlobalParams.h
**Added:**
- `TRAFFIC_TRACE_BASED` constant (line ~77)
- `trace_dir` static string variable to store trace file directory path

### 2. GlobalParams.cpp
**Added:**
- Declaration of `trace_dir` variable

### 3. ConfigurationManager.cpp
**Added:**
- YAML parameter parsing for `trace_dir` with empty string default
- Allows trace directory to be specified in configuration file

### 4. DataStructs.h
**Modified:**
- Added `feature_id` field to `Packet` structure (initialized to -1)
- Added `feature_id` field to `Flit` structure
- Enables tracking which sparse matrix feature each packet represents

### 5. ProcessingElement.h
**Added:**
- `TraceEvent` struct to represent single trace entry (cycle, src, dst, feature_id)
- `trace_events` vector to store all events for this PE
- `next_event_idx` to track progress through trace
- `loadTraceFile()` method to parse trace file at initialization
- `canShotTrace()` method to check if trace event should inject at current cycle

**Modified:**
- Constructor now initializes `next_event_idx = 0` and calls `loadTraceFile()`

### 6. ProcessingElement.cpp
**Added:**
- Include directives: `<fstream>`, `<sstream>`, `<iostream>` for file I/O

**Implemented loadTraceFile():**
- Validates that TRAFFIC_TRACE_BASED mode is enabled
- Checks that trace_dir is configured
- Builds filename: `{trace_dir}/pe_{local_id}.trace`
- Opens and parses trace file line-by-line
- Skips comments (lines starting with # or %)
- Validates that src matches local_id
- Stores events in trace_events vector
- Prints diagnostic messages about loaded events

**Implemented canShotTrace():**
- Checks if trace events remain (next_event_idx < trace_events.size())
- Calculates current cycle from SystemC timestamp
- Compares against next event's cycle
- If match, creates Packet with trace-specified destination and feature_id
- Returns true to signal packet is ready for injection

**Modified txProcess():**
- Added branch at beginning to check for TRAFFIC_TRACE_BASED mode
- Calls canShotTrace() instead of canShot() for trace mode
- Increments next_event_idx after successful injection
- Falls through to existing traffic patterns for other modes

**Modified nextFlit():**
- Added line to propagate `feature_id` from Packet to Flit
- Ensures feature ID is preserved through flit generation

## Configuration Example

```yaml
# config_examples/trace_based_sparse.yaml
mesh_dim_x: 4
mesh_dim_y: 4
topology: MESH
traffic_distribution: TRAFFIC_TRACE_BASED
trace_dir: ../other/suitesparse_traces/web-Stanford_16
routing_algorithm: XY
simulation_time: 100000
# ... other standard parameters
```

## Usage Instructions

### 1. Prepare Trace Files
- Create a directory with trace files: `pe_0.trace`, `pe_1.trace`, etc.
- One file per PE that will inject traffic
- Format: `<cycle> <src> <dst> <feature_id>` per line
- Example location: `noxim/other/suitesparse_traces/web-Stanford_16/`

### 2. Configure Noxim
Create or modify YAML configuration:
```yaml
traffic_distribution: TRAFFIC_TRACE_BASED
trace_dir: path/to/trace/directory
```

### 3. Run Simulation
```bash
cd noxim/bin
./noxim -config ../config_examples/trace_based_sparse.yaml
```

### 4. Interpret Results
- Standard Noxim statistics apply (latency, throughput, etc.)
- feature_id is carried through network for potential future analysis
- PEs without trace files or with exhausted traces will not inject packets

## Design Decisions

### Why deterministic injection?
- Trace-based mode uses exact cycle timings from preprocessed sparse matrix workloads
- No randomization - directly replays communication patterns
- Enables reproducible experiments matching real GNN workload behavior

### Why feature_id field?
- Sparse matrices represent GNN feature vectors
- feature_id allows tracking which features are in flight
- Enables potential future work on feature-aware routing or prioritization
- Currently not used for routing decisions, but carried for analysis

### Backward compatibility
- All changes are opt-in via TRAFFIC_TRACE_BASED mode
- Existing traffic patterns unchanged
- trace_dir defaults to empty string, validated before use
- If trace file missing, PE prints warning and doesn't inject (graceful degradation)

## Testing

### Compilation
```bash
cd noxim/bin
make clean && make
```
Compiles successfully with no errors (only pre-existing warnings in other modules).

### Available Test Traces
Located in `noxim/other/suitesparse_traces/`:
- `web-Stanford_16/` - 281,903 nodes, 2.3M edges, 16 PEs
- `filter3D_16/` - 3D filter sparse matrix, 16 PEs
- `roadNet-PA_16/` - Road network Pennsylvania, 16 PEs
- `roadNet-TX_16/` - Road network Texas, 16 PEs
- `web-NotreDame_16/` - Notre Dame web graph, 16 PEs
- `cit-HepTh_16/` - HepTh citation network, 16 PEs

Each directory contains `pe_0.trace` through `pe_15.trace` files.

## Performance Characteristics

### Trace Loading
- Happens once at initialization (in PE constructor)
- Linear scan of trace file: O(n) where n = events in trace
- Stored in memory for fast lookup during simulation
- Minimal overhead after initialization

### Injection Decision
- Per-cycle check: O(1) comparison of current cycle vs next event cycle
- No random number generation (unlike other traffic modes)
- Index increment on successful injection
- Sequential access pattern (trace events pre-sorted by cycle)

## Future Enhancements

Potential extensions (not implemented):
1. **Feature-aware routing**: Use feature_id to make routing decisions
2. **Priority scheduling**: Prioritize certain features over others
3. **Trace compression**: Support compressed trace file formats
4. **Online trace generation**: Generate traces on-the-fly from matrix files
5. **Multi-phase traces**: Support multiple trace phases in single simulation
6. **Trace replay controls**: Pause, rewind, or loop trace playback

## Integration with SparseNoC

This implementation directly supports the SparseNoC architecture paper workflows:
- Traces generated from TAMU sparse matrix preprocessing pipeline
- 16-PE configuration matches experimental setup
- feature_id corresponds to row/column indices in sparse matrix
- Deterministic replay enables reproducible network performance evaluation

## Debugging Tips

### No packets injected?
1. Check that trace_dir is set correctly in YAML
2. Verify trace files exist: `ls {trace_dir}/pe_*.trace`
3. Look for "Warning: PE X - trace file not found" messages
4. Check trace file format (cycle src dst feature_id)

### Wrong traffic pattern?
1. Confirm `traffic_distribution: TRAFFIC_TRACE_BASED` in YAML
2. Check GlobalParams are loaded correctly (add debug prints)

### Simulation too short?
1. Find max cycle in trace files: `awk '{print $1}' pe_*.trace | sort -n | tail -1`
2. Set `simulation_time` > max cycle in YAML

### Performance issues?
1. Large trace files load into memory - monitor RAM usage
2. Consider splitting long simulations into phases
3. Profile with `-g -DDEBUG` compile flags if needed
