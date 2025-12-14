/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the declaration of the processing element
 */

#ifndef __NOXIMPROCESSINGELEMENT_H__
#define __NOXIMPROCESSINGELEMENT_H__

#include <queue>
#include <vector>
#include <map>
#include <systemc.h>

#include "DataStructs.h"
#include "GlobalTrafficTable.h"
#include "GlobalTrafficHardcoding.h"
#include "Utils.h"
#include "MemoryController.h"

using namespace std;

// TraceEvent -- Structure for trace-driven traffic
struct TraceEvent {
    uint64_t cycle;
    int src;
    int dst;
    int feature_id;
    bool coalesce_hint;  // Hint from trace: eligible for selective coalescing
    
    TraceEvent() : cycle(0), src(-1), dst(-1), feature_id(-1), coalesce_hint(false) {}
};

SC_MODULE(ProcessingElement)
{

    // I/O Ports
    sc_in_clk clock;		// The input clock for the PE
    sc_in < bool > reset;	// The reset signal for the PE

    sc_in < Flit > flit_rx;	// The input channel
    sc_in < bool > req_rx;	// The request associated with the input channel
    sc_out < bool > ack_rx;	// The outgoing ack signal associated with the input channel
    sc_out < TBufferFullStatus > buffer_full_status_rx;	

    sc_out < Flit > flit_tx;	// The output channel
    sc_out < bool > req_tx;	// The request associated with the output channel
    sc_in < bool > ack_tx;	// The outgoing ack signal associated with the output channel
    sc_in < TBufferFullStatus > buffer_full_status_tx;

    sc_in < int >free_slots_neighbor;

    // Registers
    int local_id;		// Unique identification number
    bool current_level_rx;	// Legacy variable from ABP, not used in READY/VALID protocol
    bool current_level_tx;	// Legacy variable from ABP, not used in READY/VALID protocol
    
    // READY/VALID output register - one flit for TX
    bool has_flit;              // True if output register contains a valid flit
    Flit out_reg;               // Output register holding flit to transmit
    
    queue < Packet > packet_queue;	// Local queue of packets
    bool transmittedAtPreviousCycle;	// Used for distributions with memory
    
    // Heartbeat tracking
    uint64_t last_heartbeat_cycle;
    uint64_t total_requests_injected;
    uint64_t total_responses_injected;
    uint64_t total_responses_received;  // Number of RESPONSE packets received (HEAD count)

    // End-to-end latency tracking (REQUEST injection to RESPONSE arrival)
    map<int, uint64_t> request_injection_time;  // feature_id -> injection cycle (packet creation)
    map<int, uint64_t> request_network_entry_time;  // feature_id -> when HEAD flit enters network
    uint64_t total_e2e_latency;                  // sum of all latencies
    uint64_t max_e2e_latency;                    // maximum latency observed
    uint64_t e2e_latency_samples;                // number of completed REQUEST-RESPONSE pairs
    uint64_t total_pe_queue_delay;               // sum of time spent in source PE queue
    uint64_t max_pe_queue_delay;                 // maximum PE queue delay observed
    
    // Stall reason tracking (compute PEs only)
    uint64_t stall_cycles_no_credits;            // Cycles stalled due to no memory credits (memory backpressure)
    uint64_t stall_cycles_noc_contention;        // Cycles stalled due to NoC contention (buffer full)
    uint64_t stall_cycles_dram_bandwidth;        // Cycles stalled due to DRAM bandwidth limit (memory tiles)
    uint64_t total_injection_attempts;           // Total cycles attempted to inject

    // Functions
    void process();         // Master process that calls RX then TX in correct order
    void rxProcess();		// The receiving process
    void txProcess();		// The transmitting process
    bool canShot(Packet & packet);	// True when the packet must be shot
    Flit nextFlit();	// Take the next flit of the current packet
    Packet trafficTest();	// used for testing traffic
    Packet trafficRandom();	// Random destination distribution
    Packet trafficTranspose1();	// Transpose 1 destination distribution
    Packet trafficTranspose2();	// Transpose 2 destination distribution
    Packet trafficBitReversal();	// Bit-reversal destination distribution
    Packet trafficShuffle();	// Shuffle destination distribution
    Packet trafficButterfly();	// Butterfly destination distribution
    Packet trafficLocal();	// Random with locality
    Packet trafficULocal();	// Random with locality

    size_t traffic_cycle = 0;
    GlobalTrafficTable *traffic_table;	// Reference to the Global traffic Table
    GlobalTrafficHardcoding *traffic_hardcoded;	// Reference to the Global traffic Hardcoding
    bool never_transmit;	// true if the PE does not transmit any packet 
    //  (valid only for the table based traffic)

    // Trace-based traffic support
    vector<TraceEvent> trace_events;	// Loaded trace events for this PE
    size_t next_event_idx;		// Index of next event to inject
    uint64_t last_injection_cycle;	// Last cycle when we injected a request (for spacing)
    // REMOVED: next_vc_request, next_vc_response - now using randInt(0, n_virtual_channels-1) for all traffic
    void loadTraceFile();		// Load trace file for this PE
    bool canShotTrace(Packet & packet);	// Trace-driven packet generation
    bool isMemoryTile(int id);		// Check if a tile ID is a memory controller
    int getTracePeId(int noxim_id);	// Map Noxim tile ID to trace PE ID
    bool allTraceEventsSent() const;    // Check if all trace events have been injected
    uint64_t getInFlightRequests() const; // Get number of requests sent but not responded

    // Memory controller support
    MemoryController* memory_controller;  // DRAM controller for memory tiles
    bool is_memory_tile;		  // True if this PE is a memory tile
    void handleIncomingRequest(const Flit& flit);  // Process REQUEST from PE
    bool canShotResponse(Packet & packet);	   // Generate RESPONSE packets

    // Credit-based flow control (PE -> Memory tile)
    int total_memory_credits;		  // Total credits available (64 per PE, any destination)
    void initMemoryCredits();		  // Initialize credits
    bool hasCredit(int mem_tile_id);	  // Check if credits available
    void consumeCredit(int mem_tile_id);  // Consume 1 credit when sending REQUEST
    void returnCredit(int src_mem_tile);  // Return 1 credit when receiving RESPONSE
    
    // Debug tracking for missing responses
    struct OutstandingRequest {
        uint64_t injection_cycle;
        int dst_mem_tile;
        int feature_id;
    };
    std::map<uint64_t, OutstandingRequest> outstanding_requests;  // seq_num -> request info (NOT feature_id!)
    uint64_t next_request_seq;  // Unique sequence number for each request
    std::map<int, int> timeout_counts_per_feature;  // feature_id -> number of timeouts
    void checkMissingResponses();  // Check for requests that haven't received responses
    void printTimeoutStats() const;  // Print timeout statistics per feature

    void fixRanges(const Coord, Coord &);	// Fix the ranges of the destination
    int randInt(int min, int max);	// Extracts a random integer number between min and max
    int getRandomSize();	// Returns a random size in flits for the packet
    void setBit(int &x, int w, int v);
    int getBit(int x, int w);
    double log2ceil(double x);

    int roulett();
    int findRandomDestination(int local_id,int hops);
    unsigned int getQueueSize() const;
    
    // Memory controller statistics
    void printMemoryStats() const;	// Print memory controller stats (if memory tile)
    void printHeartbeat(int id, uint64_t cycle);  // Print heartbeat stats
    void printE2ELatencyStats() const;	// Print end-to-end latency stats (compute PEs only)
    void printStallStats() const;	// Print stall breakdown stats (compute PEs only)

    // Constructor
    SC_CTOR(ProcessingElement) {
	SC_METHOD(process);    // Master process
	sensitive << reset;
	sensitive << clock.pos();  // Run on rising edge

	next_event_idx = 0;
	last_injection_cycle = 0;
	// REMOVED: next_vc_request, next_vc_response initialization
	last_heartbeat_cycle = 0;
	next_request_seq = 0;
	total_requests_injected = 0;
	total_responses_injected = 0;
	total_e2e_latency = 0;
	max_e2e_latency = 0;
	e2e_latency_samples = 0;
	total_pe_queue_delay = 0;
	max_pe_queue_delay = 0;
	stall_cycles_no_credits = 0;
	stall_cycles_noc_contention = 0;
	stall_cycles_dram_bandwidth = 0;
	total_injection_attempts = 0;
	memory_controller = nullptr;
	is_memory_tile = false;
	// Note: Do NOT call loadTraceFile() here - local_id is not set yet!
	// Memory controller will be initialized in txProcess when local_id is known
    }

};

#endif
