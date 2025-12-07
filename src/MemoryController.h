
#ifndef __NOXIM_MEMORY_CONTROLLER_H__
#define __NOXIM_MEMORY_CONTROLLER_H__

#include <queue>
#include <systemc.h>
#include "DataStructs.h"
#include "GlobalParams.h"

using namespace std;

// Pending memory request structure
struct MemoryRequest {
    int original_src_id;     // Original PE that sent the REQUEST
    int feature_id;          // Feature ID or address
    uint64_t arrival_cycle;  // Cycle when REQUEST head arrived at memory tile
    uint64_t start_cycle;    // Cycle when RESPONSE will start injecting
    vector<int> recorded_path;  // Path recorded by the REQUEST (for reverse routing)
    
    MemoryRequest() : original_src_id(-1), feature_id(-1), 
                      arrival_cycle(0), start_cycle(0) {}
    
    MemoryRequest(int src, int fid, uint64_t arr, uint64_t start)
        : original_src_id(src), feature_id(fid), 
          arrival_cycle(arr), start_cycle(start) {}
    
    MemoryRequest(int src, int fid, uint64_t arr, uint64_t start, const vector<int>& path)
        : original_src_id(src), feature_id(fid), 
          arrival_cycle(arr), start_cycle(start), recorded_path(path) {}
};

class MemoryController {
private:
    int memory_tile_id;              // ID of this memory tile
    uint64_t next_available_cycle;   // Next cycle when DRAM can start processing next request
    uint64_t last_flit_injection_cycle;      // Last cycle ANY flit was injected (for flit-level BW control)
    uint64_t last_request_accepted_cycle;    // Last cycle a request was accepted (for ingress BW)
    queue<MemoryRequest> pending_requests;  // Queue of requests being processed
    queue<MemoryRequest> ready_responses;   // Queue of responses ready to inject
    
    // Statistics
    uint64_t total_requests_received;   // Total REQUESTs received
    uint64_t total_responses_sent;      // Total RESPONSEs sent
    uint64_t total_requests_dropped;    // REQUESTs dropped due to full queue
    uint64_t in_flight_requests;        // Requests accepted but responses not yet injected
    
    // Link utilization tracking
    uint64_t total_cycles_observed;     // Total cycles for utilization measurement
    uint64_t ingress_busy_cycles;       // Cycles with incoming REQUEST flit
    uint64_t egress_busy_cycles;        // Cycles with outgoing RESPONSE flit
    uint64_t first_request_cycle;       // First cycle we received a request (for accurate util calc)
    
    // DRAM timing parameters (cycles)
    static const uint64_t DRAM_BASE_LATENCY = 100;  // L_base: minimum latency (DRAM row access)
    // INTERVAL=0 for maximum bandwidth, with packet_queue backpressure to prevent deadlock
    static const uint64_t DRAM_FLIT_INJECTION_INTERVAL = 0;  // Cycles between consecutive FLIT injections (0 = back-to-back, 1 = 50%, 2 = 33%)
    static const uint64_t RESPONSE_SIZE_FLITS = 4; // Number of flits per response (changed from 3 to 4)
    static const size_t MAX_OUTSTANDING_REQUESTS = 32;  // MSHR depth per memory tile - limited by packet_queue backpressure
    
    // Helper: get current simulation cycle
    uint64_t getCurrentCycle() const {
        double time_ps = sc_time_stamp().to_double();
        uint64_t cycles = (uint64_t)(time_ps / GlobalParams::clock_period_ps);
        return cycles;
    }

public:
    MemoryController(int tile_id) 
        : memory_tile_id(tile_id), next_available_cycle(0),
          last_flit_injection_cycle(0), last_request_accepted_cycle(0),
          total_requests_received(0), total_responses_sent(0), 
          total_requests_dropped(0), in_flight_requests(0), total_cycles_observed(0),
          ingress_busy_cycles(0), egress_busy_cycles(0), first_request_cycle(0) {}
    
    // Check if memory controller can accept more requests
    // Use ready_responses queue size (actual pending work) not in_flight_requests
    bool canAcceptRequest() const {
         return ready_responses.size() < MAX_OUTSTANDING_REQUESTS;
    }
    
    // Process an incoming REQUEST packet (called when HEAD flit arrives)
    bool processRequest(int src_pe_id, int feature_id, uint64_t arrival_cycle, 
                        const vector<int>& recorded_path = vector<int>()) {
        total_requests_received++;
        
        // PE credits (26 per PE × 9 PEs = 234 total) are system-wide limit
        // Memory tile MSHRs (64 per tile) are per-controller hardware limit
        // Credits prevent total system overload, MSHRs limit per-tile capacity
        
        // REALISTIC: Memory tile MSHRs can fill up even with credits!
        // This happens when traffic has hotspots (many PEs target same tile)
        if (ready_responses.size() >= MAX_OUTSTANDING_REQUESTS) {
            cout << "WARNING: MemCtrl[" << memory_tile_id << "] MSHR full! "
                 << "Queue: " << ready_responses.size() << "/" << MAX_OUTSTANDING_REQUESTS
                 << " - Hotspot detected, applying backpressure!" << endl;
            total_requests_dropped++;
            return false;
        }
        
        // Process request with DRAM timing
        // With MSHRs, requests can overlap! DRAM processes immediately (no serialization)
        // Response ready after: DRAM access latency (base latency is for data fetch)
        uint64_t response_ready = arrival_cycle + DRAM_BASE_LATENCY;
        
        // NOTE: next_available_cycle is NOT updated here anymore!
        // Requests overlap fully using MSHRs (up to 64 in-flight)
        // SERVICE_TIME only limits response INJECTION rate (modeled in hasReadyResponse)
        
        // Create response with realistic DRAM timing, including recorded path for reverse routing
        MemoryRequest response(src_pe_id, feature_id, arrival_cycle, response_ready, recorded_path);
        ready_responses.push(response);
        in_flight_requests++;  // Track in-flight requests
        
        if (GlobalParams::verbose_mode > VERBOSE_OFF) {
            cout << "MemCtrl[" << memory_tile_id << "] @ cycle " << arrival_cycle
                 << ": REQ from PE " << src_pe_id << " -> RESP @ " << response_ready 
                 << " (MSHR: " << ready_responses.size() << "/" << MAX_OUTSTANDING_REQUESTS << ")" << endl;
        }
        
        return true;  // Request accepted
    }
    
    // Check if there's a response ready to inject (packet-level check)
    // Only checks if DRAM processing is complete, not flit-level bandwidth
    // Flit-level bandwidth control happens in canInjectFlit()
    bool hasReadyResponse(uint64_t current_cycle) const {
        if (ready_responses.empty())
            return false;
        
        // Response must be ready (DRAM processing complete)
        return ready_responses.front().start_cycle <= current_cycle;
    }
    
    // Check if we can inject a flit this cycle (flit-level bandwidth control)
    // This is called before EACH flit injection to enforce DRAM data channel bandwidth
    bool canInjectFlit(uint64_t current_cycle) const {
        // FLIT-LEVEL BANDWIDTH CONTROL:
        // Can only inject one flit every DRAM_FLIT_INJECTION_INTERVAL cycles
        // With INTERVAL=2 and RESPONSE_SIZE=4:
        //   Response A: flits at cycles 0, 2, 4, 6
        //   Response B: flits at cycles 8, 10, 12, 14 (can start at 6+2=8)
        // Result: 4 flits over 8 cycles = 0.5 flits/cycle = 26.67 GB/s
        // More realistic than burst-then-idle pattern
        return (current_cycle >= last_flit_injection_cycle + DRAM_FLIT_INJECTION_INTERVAL);
    }
    
    // Track that a flit was injected this cycle
    void notifyFlitInjected(uint64_t current_cycle) {
        last_flit_injection_cycle = current_cycle;
    }
    
    // Get the next ready response (and remove from queue)
    MemoryRequest getNextResponse(uint64_t current_cycle) {
        if (ready_responses.empty()) {
            cerr << "ERROR: getNextResponse called with empty queue!" << endl;
            return MemoryRequest();
        }
        MemoryRequest resp = ready_responses.front();
        ready_responses.pop();
        total_responses_sent++;
        in_flight_requests--;  // Response injected, no longer in-flight
        
        // NOTE: Flit-level bandwidth tracking happens in notifyFlitInjected()
        // which is called from ProcessingElement after each flit transmission
        
        return resp;
    }
    
    // Force a response (used when request is dropped but credit must be returned)
    // NOTE: This bypasses the queue limit to ensure credit is returned

    
    // Get queue sizes for debugging
    size_t getPendingCount() const { return ready_responses.size(); }
    uint64_t getInFlightCount() const { return in_flight_requests; }
    uint64_t getNextAvailableCycle() const { return next_available_cycle; }
    
    // Get statistics
    uint64_t getTotalRequestsReceived() const { return total_requests_received; }
    uint64_t getTotalResponsesSent() const { return total_responses_sent; }
    uint64_t getTotalRequestsDropped() const { return total_requests_dropped; }
    
    // Link utilization tracking
    void trackIngressActivity() { 
        ingress_busy_cycles++; 
    }
    
    void trackEgressActivity() { 
        egress_busy_cycles++; 
    }
    
    void updateTotalCycles(uint64_t current_cycle) {
        if (first_request_cycle == 0) return;  // Haven't started yet
        total_cycles_observed = current_cycle - first_request_cycle;
    }
    
    void markFirstRequest(uint64_t cycle) {
        if (first_request_cycle == 0) {
            first_request_cycle = cycle;
        }
    }
    
    // Get link utilization percentages
    double getIngressUtilization() const {
        if (total_cycles_observed == 0) return 0.0;
        return 100.0 * ingress_busy_cycles / total_cycles_observed;
    }
    
    double getEgressUtilization() const {
        if (total_cycles_observed == 0) return 0.0;
        return 100.0 * egress_busy_cycles / total_cycles_observed;
    }
    
    uint64_t getIngressBusyCycles() const { return ingress_busy_cycles; }
    uint64_t getEgressBusyCycles() const { return egress_busy_cycles; }
    uint64_t getTotalCyclesObserved() const { return total_cycles_observed; }
    
    // Reset the controller
    void reset() {
        next_available_cycle = 0;
        total_requests_received = 0;
        total_responses_sent = 0;
        total_requests_dropped = 0;
        in_flight_requests = 0;
        total_cycles_observed = 0;
        ingress_busy_cycles = 0;
        egress_busy_cycles = 0;
        first_request_cycle = 0;
        while (!pending_requests.empty()) pending_requests.pop();
        while (!ready_responses.empty()) ready_responses.pop();
    }
};

#endif // __NOXIM_MEMORY_CONTROLLER_H__
/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the declaration of the router
 */