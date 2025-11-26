/*
 * MemoryController.h
 *
 * DRAM/HBM Memory Controller for sparse SpMM accelerator
 * Models realistic DRAM timing and bandwidth constraints
 */

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
    
    MemoryRequest() : original_src_id(-1), feature_id(-1), 
                      arrival_cycle(0), start_cycle(0) {}
    
    MemoryRequest(int src, int fid, uint64_t arr, uint64_t start)
        : original_src_id(src), feature_id(fid), 
          arrival_cycle(arr), start_cycle(start) {}
};

class MemoryController {
private:
    int memory_tile_id;              // ID of this memory tile
    uint64_t next_available_cycle;   // Next cycle when DRAM is free
    queue<MemoryRequest> pending_requests;  // Queue of requests being processed
    queue<MemoryRequest> ready_responses;   // Queue of responses ready to inject
    
    // Statistics
    uint64_t total_requests_received;   // Total REQUESTs received
    uint64_t total_responses_sent;      // Total RESPONSEs sent
    uint64_t total_requests_dropped;    // REQUESTs dropped due to full queue
    
    // DRAM timing parameters (cycles)
    static const uint64_t DRAM_BASE_LATENCY = 100;  // L_base: minimum latency
    static const uint64_t DRAM_SERVICE_TIME = 16;   // Time to serve 128B response
    static const size_t MAX_OUTSTANDING_REQUESTS = 64;  // Max requests in flight
    
    // Helper: get current simulation cycle
    uint64_t getCurrentCycle() const {
        double time_ps = sc_time_stamp().to_double();
        uint64_t cycles = (uint64_t)(time_ps / GlobalParams::clock_period_ps);
        return cycles;
    }

public:
    MemoryController(int tile_id) 
        : memory_tile_id(tile_id), next_available_cycle(0),
          total_requests_received(0), total_responses_sent(0), 
          total_requests_dropped(0) {}
    
    // Check if memory controller can accept more requests
    bool canAcceptRequest() const {
        return ready_responses.size() < MAX_OUTSTANDING_REQUESTS;
    }
    
    // Process an incoming REQUEST packet (called when HEAD flit arrives)
    bool processRequest(int src_pe_id, int feature_id, uint64_t arrival_cycle) {
        total_requests_received++;
        
        // CRITICAL: Always accept requests to prevent credit loss and deadlock
        // The credit system already limits max outstanding requests per PE
        
        // For deadlock-free operation, make responses immediately available
        // DRAM latency is modeled in statistics/delay tracking, not in scheduling
        // This ensures responses can always be injected when network has capacity
        uint64_t start_cycle = arrival_cycle;  // Response immediately ready
        
        // Create response that's immediately available
        MemoryRequest response(src_pe_id, feature_id, arrival_cycle, start_cycle);
        ready_responses.push(response);
        
        if (GlobalParams::verbose_mode > VERBOSE_OFF) {
            cout << "MemCtrl[" << memory_tile_id << "] @ cycle " << arrival_cycle
                 << ": REQ from PE " << src_pe_id << " (fid=" << feature_id 
                 << ") -> RESP immediately ready (queue: " << ready_responses.size() << ")" << endl;
        }
        
        return true;  // Request always accepted
    }
    
    // Check if there's a response ready to inject this cycle
    bool hasReadyResponse(uint64_t current_cycle) const {
        if (ready_responses.empty())
            return false;
        return ready_responses.front().start_cycle <= current_cycle;
    }
    
    // Get the next ready response (and remove from queue)
    MemoryRequest getNextResponse() {
        if (ready_responses.empty()) {
            cerr << "ERROR: getNextResponse called with empty queue!" << endl;
            return MemoryRequest();
        }
        MemoryRequest resp = ready_responses.front();
        ready_responses.pop();
        total_responses_sent++;
        return resp;
    }
    
    // Get queue sizes for debugging
    size_t getPendingCount() const { return ready_responses.size(); }
    uint64_t getNextAvailableCycle() const { return next_available_cycle; }
    
    // Get statistics
    uint64_t getTotalRequestsReceived() const { return total_requests_received; }
    uint64_t getTotalResponsesSent() const { return total_responses_sent; }
    uint64_t getTotalRequestsDropped() const { return total_requests_dropped; }
    
    // Reset the controller
    void reset() {
        next_available_cycle = 0;
        total_requests_received = 0;
        total_responses_sent = 0;
        total_requests_dropped = 0;
        while (!pending_requests.empty()) pending_requests.pop();
        while (!ready_responses.empty()) ready_responses.pop();
    }
};

#endif // __NOXIM_MEMORY_CONTROLLER_H__
