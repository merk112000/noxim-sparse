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
    bool current_level_rx;	// Current level for Alternating Bit Protocol (ABP)
    bool current_level_tx;	// Current level for Alternating Bit Protocol (ABP)
    queue < Packet > packet_queue;	// Local queue of packets
    bool transmittedAtPreviousCycle;	// Used for distributions with memory

    // Functions
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
    void loadTraceFile();		// Load trace file for this PE
    bool canShotTrace(Packet & packet);	// Trace-driven packet generation
    bool isMemoryTile(int id);		// Check if a tile ID is a memory controller
    int getTracePeId(int noxim_id);	// Map Noxim tile ID to trace PE ID

    // Memory controller support
    MemoryController* memory_controller;  // DRAM controller for memory tiles
    bool is_memory_tile;		  // True if this PE is a memory tile
    void handleIncomingRequest(const Flit& flit);  // Process REQUEST from PE
    bool canShotResponse(Packet & packet);	   // Generate RESPONSE packets

    // Credit-based flow control (PE -> Memory tile)
    map<int, int> memory_credits;	  // credits[mem_tile_id] = available credits
    void initMemoryCredits();		  // Initialize credits for all memory tiles
    bool hasCredit(int mem_tile_id);	  // Check if credits available
    void consumeCredit(int mem_tile_id);  // Consume 1 credit when sending REQUEST
    void returnCredit(int src_mem_tile);  // Return 1 credit when receiving RESPONSE

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

    // Constructor
    SC_CTOR(ProcessingElement) {
	SC_METHOD(rxProcess);
	sensitive << reset;
	sensitive << clock.pos();

	SC_METHOD(txProcess);
	sensitive << reset;
	sensitive << clock.pos();

	next_event_idx = 0;
	memory_controller = nullptr;
	is_memory_tile = false;
	// Note: Do NOT call loadTraceFile() here - local_id is not set yet!
	// Memory controller will be initialized in txProcess when local_id is known
    }

};

#endif
