/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the declaration of the router
 */

#ifndef __NOXIMROUTER_H__
#define __NOXIMROUTER_H__

#include <systemc.h>
#include <unordered_map>
#include <bitset>
#include <array>
#include "DataStructs.h"
#include "Buffer.h"
#include "Stats.h"
#include "GlobalRoutingTable.h"
#include "LocalRoutingTable.h"
#include "ReservationTable.h"
#include "Utils.h"
#include "routingAlgorithms/RoutingAlgorithm.h"
#include "routingAlgorithms/RoutingAlgorithms.h"
#include "selectionStrategies/SelectionStrategy.h"
#include "selectionStrategies/SelectionStrategy.h"
#include "selectionStrategies/Selection_NOP.h"
#include "selectionStrategies/Selection_BUFFER_LEVEL.h"

using namespace std;

extern unsigned int drained_volume;

SC_MODULE(Router)
{
    friend class Selection_NOP;
    friend class Selection_BUFFER_LEVEL;

    // I/O Ports
    sc_in_clk clock;		                  // The input clock for the router
    sc_in <bool> reset;                           // The reset signal for the router

    // number of ports: 4 mesh directions + local + wireless 
    sc_in <Flit> flit_rx[DIRECTIONS + 2];	  // The input channels 
    sc_in <bool> req_rx[DIRECTIONS + 2];	  // The requests associated with the input channels
    sc_out <bool> ack_rx[DIRECTIONS + 2];	  // The outgoing ack signals associated with the input channels
    sc_out <TBufferFullStatus> buffer_full_status_rx[DIRECTIONS+2];

    sc_out <Flit> flit_tx[DIRECTIONS + 2];   // The output channels
    sc_out <bool> req_tx[DIRECTIONS + 2];	  // The requests associated with the output channels
    sc_in <bool> ack_tx[DIRECTIONS + 2];	  // The outgoing ack signals associated with the output channels
    sc_in <TBufferFullStatus> buffer_full_status_tx[DIRECTIONS+2];

    sc_out <int> free_slots[DIRECTIONS + 1];
    sc_in <int> free_slots_neighbor[DIRECTIONS + 1];

    // Neighbor-on-Path related I/O
    sc_out < NoP_data > NoP_data_out[DIRECTIONS];
    sc_in < NoP_data > NoP_data_in[DIRECTIONS];

    // Registers

    int local_id;		                // Unique ID
    int routing_type;		                // Type of routing algorithm
    int selection_type;
    BufferBank buffer[DIRECTIONS + 2];		// buffer[direction][virtual_channel] 
    bool current_level_rx[DIRECTIONS + 2];	// Legacy variable from ABP, not used in READY/VALID protocol
    bool current_level_tx[DIRECTIONS + 2];	// Legacy variable from ABP, not used in READY/VALID protocol
    
    // READY/VALID output registers - one flit per output port
    bool has_flit[DIRECTIONS + 2];              // True if output register contains a valid flit
    Flit out_reg[DIRECTIONS + 2];               // Output register holding flit to transmit
    
    Stats stats;		                // Statistics
    // Global end-to-end latency statistics (across all PEs)
    uint64_t global_latency_sum = 0;     // Sum of all request latencies
    uint64_t global_latency_count = 0;   // Number of completed requests
    uint64_t global_latency_max = 0;     // Maximum observed latency
    Power power;
    LocalRoutingTable routing_table;		// Routing table
    ReservationTable reservation_table;		// Switch reservation table
    unsigned long routed_flits;
    RoutingAlgorithm * routingAlgorithm; 
    SelectionStrategy * selectionStrategy;
    
    // Link utilization tracking
    uint64_t port_rx_busy[DIRECTIONS + 2];      // Cycles each input port received a flit
    uint64_t port_tx_busy[DIRECTIONS + 2];      // Cycles each output port transmitted a flit
    uint64_t total_observation_cycles;           // Total cycles observed (for utilization calc)
    uint64_t buffer_occupancy_sum[DIRECTIONS + 2]; // Sum of buffer occupancy over time
    uint64_t buffer_samples;                     // Number of samples taken 
    
    // Oracle coalescing instrumentation (stats only, no functional changes)
    struct OracleEntry {
        uint64_t first_cycle;   // when first HEAD for this feature arrived at this router
        int count;              // how many HEADs seen here (only unmarked ones)
        
        OracleEntry() : first_cycle(0), count(0) {}
    };
    
    std::unordered_map<int, OracleEntry> oracle_global;   // no time window, keyed by feature_id
    std::unordered_map<int, OracleEntry> oracle_window;   // 200-cycle window, keyed by feature_id
    
    // In-flight oracle: track outstanding requests until responses return
    struct OracleInflightEntry {
        uint64_t first_cycle;   // cycle when first request for this feature arrived (for debugging)
        uint32_t outstanding;   // number of outstanding requests for THIS batch at this router
        uint32_t pending_old_responses; // responses still expected from PREVIOUS batch (ignore these)
        
        OracleInflightEntry() : first_cycle(0), outstanding(0), pending_old_responses(0) {}
    };
    
    std::unordered_map<int, OracleInflightEntry> oracle_inflight;  // keyed by feature_id
    
    // Oracle statistics
    uint64_t oracle_global_total_heads;      // unmarked REQUEST HEADs seen
    uint64_t oracle_global_coalesced_heads;  // heads that participate in >1 at this router (no window)
    uint64_t oracle_window_total_heads;
    uint64_t oracle_window_coalesced_heads;
    uint64_t oracle_inflight_total_heads;      // all request heads considered
    uint64_t oracle_inflight_coalesced_heads;  // heads that arrived while another for same feature was outstanding
    
    // ========================================================================
    // SELECTIVE IN-ROUTER COALESCING MECHANISM
    // ========================================================================
    
    // Coalescing table entry
    struct CoalesceEntry {
        bool valid;                                // Entry is allocated
        int feature_id;                            // Feature being coalesced
        bool inflight;                             // Request has been forwarded downstream
        bool response_started;                     // Response arrived, multicast started - don't drop late requests
        std::bitset<8> requester_ports;            // Bitset of input ports (max 6 directions + local + hub)
        std::map<int, int> requester_pe_counts;    // Map: PE_ID -> count of requests from that PE
        std::map<int, std::set<int>> port_to_pe_ids;  // Map: Port -> set of PE IDs that requested from that port
        uint64_t allocation_cycle;                 // Cycle when this entry was allocated (for timeout cleanup)
        
        CoalesceEntry() : valid(false), feature_id(-1), inflight(false), response_started(false), allocation_cycle(0) {
            requester_ports.reset();
        }
    };
    
    // Coalescing table (128 entries per router)
    CoalesceEntry coalesce_table[COALESCE_TABLE_SIZE];
    
    // ========================================================================
    // MULTICAST ENGINE (separate from reservation table)
    // ========================================================================
    
    struct McPortState {
        bool needed;        // This output must receive this packet
        bool head_sent;     // Whether HEAD was sent to this output
        bool done;          // Whether TAIL was sent to this output
        bool reserved;      // Whether reservation table has reserved this (output, VC)
        int next_flit_idx;  // Index of next flit this port needs (0-based)
        int  vc;            // Chosen multicast VC (preserved from incoming traffic)
        
        McPortState() : needed(false), head_sent(false), done(false), reserved(false), next_flit_idx(0), vc(0) {}
    };
    
    struct McEntry {
        bool valid;
        int feature_id;
        int src_memtile;
        std::bitset<8> out_ports_needed;  // Which output ports need this packet
        std::vector<Flit> fifo;           // Stores packet flits in order (bounded)
        McPortState port[8];              // State for each output port (DIRECTIONS+2)
        int port_rr_start;                // Round-robin starting port for fair scheduling
        
        static const int MAX_FIFO_SIZE = 256;  // Maximum flits per multicast packet (e.g., 8 flits × 4 outputs)
        
        McEntry() : valid(false), feature_id(-1), src_memtile(-1), port_rr_start(0) {
            out_ports_needed.reset();
            fifo.reserve(MAX_FIFO_SIZE);  // Pre-allocate to avoid reallocation
        }
    };
    
    static const int MC_ENGINE_SIZE = 1024;  // Number of multicast engine entries
    McEntry mc_engine[MC_ENGINE_SIZE];
    
    // Mapping from (input_port, input_vc, feature_id) -> mc_entry_id (for collecting flits)
    // Using tuple to support multiple responses on same port+VC
    std::map<std::tuple<int,int,int>, int> input_to_mc_entry;
    
    // No longer used - multicast now preserves incoming VC instead of round-robin
    int multicast_vc_rr_counter;
    
    // Track which VCs are currently occupied by multicast engine on each output port
    // mc_vc_busy[output_port][vc] = true if multicast is using this VC on this port
    bool mc_vc_busy[DIRECTIONS+2][MAX_VIRTUAL_CHANNELS];  // 8 ports (DIRECTIONS+2), up to 16 VCs
    
    // Round-robin scheduling for multicast engine fairness
    int mc_rr_idx;  // Starting index for round-robin scan over mc_engine entries
    
    // Coalescing statistics
    uint64_t coalesce_requests_received;      // Total coalescing-eligible requests seen
    uint64_t coalesce_requests_merged;        // Requests that were merged (dropped at this router)
    uint64_t coalesce_table_full_events;      // Times table was full, couldn't coalesce
    uint64_t coalesce_responses_multicast;    // Response packets that were multicast
    uint64_t coalesce_multicast_flits_sent;   // Individual flit copies sent via multicast
    uint64_t coalesce_entries_timed_out;      // Entries freed due to timeout (>600 cycles)
    
    // Coalescing helper functions
    int findCoalesceEntry(int feature_id);                    // Find existing entry by feature_id
    int allocateCoalesceEntry(int feature_id);                // Allocate new entry
    void freeCoalesceEntry(int entry_idx);                    // Free entry
    void cleanupStaleCoalesceEntries();                       // Free entries stuck for >600 cycles
    bool handleRequestCoalescing(Flit &f, int input_dir);     // Try to coalesce request
    
    // Multicast engine helper functions
    int allocateMcEntry(int feature_id);                      // Allocate multicast engine entry
    void freeMcEntry(int mc_idx);                             // Free multicast engine entry
    int findMcEntry(int feature_id);                          // Find existing MC entry by feature_id
    void serveMcEngine();                                      // Serve multicast engine (Phase 0)
    std::vector<int> getMulticastOutputs(const Flit &f);      // Get output dirs for multicast response
    void printStuckMcEntries();                                // Debug: print McEngine entries that never completed
    
    // Functions

    void process();
    void rxProcess();		// The receiving process
    void txProcess();		// The transmitting process
    void perCycleUpdate();
    void configure(const int _id, const double _warm_up_time,
		   const unsigned int _max_buffer_size,
		   GlobalRoutingTable & grt);

    unsigned long getRoutedFlits();	// Returns the number of routed flits 

    // Constructor

    SC_CTOR(Router) {
        SC_METHOD(process);
        sensitive << reset;
        sensitive << clock.pos();

        SC_METHOD(perCycleUpdate);
        sensitive << reset;
        sensitive << clock.pos();

        routingAlgorithm = RoutingAlgorithms::get(GlobalParams::routing_algorithm);

        if (routingAlgorithm == 0)
        {
            cerr << " FATAL: invalid routing -routing " << GlobalParams::routing_algorithm << ", check with noxim -help" << endl;
            exit(-1);
        }

        selectionStrategy = SelectionStrategies::get(GlobalParams::selection_strategy);

        if (selectionStrategy == 0)
        {
            cerr << " FATAL: invalid selection strategy -sel " << GlobalParams::selection_strategy << ", check with noxim -help" << endl;
            exit(-1);
        }
    }

  private:

    // performs actual routing + selection
    int route(const RouteData & route_data);

    // wrappers
    int selectionFunction(const vector <int> &directions,
			  const RouteData & route_data);
    vector < int >routingFunction(const RouteData & route_data);
 
    NoP_data getCurrentNoPData();
    void NoP_report() const;
    int NoPScore(const NoP_data & nop_data, const vector <int> & nop_channels) const;
    int reflexDirection(int direction) const;
    int getNeighborId(int _id, int direction) const;
   
    vector<int> getNextHops(int src, int dst);
    int start_from_port;	     // Port from which to start the reservation cycle
    int start_from_vc[DIRECTIONS+2]; // VC from which to start the reservation cycle for the specific port

    vector<int> nextDeltaHops(RouteData rd);
  public:
    unsigned int local_drained;

    bool inCongestion();
    void ShowBuffersStats(std::ostream & out);

    bool connectedHubs(int src_hub, int dst_hub);
    
    // Link utilization statistics
    void printLinkUtilization() const;
    
    // Selective coalescing statistics
    void printSelectiveCoalescingStats() const;
    
    // Oracle coalescing tracking (instrumentation only)
    void trackOracleCoalescing(const Flit &f);
    void printOracleCoalescingStats() const;
};

#endif
