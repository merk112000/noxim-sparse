#include "ProcessingElement.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <set>

const unsigned int MAX_NI_QUEUE_SIZE = 64;

int ProcessingElement::randInt(int min, int max)
{
    return min +
	(int) ((double) (max - min + 1) * rand() / (RAND_MAX + 1.0));
}

// Master process: ensures RX runs before TX within same cycle
void ProcessingElement::process()
{
    rxProcess();   // RX first - receive flits and return credits
    txProcess();   // TX second - check credits and send (credits from RX are immediately available)
}

void ProcessingElement::rxProcess()
{
    if (reset.read()) {
	ack_rx.write(true);  // READY/VALID: signal READY on reset
	current_level_rx = 0;  // kept for compatibility only, not used in protocol
	if (memory_controller) {
	    memory_controller->reset();
	}
    } else {
	// READY/VALID protocol: req_rx is VALID, ack_rx is READY
	bool valid = req_rx.read();  // upstream VALID signal
	bool ready = true;  // assume we're READY unless we need backpressure
	
	if (valid) {
	    Flit flit_tmp = flit_rx.read();
	    
	    // BACKPRESSURE: If this is a memory tile receiving a REQUEST HEAD and MSHR is full, signal NOT READY
	    if (is_memory_tile && 
	        flit_tmp.packet_type == PACKET_TYPE_REQUEST && 
	        (flit_tmp.flit_type == FLIT_TYPE_HEAD || flit_tmp.flit_type == FLIT_TYPE_HEAD_TAIL)) {
	        
	       // cerr << "PE[" << local_id << "] Memory tile received REQUEST HEAD fid=" << flit_tmp.feature_id << " from " << flit_tmp.src_id << endl;
	        
	        // Check both MSHR queue AND packet_queue (egress packets waiting to be sent)
	        // If packet_queue is full, we can't generate more response packets!
	        const size_t MAX_PACKET_QUEUE = 128;  // Limit egress packet queue size
	        bool mshr_has_space = memory_controller->canAcceptRequest();
	        bool packet_queue_has_space = (packet_queue.size() < MAX_NI_QUEUE_SIZE);
	        
	        if (!mshr_has_space || !packet_queue_has_space) {
	            // MSHR or packet queue FULL! Signal NOT READY - router will buffer the flit
	            ready = false;
	            if (GlobalParams::verbose_mode > VERBOSE_OFF) {
	                uint64_t cur_cycle = static_cast<uint64_t>(
	                    sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
	                cout << "BACKPRESSURE @ cycle " << cur_cycle 
	                     << ": MemTile[" << local_id << "] "
	                     << (mshr_has_space ? "" : "MSHR full, ")
	                     << (packet_queue_has_space ? "" : "Packet queue full, ")
	                     << "signaling NOT READY for REQUEST from PE[" 
	                     << flit_tmp.src_id << "]" 
	                     << " (MSHR: " << memory_controller->getPendingCount() 
	                     << ", PktQ: " << packet_queue.size() << "/" << MAX_PACKET_QUEUE << ")" << endl;
	            }
	        } else {
	            // MSHR has space - accept the request (ready=true, will be processed)
	            // Track ingress link utilization ONLY when actually accepting
	            if (memory_controller) {
	                memory_controller->trackIngressActivity();
	            }
	            handleIncomingRequest(flit_tmp);
	        }
    } else {
        // For non-memory tiles or non-REQUEST-HEAD flits, always accept (no backpressure needed)
        // Process RESPONSE flits for compute PEs
        if (!is_memory_tile && flit_tmp.packet_type == PACKET_TYPE_RESPONSE) {
            
            // Handle RESPONSE HEAD - return credit and track latency
            if (flit_tmp.flit_type == FLIT_TYPE_HEAD) {
                // CRITICAL: If response reached this PE's LOCAL port, it MUST be for us!
                // Router wouldn't send it here otherwise (either via coalesce_table multicast or unicast routing)
                // Always return credit when response HEAD arrives at PE
                
               /* cerr << "PE[" << local_id << "] Received RESPONSE HEAD from MemTile " << flit_tmp.src_id
                     << " feature=" << flit_tmp.feature_id << " vc=" << flit_tmp.vc_id 
                     << " - RETURNING CREDIT (now " << (total_memory_credits+1) << ")" << endl;*/
                
                total_responses_received++;  // Count responses received
                
                // Return credits - may be >1 if this response covers multiple coalesced requests from this PE
                if (flit_tmp.credit_count > 1 && GlobalParams::simulation_time <= 50000) {
                    cerr << "PE[" << local_id << "] Returning " << flit_tmp.credit_count 
                         << " credits for feature " << flit_tmp.feature_id << endl;
                }
                for (int i = 0; i < flit_tmp.credit_count; i++) {
                    returnCredit(flit_tmp.src_id);
                }
                
                // Track end-to-end latency (REQUEST injection to RESPONSE arrival)
                int feature_id = flit_tmp.feature_id;
                if (request_injection_time.count(feature_id) > 0) {
                    uint64_t cur_cycle = static_cast<uint64_t>(
                        sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
                    uint64_t injection_cycle = request_injection_time[feature_id];
                    uint64_t latency = cur_cycle - injection_cycle;
                    
                    total_e2e_latency += latency;
                    e2e_latency_samples++;
                    if (latency > max_e2e_latency) {
                        max_e2e_latency = latency;
                    }
                    
                    // Track PE queue delay (time from packet creation to network entry)
                    if (request_network_entry_time.count(feature_id) > 0) {
                        uint64_t network_entry = request_network_entry_time[feature_id];
                        uint64_t pe_queue_delay = network_entry - injection_cycle;
                        total_pe_queue_delay += pe_queue_delay;
                        if (pe_queue_delay > max_pe_queue_delay) {
                            max_pe_queue_delay = pe_queue_delay;
                        }
                        request_network_entry_time.erase(feature_id);
                    }
                    
                    // Remove from map to free memory
                    request_injection_time.erase(feature_id);
                    
                    // Remove from outstanding requests tracking
                    // NOTE: outstanding_requests is indexed by sequence number, not feature_id
                    // We need to find and remove the OLDEST request with this feature_id
                    // (should be the first one that was sent, FIFO order for same feature)
                    uint64_t seq_to_remove = UINT64_MAX;
                    uint64_t oldest_injection_cycle = UINT64_MAX;
                    for (const auto& entry : outstanding_requests) {
                        if (entry.second.feature_id == feature_id) {
                            // Found a matching feature_id - check if it's older
                            if (entry.second.injection_cycle < oldest_injection_cycle) {
                                oldest_injection_cycle = entry.second.injection_cycle;
                                seq_to_remove = entry.first;
                            }
                        }
                    }
                    if (seq_to_remove != UINT64_MAX) {
                        outstanding_requests.erase(seq_to_remove);
                    }
                }
            }
            // Note: BODY and TAIL flits are silently consumed (no action needed)
            // The flit will be accepted (ready=true) and discarded
        }
    }
	}
	
	// Always write READY status to upstream
	ack_rx.write(ready);
    }
}

void ProcessingElement::txProcess()
{
    if (reset.read()) {
	req_tx.write(false);      // No VALID on reset
	has_flit = false;         // Output register empty
	current_level_tx = 0;     // Legacy, not used
	transmittedAtPreviousCycle = false;
    } else {

    // Lazy initialization of memory controller and credits (local_id now available)
    if (memory_controller == nullptr) {
        is_memory_tile = isMemoryTile(local_id);
        if (is_memory_tile) {
            memory_controller = new MemoryController(local_id);
        } else {
            // Compute PEs need to initialize credits for memory tiles
            initMemoryCredits();
            // Load trace file immediately for compute PEs
            if (GlobalParams::traffic_distribution == TRAFFIC_TRACE_BASED) {
                loadTraceFile();
            }
        }
    }

    // ========================================================================
    // PHASE 1: Drive output from register and handle handshake completion
    // ========================================================================
    if (has_flit) {
	// We have a flit in the output register - drive VALID and data
	flit_tx->write(out_reg);
	req_tx.write(true);  // Assert VALID
	
	// Check if handshake completes this cycle (VALID=1 && READY=1)
	bool ready = ack_tx.read();
	if (ready) {
	    // Handshake completed! Clear the output register
	    has_flit = false;
	    
	    // Track egress link utilization for memory tiles (RESPONSE flits departing)
	    if (is_memory_tile && out_reg.packet_type == PACKET_TYPE_RESPONSE && memory_controller) {
		memory_controller->trackEgressActivity();
		// Notify memory controller that a flit was injected (for bandwidth tracking)
		uint64_t current_cycle = static_cast<uint64_t>(
		    sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
		memory_controller->notifyFlitInjected(current_cycle);
	    }
	}
    } else {
	// No flit in output register - deassert VALID
	req_tx.write(false);
    }
    
    // ========================================================================
    // PHASE 2: Generate new packets and load output register (if empty)
    // ========================================================================
    // Only generate packets if output register is empty (backpressure from network handled by has_flit)
    static int debug_phase2 = 0;
    if (debug_phase2++ < 10 && GlobalParams::verbose_mode >= VERBOSE_LOW) {
        cerr << "PE[" << local_id << "] PHASE 2: has_flit=" << has_flit 
             << ", is_memory_tile=" << is_memory_tile << endl;
    }
    
    if (!has_flit) {
	// Memory tiles generate RESPONSE packets
	if (is_memory_tile) {

	    // Each response is multiple flits, so queue needs sufficient depth
	    
	    
	    if (packet_queue.size() < MAX_NI_QUEUE_SIZE) {
		Packet packet;
		if (canShotResponse(packet)) {
		    packet_queue.push(packet);
		    transmittedAtPreviousCycle = true;
		    total_responses_injected++;
		} else {
		    transmittedAtPreviousCycle = false;
		}
	    } else {
		transmittedAtPreviousCycle = false;
	    }
	}
	// Compute PEs generate REQUEST packets
	else if(GlobalParams::traffic_distribution != TRAFFIC_HARDCODED) {
	    // Compute PEs use credit-based flow control, so no need for NI queue limit
	    // (credits already limit outstanding requests to prevent network flooding)
	    static int debug_count = 0;
	    if (debug_count++ < 5 && GlobalParams::verbose_mode >= VERBOSE_LOW) {
	        cerr << "PE[" << local_id << "] Compute PE block reached, calling canShot()" << endl;
	    }
	    
	    Packet packet;
	    if (canShot(packet)) {
		packet_queue.push(packet);
		transmittedAtPreviousCycle = true;
		total_requests_injected++;
	    } else {
		transmittedAtPreviousCycle = false;
	    }
	} else if(traffic_cycle < traffic_hardcoded->num_cycles()) {
	    double now = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
	    
	    bool any = false;
	    for (HardcodedTrafficEntry const& expected_packet
		       : traffic_hardcoded->traffic_at_cycle(traffic_cycle)) {
		if(expected_packet.src == local_id) {
		    Packet packet;
		    int vc = randInt(0, GlobalParams::n_virtual_channels - 1);
		    packet.make(local_id, expected_packet.dst, vc, now, getRandomSize());
		    packet_queue.push(packet);
		    any = true;
		}
	    }

	    if(any)
		transmittedAtPreviousCycle = true;
	    else
		transmittedAtPreviousCycle = false;
	    
	    traffic_cycle += 1;
	}
    }

    // ========================================================================
    // PHASE 3: Load output register from packet queue (if register is empty)
    // ========================================================================
    if (!has_flit && !packet_queue.empty()) {
	// Check if we can inject this flit (DRAM bandwidth limiting for memory tiles)
	bool can_inject = true;
	if (is_memory_tile && memory_controller && packet_queue.front().packet_type == PACKET_TYPE_RESPONSE) {
	    uint64_t current_cycle = static_cast<uint64_t>(
		sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
	    can_inject = memory_controller->canInjectFlit(current_cycle);
	}
	
	// CRITICAL: Check router buffer status BEFORE loading flit
	// Memory tiles need to respect router backpressure at injection time
	bool router_can_accept = true;
	if (is_memory_tile) {
	    // Check if router has space for this packet's VC
	    // The packet's VC is already assigned, so check that specific VC
	    TBufferFullStatus bfs = buffer_full_status_tx.read();
	    int packet_vc = packet_queue.front().vc_id;
	    router_can_accept = !bfs.mask[packet_vc];
	}
	
	if (can_inject && router_can_accept) {
	    // Generate next flit and load into output register
	    Flit flit = nextFlit();  // This may pop packet if last flit
	    out_reg = flit;
	    has_flit = true;
	} else {
	    // DRAM bandwidth limiting or router buffer full - can't load this cycle
	    if (is_memory_tile) {
		if (!can_inject) {
		    stall_cycles_dram_bandwidth++;
		} else if (!router_can_accept) {
		    stall_cycles_noc_contention++;
		}
	    }
	}
    } else if (!has_flit && packet_queue.empty()) {
	// No packets to send - already handled by req_tx.write(false) in Phase 1
    } else if (has_flit) {
	// Output register occupied - will try again next cycle after handshake
	// This is normal operation - no stall counting here
    }
    
    }
}

Flit ProcessingElement::nextFlit()
{
    Flit flit;
    Packet packet = packet_queue.front();

    flit.src_id = packet.src_id;
    flit.dst_id = packet.dst_id;
    flit.vc_id = packet.vc_id;
    flit.timestamp = packet.timestamp;
    flit.sequence_no = packet.size - packet.flit_left;
    flit.sequence_length = packet.size;
    flit.hop_no = 0;
    //  flit.payload     = DEFAULT_PAYLOAD;
    flit.feature_id = packet.feature_id;  // Propagate feature_id from packet to flit
    flit.packet_type = packet.packet_type; // Propagate packet type
    flit.recorded_path = packet.recorded_path;  // Propagate recorded path for reverse routing
    flit.multicast_dests = packet.multicast_dests;  // Propagate multicast destinations for response multicasting
    flit.oracle_coalesce_marked = false;  // Initialize oracle coalescing marker
    flit.coalesce_hint = packet.coalesce_hint;  // Propagate coalescing hint
    flit.coalesce_allowed = (packet.packet_type == PACKET_TYPE_REQUEST && packet.coalesce_hint);  // Initially allow coalescing for eligible REQUESTs
    flit.multicast_allowed = packet.multicast_allowed;  // Propagate per-request multicast permission from packet
    flit.multicast_root_id = packet.multicast_root_id;  // Propagate multicast root router ID
    flit.original_src_id = packet.src_id;  // Track original requester for multicast
    flit.credit_count = 1;  // Default: return 1 credit (will be updated by coalesce table for duplicates)

    flit.hub_relay_node = NOT_VALID;

    // Handle single-flit packets as both HEAD and TAIL
    if (packet.size == 1) {
	flit.flit_type = FLIT_TYPE_HEAD_TAIL;
    } else if (packet.size == packet.flit_left) {
	flit.flit_type = FLIT_TYPE_HEAD;
    } else if (packet.flit_left == 1) {
	flit.flit_type = FLIT_TYPE_TAIL;
    } else {
	flit.flit_type = FLIT_TYPE_BODY;
    }

    // Track when HEAD flit of REQUEST enters network (for PE queue delay measurement)
    if ((flit.flit_type == FLIT_TYPE_HEAD || flit.flit_type == FLIT_TYPE_HEAD_TAIL) && 
        flit.packet_type == PACKET_TYPE_REQUEST &&
        !is_memory_tile) {
        uint64_t cur_cycle = static_cast<uint64_t>(
            sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
        request_network_entry_time[flit.feature_id] = cur_cycle;
    }

    packet_queue.front().flit_left--;
    if (packet_queue.front().flit_left == 0)
	packet_queue.pop();

    return flit;
}

bool ProcessingElement::canShot(Packet & packet)
{
    static int debug_count = 0;
    if (debug_count++ < 10 && GlobalParams::verbose_mode >= VERBOSE_LOW) {
        cerr << "PE[" << local_id << "] canShot() called, is_memory_tile=" << is_memory_tile 
             << ", traffic_dist=" << GlobalParams::traffic_distribution << endl;
    }
    
   // assert(false);
    if(never_transmit) return false;
   
    //if(local_id!=16) return false;
    /* DEADLOCK TEST 
	double current_time = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;

	if (current_time >= 4100) 
	{
	    //if (current_time==3500)
	         //cout << name() << " IN CODA " << packet_queue.size() << endl;
	    return false;
	}
	//*/

#ifdef DEADLOCK_AVOIDANCE
    if (local_id%2==0)
	return false;
#endif
    bool shot;
    double threshold;

    double now = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;

    // Trace-based traffic mode - deterministic injection from trace files
    if (GlobalParams::traffic_distribution == TRAFFIC_TRACE_BASED) {
        shot = canShotTrace(packet);
        if (shot) {
            // Successfully scheduled trace event - advance to next
            next_event_idx++;
        }
    } else if (GlobalParams::traffic_distribution != TRAFFIC_TABLE_BASED) {
	if (!transmittedAtPreviousCycle)
	    threshold = GlobalParams::packet_injection_rate;
	else
	    threshold = GlobalParams::probability_of_retransmission;

	shot = (((double) rand()) / RAND_MAX < threshold);
	if (shot) {
	    if (GlobalParams::traffic_distribution == TRAFFIC_RANDOM)
		    packet = trafficRandom();
        else if (GlobalParams::traffic_distribution == TRAFFIC_TRANSPOSE1)
		    packet = trafficTranspose1();
        else if (GlobalParams::traffic_distribution == TRAFFIC_TRANSPOSE2)
    		packet = trafficTranspose2();
        else if (GlobalParams::traffic_distribution == TRAFFIC_BIT_REVERSAL)
		    packet = trafficBitReversal();
        else if (GlobalParams::traffic_distribution == TRAFFIC_SHUFFLE)
		    packet = trafficShuffle();
        else if (GlobalParams::traffic_distribution == TRAFFIC_BUTTERFLY)
		    packet = trafficButterfly();
        else if (GlobalParams::traffic_distribution == TRAFFIC_LOCAL)
		    packet = trafficLocal();
        else if (GlobalParams::traffic_distribution == TRAFFIC_ULOCAL)
		    packet = trafficULocal();
        else {
            cout << "Invalid traffic distribution: " << GlobalParams::traffic_distribution << endl;
            exit(-1);
        }
	}
    } else {			// Table based communication traffic
	if (never_transmit)
	    return false;

	bool use_pir = (transmittedAtPreviousCycle == false);
	vector < pair < int, double > > dst_prob;
	double threshold =
	    traffic_table->getCumulativePirPor(local_id, (int) now, use_pir, dst_prob);

	double prob = (double) rand() / RAND_MAX;
	shot = (prob < threshold);
	if (shot) {
	    for (unsigned int i = 0; i < dst_prob.size(); i++) {
		if (prob < dst_prob[i].second) {
                    int vc = randInt(0, GlobalParams::n_virtual_channels - 1);
		    packet.make(local_id, dst_prob[i].first, vc, now, getRandomSize());
		    break;
		}
	    }
	}
    }

    return shot;
}


Packet ProcessingElement::trafficLocal()
{
    Packet p;
    p.src_id = local_id;
    double rnd = rand() / (double) RAND_MAX;

    vector<int> dst_set;

    int max_id = (GlobalParams::mesh_dim_x * GlobalParams::mesh_dim_y);

    for (int i=0;i<max_id;i++)
    {
	if (rnd<=GlobalParams::locality)
	{
	    if (local_id!=i && sameRadioHub(local_id,i))
		dst_set.push_back(i);
	}
	else
	    if (!sameRadioHub(local_id,i))
		dst_set.push_back(i);
    }


    int i_rnd = rand()%dst_set.size();

    p.dst_id = dst_set[i_rnd];
    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();
    p.vc_id = randInt(0, GlobalParams::n_virtual_channels - 1);
    
    return p;
}


int ProcessingElement::findRandomDestination(int id, int hops)
{
    assert(GlobalParams::topology == TOPOLOGY_MESH);

    int inc_y = rand()%2?-1:1;
    int inc_x = rand()%2?-1:1;
    
    Coord current =  id2Coord(id);
    


    for (int h = 0; h<hops; h++)
    {

	if (current.x==0)
	    if (inc_x<0) inc_x=0;

	if (current.x== GlobalParams::mesh_dim_x-1)
	    if (inc_x>0) inc_x=0;

	if (current.y==0)
	    if (inc_y<0) inc_y=0;

	if (current.y==GlobalParams::mesh_dim_y-1)
	    if (inc_y>0) inc_y=0;

	if (rand()%2)
	    current.x +=inc_x;
	else
	    current.y +=inc_y;
    }
    return coord2Id(current);
}


int roulette()
{
    int slices = GlobalParams::mesh_dim_x + GlobalParams::mesh_dim_y -2;


    double r = rand()/(double)RAND_MAX;


    for (int i=1;i<=slices;i++)
    {
	if (r< (1-1/double(2<<i)))
	{
	    return i;
	}
    }
    assert(false);
    return 1;
}


Packet ProcessingElement::trafficULocal()
{
    Packet p;
    p.src_id = local_id;

    int target_hops = roulette();

    p.dst_id = findRandomDestination(local_id,target_hops);

    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();
    p.vc_id = randInt(0, GlobalParams::n_virtual_channels - 1);

    return p;
}

Packet ProcessingElement::trafficRandom()
{
    Packet p;
    p.src_id = local_id;
    double rnd = rand() / (double) RAND_MAX;
    double range_start = 0.0;
    int max_id;

    if (GlobalParams::topology == TOPOLOGY_MESH)
	max_id = (GlobalParams::mesh_dim_x * GlobalParams::mesh_dim_y) - 1; //Mesh 
    else    // other delta topologies
	max_id = GlobalParams::n_delta_tiles-1; 

    // Random destination distribution
    do {
	p.dst_id = randInt(0, max_id);

	// check for hotspot destination
	for (size_t i = 0; i < GlobalParams::hotspots.size(); i++) {

	    if (rnd >= range_start && rnd < range_start + GlobalParams::hotspots[i].second) {
		if (local_id != GlobalParams::hotspots[i].first ) {
		    p.dst_id = GlobalParams::hotspots[i].first;
		}
		break;
	    } else
		range_start += GlobalParams::hotspots[i].second;	// try next
	}
#ifdef DEADLOCK_AVOIDANCE
	assert((GlobalParams::topology == TOPOLOGY_MESH));
	if (p.dst_id%2!=0)
	{
	    p.dst_id = (p.dst_id+1)%256;
	}
#endif

    } while (p.dst_id == p.src_id);

    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();
    p.vc_id = randInt(0, GlobalParams::n_virtual_channels - 1);

    return p;
}
// TODO: for testing only
Packet ProcessingElement::trafficTest()
{
    Packet p;
    p.src_id = local_id;
    p.dst_id = 10;

    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();
    p.vc_id = randInt(0, GlobalParams::n_virtual_channels - 1);

    return p;
}

Packet ProcessingElement::trafficTranspose1()
{
    assert(GlobalParams::topology == TOPOLOGY_MESH);
    Packet p;
    p.src_id = local_id;
    Coord src, dst;

    // Transpose 1 destination distribution
    src.x = id2Coord(p.src_id).x;
    src.y = id2Coord(p.src_id).y;
    dst.x = GlobalParams::mesh_dim_x - 1 - src.y;
    dst.y = GlobalParams::mesh_dim_y - 1 - src.x;
    fixRanges(src, dst);
    p.dst_id = coord2Id(dst);

    p.vc_id = randInt(0, GlobalParams::n_virtual_channels - 1);
    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();

    return p;
}

Packet ProcessingElement::trafficTranspose2()
{
    assert(GlobalParams::topology == TOPOLOGY_MESH);
    Packet p;
    p.src_id = local_id;
    Coord src, dst;

    // Transpose 2 destination distribution
    src.x = id2Coord(p.src_id).x;
    src.y = id2Coord(p.src_id).y;
    dst.x = src.y;
    dst.y = src.x;
    fixRanges(src, dst);
    p.dst_id = coord2Id(dst);

    p.vc_id = randInt(0, GlobalParams::n_virtual_channels - 1);
    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();

    return p;
}

void ProcessingElement::setBit(int &x, int w, int v)
{
    int mask = 1 << w;

    if (v == 1)
	x = x | mask;
    else if (v == 0)
	x = x & ~mask;
    else
	assert(false);
}

int ProcessingElement::getBit(int x, int w)
{
    return (x >> w) & 1;
}

inline double ProcessingElement::log2ceil(double x)
{
    return ceil(log(x) / log(2.0));
}

Packet ProcessingElement::trafficBitReversal()
{

    int nbits =
	(int)
	log2ceil((double)
		 (GlobalParams::mesh_dim_x *
		  GlobalParams::mesh_dim_y));
    int dnode = 0;
    for (int i = 0; i < nbits; i++)
	setBit(dnode, i, getBit(local_id, nbits - i - 1));

    Packet p;
    p.src_id = local_id;
    p.dst_id = dnode;

    p.vc_id = randInt(0, GlobalParams::n_virtual_channels - 1);
    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();

    return p;
}

Packet ProcessingElement::trafficShuffle()
{

    int nbits =
	(int)
	log2ceil((double)
		 (GlobalParams::mesh_dim_x *
		  GlobalParams::mesh_dim_y));
    int dnode = 0;
    for (int i = 0; i < nbits - 1; i++)
	setBit(dnode, i + 1, getBit(local_id, i));
    setBit(dnode, 0, getBit(local_id, nbits - 1));

    Packet p;
    p.src_id = local_id;
    p.dst_id = dnode;

    p.vc_id = randInt(0, GlobalParams::n_virtual_channels - 1);
    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();

    return p;
}

Packet ProcessingElement::trafficButterfly()
{

    int nbits = (int) log2ceil((double)
		 (GlobalParams::mesh_dim_x *
		  GlobalParams::mesh_dim_y));
    int dnode = 0;
    for (int i = 1; i < nbits - 1; i++)
	setBit(dnode, i, getBit(local_id, i));
    setBit(dnode, 0, getBit(local_id, nbits - 1));
    setBit(dnode, nbits - 1, getBit(local_id, 0));

    Packet p;
    p.src_id = local_id;
    p.dst_id = dnode;

    p.vc_id = randInt(0, GlobalParams::n_virtual_channels - 1);
    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();

    return p;
}

void ProcessingElement::fixRanges(const Coord src,
				       Coord & dst)
{
    // Fix ranges
    if (dst.x < 0)
	dst.x = 0;
    if (dst.y < 0)
	dst.y = 0;
    if (dst.x >= GlobalParams::mesh_dim_x)
	dst.x = GlobalParams::mesh_dim_x - 1;
    if (dst.y >= GlobalParams::mesh_dim_y)
	dst.y = GlobalParams::mesh_dim_y - 1;
}

int ProcessingElement::getRandomSize()
{
    return randInt(GlobalParams::min_packet_size,
		   GlobalParams::max_packet_size);
}

unsigned int ProcessingElement::getQueueSize() const
{
    return packet_queue.size();
}

bool ProcessingElement::isMemoryTile(int id)
{
    // Memory tile IDs for a 5x5 mesh (25 total tiles)
    // ID layout in 5x5 mesh (row-major: id = x + y * mesh_dim_x):
    //  0  1  2  3  4
    //  5  6  7  8  9
    // 10 11 12 13 14
    // 15 16 17 18 19
    // 20 21 22 23 24
    //
    // Memory tiles: Top and bottom rows (10 total)
    // Top row: 0, 1, 2, 3, 4
    // Bottom row: 20, 21, 22, 23, 24
    // Compute PEs (15 total): 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
    
    static const int MEMORY_TILE_IDS[] = {
       0,1,2,3,4,20,21,22,23,24
    };

    static const int NUM_MEMORY_TILES = 10;
    
    for (int i = 0; i < NUM_MEMORY_TILES; i++) {
        if (id == MEMORY_TILE_IDS[i])
            return true;
    }
    return false;
}

int ProcessingElement::getTracePeId(int noxim_id)
{
    // Map Noxim mesh tile ID to trace PE ID
    // For compute PEs, they map directly to their trace file number
    // Memory tiles (4, 7, 8, 11) should not call this function
    //
    // Noxim ID -> Trace PE ID mapping:
    //  0 -> 0    1 -> 1    2 -> 2    3 -> 3
    //  4 -> MEM  5 -> 5    6 -> 6    7 -> MEM
    //  8 -> MEM  9 -> 9   10 -> 10  11 -> MEM
    // 12 -> 12  13 -> 13  14 -> 14  15 -> 15
    
    // Since compute PEs use the same ID in both Noxim and traces,
    // we can just return the noxim_id directly for compute PEs
    return noxim_id;
}

void ProcessingElement::loadTraceFile()
{
    // Only load trace file if TRAFFIC_TRACE_BASED mode is enabled
    if (GlobalParams::traffic_distribution != TRAFFIC_TRACE_BASED)
        return;

    // Check if trace_dir is set
    if (GlobalParams::trace_dir.empty()) {
        cerr << "Warning: TRAFFIC_TRACE_BASED mode enabled but trace_dir not specified" << endl;
        return;
    }

    // Only load traces for actual mesh tiles, not hubs or other special PEs
    // In a mesh topology, valid tile IDs are 0 to (mesh_dim_x * mesh_dim_y - 1)
    int max_tile_id = GlobalParams::mesh_dim_x * GlobalParams::mesh_dim_y - 1;
    if (local_id > max_tile_id) {
        // This is likely a hub or wireless component PE - skip trace loading
        return;
    }

    // Memory tiles do not inject traffic - they only receive and respond
    if (isMemoryTile(local_id)) {
        cout << "Tile " << local_id << " is a memory controller - no trace injection" << endl;
        return;
    }
    
    // Avoid reloading if already loaded
    if (!trace_events.empty()) {
        return;
    }

    // Get the trace PE ID for this compute PE
    int trace_pe_id = getTracePeId(local_id);
    
    // Build trace filename: trace_dir/pe_<trace_pe_id>.trace
    string trace_filename = GlobalParams::trace_dir + "/pe_" + to_string(trace_pe_id) + ".trace";

    ifstream trace_file(trace_filename);
    if (!trace_file.is_open()) {
        cout << "Warning: Compute PE " << local_id << " (trace_id=" << trace_pe_id << ") - trace file not found: " << trace_filename << endl;
        cout << "         This PE will not inject any packets." << endl;
        return;
    }

    cout << "Compute PE " << local_id << " (trace_id=" << trace_pe_id << ") loading trace from: " << trace_filename << endl;

    string line;
    int line_num = 0;
    while (getline(trace_file, line)) {
        line_num++;
        
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#' || line[0] == '%')
            continue;

        // Parse trace line: cycle src dst feature_id [coalesce_hint]
        istringstream iss(line);
        TraceEvent event;
        
        if (iss >> event.cycle >> event.src >> event.dst >> event.feature_id) {
            // Sanity check: src should match local_id
            if (event.src != local_id) {
                cerr << "Error: PE " << local_id << " trace file line " << line_num 
                     << " has mismatched src=" << event.src << endl;
                continue;
            }
            
            // Try to read optional coalesce_hint (0 or 1)
            int hint_val = 0;
            if (iss >> hint_val) {
                event.coalesce_hint = (hint_val == 1);
            } else {
                event.coalesce_hint = false;  // Default: no coalescing
            }
            
            trace_events.push_back(event);
        } else {
            cerr << "Warning: PE " << local_id << " - malformed line " << line_num 
                 << " in trace file" << endl;
        }
    }

    trace_file.close();
    
    cout << "Compute PE " << local_id << " (trace_id=" << trace_pe_id << ") loaded " << trace_events.size() << " trace events" << endl;
}

bool ProcessingElement::allTraceEventsSent() const
{
    // For memory tiles, always return true (they don't inject trace traffic)
    if (is_memory_tile)
        return true;
    
    // For compute PEs, check if all trace events have been sent
    return next_event_idx >= trace_events.size();
}

uint64_t ProcessingElement::getInFlightRequests() const
{
    // For memory tiles, no in-flight requests to track
    if (is_memory_tile)
        return 0;
    
    // Calculate in-flight = requests sent - responses received
    // total_requests_injected counts REQUEST packets sent
    // total_responses_received counts RESPONSE packets received
    if (total_requests_injected >= total_responses_received)
        return total_requests_injected - total_responses_received;
    else
        return 0;  // Shouldn't happen, but protect against underflow
}

bool ProcessingElement::canShotTrace(Packet & packet)
{
    // Memory tiles do not inject trace-based traffic
    if (isMemoryTile(local_id))
        return false;

    // If we've exhausted all trace events, no more injections
    if (next_event_idx >= trace_events.size()) {
        static bool warned = false;
        if (!warned && GlobalParams::verbose_mode >= VERBOSE_LOW) {
            cerr << "PE[" << local_id << "] exhausted all trace events" << endl;
            warned = true;
        }
        return false;
    }

    // Get current cycle (absolute simulation time)
    uint64_t cur_cycle = static_cast<uint64_t>(
        sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
    
    // Adjust for reset/warmup time - trace cycles are relative to actual simulation start
    // Subtract reset_time to get the cycle relative to simulation start
    uint64_t sim_cycle = (cur_cycle > GlobalParams::reset_time) ? 
                         (cur_cycle - GlobalParams::reset_time) : 0;

    // Get the next event
    const TraceEvent& event = trace_events[next_event_idx];

    // Wait until the trace event's scheduled cycle
    if (sim_cycle < event.cycle) {
        static int debug_count = 0;
        if (debug_count++ < 5 && GlobalParams::verbose_mode >= VERBOSE_LOW) {
            cerr << "PE[" << local_id << "] waiting for event " << next_event_idx 
                 << ": sim_cycle=" << sim_cycle << " < event.cycle=" << event.cycle << endl;
        }
        return false;  // Not time yet - wait
    }

    // CRITICAL: Preserve inter-event spacing to avoid "catch-up" bursts after stalls
    // If we stalled (no credits), we maintain the original spacing between requests
    // This models realistic compute behavior where dependencies are preserved
    if (next_event_idx > 0 && last_injection_cycle > 0) {
        // Calculate minimum spacing from trace (typically 10 cycles for cache misses)
        // Use actual spacing from previous event
        uint64_t prev_event_cycle = trace_events[next_event_idx - 1].cycle;
        uint64_t trace_spacing = event.cycle - prev_event_cycle;
        
        // Enforce same spacing in actual injections
        uint64_t cycles_since_last = sim_cycle - last_injection_cycle;
        if (cycles_since_last < trace_spacing) {
            return false;  // Too soon after last injection - preserve spacing
        }
    }

    // Track injection attempts and stall reasons
    total_injection_attempts++;

    // Check if we have credit for the destination memory tile
    if (!hasCredit(event.dst)) {
        static int debug_count = 0;
        if (debug_count++ < 5 && GlobalParams::verbose_mode >= VERBOSE_LOW) {
            cerr << "PE[" << local_id << "] NO CREDIT for dst=" << event.dst 
                 << " (have " << total_memory_credits << " credits)" << endl;
        }
        stall_cycles_no_credits++;  // STALL REASON: Memory system backpressure (no credits)
        return false;  // No credit available - wait (backpressure from memory system)
    }
    
    // Create the REQUEST packet
    double now = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    
    // NO VC PARTITIONING: All traffic uses ALL VCs (0 to n_virtual_channels-1)
    // REQUESTs, RESPONSEs, unicast, and multicast all share the full VC range
    // Multicast engine coordinates with reservation table via mc_vc_busy tracking
    const int REQUEST_VC_START = 0;
    const int REQUEST_VC_END = GlobalParams::n_virtual_channels - 1;
    int vc = randInt(REQUEST_VC_START, REQUEST_VC_END);
    
    // REQUEST packets are 2 flits (2 flits × 128 bits/flit = 256 bits = 32 bytes)
    const int REQUEST_SIZE_FLITS = 1;
    
    packet.make(local_id, event.dst, vc, now, REQUEST_SIZE_FLITS);
    packet.feature_id = event.feature_id;
    packet.packet_type = PACKET_TYPE_REQUEST;
    packet.coalesce_hint = event.coalesce_hint;  // Propagate hint from trace
    
    // Consume one credit for this memory tile
    consumeCredit(event.dst);
    
    // Record injection time to preserve inter-event spacing
    last_injection_cycle = sim_cycle;

    // Track REQUEST injection time for end-to-end latency measurement
    request_injection_time[event.feature_id] = cur_cycle;
    
    // Track outstanding request for debugging using unique sequence number
    uint64_t this_request_seq = next_request_seq++;
    OutstandingRequest req;
    req.injection_cycle = cur_cycle;
    req.dst_mem_tile = event.dst;
    req.feature_id = event.feature_id;
    outstanding_requests[this_request_seq] = req;

    // Advance to next event (will be done after successful transmission)
    // Note: We don't increment here - let txProcess do it after pushing to queue
    
    return true;
}

// Handle incoming REQUEST packet at memory tile
// NOTE: Should only be called when canAcceptRequest() returned true (checked in rxProcess)
void ProcessingElement::handleIncomingRequest(const Flit& flit)
{
    if (!memory_controller) {
        cerr << "ERROR: handleIncomingRequest called but no memory controller!" << endl;
        return;
    }

    // Get current cycle when HEAD flit arrives
    uint64_t arrival_cycle = static_cast<uint64_t>(
        sc_time_stamp().to_double() / GlobalParams::clock_period_ps);

    // Mark the first request for utilization tracking
    memory_controller->markFirstRequest(arrival_cycle);

    // Process the request through DRAM model
    // Pass the recorded path for XY_PATH_REVERSE mode, coalesce hint, coalesce_allowed (per-request), multicast destinations, and multicast_root_id
    // Should always succeed since we checked canAcceptRequest() before calling this
    bool accepted = memory_controller->processRequest(flit.src_id, flit.feature_id, arrival_cycle, flit.recorded_path, flit.coalesce_hint, flit.coalesce_allowed, flit.multicast_dests, flit.multicast_root_id);
    
    if (!accepted) {
        cerr << "ERROR: MemCtrl rejected request even though canAcceptRequest() returned true!" << endl;
    }
}

// Generate RESPONSE packets for memory tiles
bool ProcessingElement::canShotResponse(Packet & packet)
{
    if (!memory_controller) {
        return false;  // Should not happen
    }

    // Get current cycle
    uint64_t current_cycle = static_cast<uint64_t>(
        sc_time_stamp().to_double() / GlobalParams::clock_period_ps);

    // Check if there's a response ready to send
    if (!memory_controller->hasReadyResponse(current_cycle)) {
        return false;
    }

    // Get the response details
    MemoryRequest resp = memory_controller->getNextResponse(current_cycle);

    // Create RESPONSE packet (4 flits × 256 bits = 128 bytes)
    const int RESPONSE_SIZE_FLITS = 4;  // Updated from 3 to 4
    double now = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    
    // NO VC PARTITIONING: All traffic uses ALL VCs (0 to n_virtual_channels-1)
    // Memory tiles use full VC range for RESPONSE transmission
    // Multicast engine preserves incoming VC, reservation table checks mc_vc_busy
    int vc = randInt(0, GlobalParams::n_virtual_channels - 1);
    
    packet.make(local_id, resp.original_src_id, vc, now, RESPONSE_SIZE_FLITS);
    packet.feature_id = resp.feature_id;
    packet.packet_type = PACKET_TYPE_RESPONSE;
    packet.recorded_path = resp.recorded_path;  // Attach recorded path for reverse routing
    packet.coalesce_hint = resp.coalesce_hint;  // Propagate coalesce hint from original request
    packet.multicast_allowed = resp.coalesce_allowed;  // CRITICAL: Copy per-request coalesce decision to response
    packet.multicast_root_id = resp.multicast_root_id;  // Propagate multicast root router ID (for future use)
    
    // NOTE: multicast_dests left EMPTY - routers handle multicasting via coalesce_table

    if (GlobalParams::verbose_mode > VERBOSE_OFF) {
        cout << "MemTile[" << local_id << "] @ cycle " << current_cycle
             << ": Injecting RESPONSE to PE " << resp.original_src_id 
             << " (fid=" << resp.feature_id << ", scheduled @ " << resp.start_cycle << ")" << endl;
    }

    return true;
}

// Print memory controller statistics
void ProcessingElement::printMemoryStats() const
{
    if (!is_memory_tile || !memory_controller) {
        return;  // Not a memory tile
    }
    
    // Update total cycles for utilization calculation
    uint64_t current_cycle = static_cast<uint64_t>(
        sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
    memory_controller->updateTotalCycles(current_cycle);
    
    cout << "MemTile[" << local_id << "] Statistics:" << endl;
    cout << "  Requests received: " << memory_controller->getTotalRequestsReceived() << endl;
    cout << "  Responses sent:    " << memory_controller->getTotalResponsesSent() << endl;
    cout << "  Requests dropped:  " << memory_controller->getTotalRequestsDropped() << endl;
    cout << "  Queue size:        " << memory_controller->getPendingCount() << endl;
    
    // Link utilization statistics
    cout << "  Link Utilization:" << endl;
    cout << "    Ingress (REQs):  " << fixed << setprecision(1) 
         << memory_controller->getIngressUtilization() << "% "
         << "(" << memory_controller->getIngressBusyCycles() << "/" 
         << memory_controller->getTotalCyclesObserved() << " cycles)" << endl;
    cout << "    Egress (RESPs):  " << fixed << setprecision(1) 
         << memory_controller->getEgressUtilization() << "% "
         << "(" << memory_controller->getEgressBusyCycles() << "/" 
         << memory_controller->getTotalCyclesObserved() << " cycles)" << endl;
}

void ProcessingElement::printE2ELatencyStats() const
{
    if (is_memory_tile) {
        return;  // Only compute PEs track end-to-end latency
    }
    
    if (e2e_latency_samples == 0) {
        cout << "PE[" << local_id << "] End-to-End Latency: No completed REQUEST-RESPONSE pairs" << endl;
        return;
    }
    
    double avg_latency = static_cast<double>(total_e2e_latency) / e2e_latency_samples;
    double avg_pe_queue = static_cast<double>(total_pe_queue_delay) / e2e_latency_samples;
    double avg_network_dram = avg_latency - avg_pe_queue;
    
    cout << "PE[" << local_id << "] End-to-End Latency Statistics:" << endl;
    cout << "  Samples:            " << e2e_latency_samples << endl;
    cout << "  Total Avg (cyc):    " << avg_latency << endl;
    cout << "  Total Max (cyc):    " << max_e2e_latency << endl;
    cout << "  PE Queue Avg (cyc): " << avg_pe_queue << endl;
    cout << "  PE Queue Max (cyc): " << max_pe_queue_delay << endl;
    cout << "  Net+DRAM Avg (cyc): " << avg_network_dram << endl;
    cout << "  Total Avg (ns):     " << (avg_latency * GlobalParams::clock_period_ps / 1000.0) << endl;
    cout << "  Total Max (ns):     " << (max_e2e_latency * GlobalParams::clock_period_ps / 1000.0) << endl;
}

void ProcessingElement::printStallStats() const
{
    if (is_memory_tile) {
        return;  // Only compute PEs track stall statistics
    }
    
    if (total_injection_attempts == 0) {
        cout << "PE[" << local_id << "] Stall Statistics: No injection attempts" << endl;
        return;
    }
    
    // Calculate percentages
    double pct_no_credits = 100.0 * stall_cycles_no_credits / total_injection_attempts;
    double pct_noc_contention = 100.0 * stall_cycles_noc_contention / total_injection_attempts;
    double pct_successful = 100.0 - pct_no_credits;  // Attempts that had credits (may still stall on NoC)
    
    cout << "PE[" << local_id << "] Stall Statistics:" << endl;
    cout << "  Total injection attempts:  " << total_injection_attempts << endl;
    cout << "  Stalled (no credits):      " << stall_cycles_no_credits 
         << " (" << fixed << setprecision(1) << pct_no_credits << "%)" << endl;
    cout << "  Had credits:               " << (total_injection_attempts - stall_cycles_no_credits)
         << " (" << fixed << setprecision(1) << pct_successful << "%)" << endl;
    cout << "  NoC contention stalls:     " << stall_cycles_noc_contention << " cycles" << endl;
    cout << "  Requests injected:         " << total_requests_injected << endl;
    cout << endl;
    cout << "  Interpretation:" << endl;
    cout << "    - No credits = Memory system backpressure (responses not returning)" << endl;
    cout << "    - NoC contention = Router buffers full (network congestion)" << endl;
}

// Initialize credits for memory requests
void ProcessingElement::initMemoryCredits()
{
    // Each PE gets credits to prevent deadlock
    // Calculation for deadlock-free operation with shared VCs:
    // - Total MSHR capacity: 8 memory tiles × 64 = 512
    // - Buffer capacity constraint: (buffer_depth × n_VCs) ≥ (MSHR × response_size + credits × request_size)
    //   64 × 8 ≥ 64 × 4 + 17 × credits × 1
    //   512 ≥ 256 + 17 × credits
    //   credits ≤ 256/17 = 15.05 → 15 credits per PE
    const int TOTAL_CREDITS_PER_PE = 32;  // Total credits per PE for all memory tiles
    
    // Only initialize once per PE (track by PE id)
    static std::set<int> initialized_pes;
    if (initialized_pes.count(local_id) > 0) return;
    initialized_pes.insert(local_id);
    
    total_memory_credits = TOTAL_CREDITS_PER_PE;
    total_responses_received = 0;  // Initialize response counter
    
    if (GlobalParams::verbose_mode >= VERBOSE_LOW) {
        cout << "PE[" << local_id << "] initialized with " << TOTAL_CREDITS_PER_PE 
             << " total credits (any destination)" << endl;
    }
}

// Check if credits are available (for any memory tile)
bool ProcessingElement::hasCredit(int mem_tile_id)
{
    // Simple: just check if PE has any credits remaining
    return total_memory_credits > 0;
}

// Consume one credit when sending a REQUEST
void ProcessingElement::consumeCredit(int mem_tile_id)
{
    if (total_memory_credits > 0) {
        total_memory_credits--;
    } else {
        cerr << "ERROR: PE[" << local_id << "] tried to consume credit but none available!" << endl;
    }
}

// Return one credit when receiving a RESPONSE
void ProcessingElement::returnCredit(int src_mem_tile)
{
    const int MAX_CREDITS = 33;  // Match TOTAL_CREDITS_PER_PE initialization
    
    // Safety check: prevent credit overflow bug
    if (total_memory_credits >= MAX_CREDITS) {
        cerr << "WARNING: PE[" << local_id << "] credit overflow! "
             << "Already has " << total_memory_credits << "/" << MAX_CREDITS 
             << " credits. Ignoring return." << endl;
        return;  // Don't increment beyond max
    }
    
    total_memory_credits++;
    
    //cerr << "PE[" << local_id << "] returnCredit() - now have " << total_memory_credits << " credits" << endl;
}

// Print heartbeat statistics
void ProcessingElement::printHeartbeat(int id, uint64_t cycle)
{
    if (is_memory_tile && memory_controller != nullptr) {
        // Memory tile status
        cout << "  [MemTile " << id << "] RESPs sent: " << total_responses_injected
             << ", Queue: " << packet_queue.size() << "/16"
             << ", In-flight: " << memory_controller->getInFlightCount()
             << ", DRAM egress queue: " << memory_controller->getPendingCount() << endl;
    } else if (!is_memory_tile && !trace_events.empty()) {
        // Compute PE status (trace-based)
        int outstanding = total_requests_injected - total_responses_received;
        int sum = total_memory_credits + outstanding;
        cout << "  [PE " << id << "] REQs sent: " << total_requests_injected 
            << ", RESPs recv: " << total_responses_received
            << ", Trace: " << next_event_idx << "/" << trace_events.size()
            << ", Credits: " << total_memory_credits << "/32"
            << ", Outstanding: " << outstanding
            << ", Credits+Outstanding: " << sum
            << ", Queue: " << packet_queue.size() << endl;
        
        // Check for missing responses
        checkMissingResponses();
    }
}

void ProcessingElement::checkMissingResponses()
{
    uint64_t cur_cycle = static_cast<uint64_t>(
        sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
    
    // Check for requests outstanding longer than 1000 cycles (should be ~300 cycles max)
    // If response hasn't arrived, return the credit to prevent deadlock
    const uint64_t TIMEOUT_CYCLES = 750;
    
    std::vector<uint64_t> timed_out_seqs;
    
    for (const auto& entry : outstanding_requests) {
        uint64_t seq = entry.first;  // sequence number (unique per request)
        const OutstandingRequest& req = entry.second;
        uint64_t age = cur_cycle - req.injection_cycle;
        
        if (age > TIMEOUT_CYCLES) {
            //cerr << "*** PE[" << local_id << "] TIMEOUT: seq=" << seq
              //   << " feature_id=" << req.feature_id
                // << " dst=MemTile[" << req.dst_mem_tile << "]"
                 //<< " age=" << age << " cycles - RETURNING CREDIT" << endl;
            
            // Return the credit that was consumed when this request was sent
            returnCredit(req.dst_mem_tile);
            
            // Track timeout count for this feature
            timeout_counts_per_feature[req.feature_id]++;
            
            // Mark for removal
            timed_out_seqs.push_back(seq);
        }
    }
    
    // Remove timed-out requests from tracking map
    // NOTE: We do NOT remove from request_injection_time or request_network_entry_time
    // because those are indexed by feature_id and other requests with same feature_id may still be in-flight
    for (uint64_t seq : timed_out_seqs) {
        outstanding_requests.erase(seq);
    }
}

void ProcessingElement::printTimeoutStats() const
{
    if (is_memory_tile) {
        return;  // Only compute PEs track timeouts
    }
    
    if (timeout_counts_per_feature.empty()) {
        cout << "PE[" << local_id << "] Timeout Statistics: No timeouts occurred" << endl;
        return;
    }
    
    int total_timeouts = 0;
    for (const auto& entry : timeout_counts_per_feature) {
        total_timeouts += entry.second;
    }
    
    cout << "PE[" << local_id << "] Timeout Statistics (Credits Returned):" << endl;
    cout << "  Total timeouts: " << total_timeouts << endl;
    cout << "  Unique features: " << timeout_counts_per_feature.size() << endl;
    cout << "  Per-feature breakdown (showing features with >0 timeouts):" << endl;
    
    // Sort by feature_id for consistent output
    std::vector<std::pair<int, int>> sorted_timeouts(timeout_counts_per_feature.begin(), timeout_counts_per_feature.end());
    std::sort(sorted_timeouts.begin(), sorted_timeouts.end());
    
    int shown = 0;
    const int MAX_SHOW = 20;  // Show first 20 features
    for (const auto& entry : sorted_timeouts) {
        if (shown >= MAX_SHOW) {
            cout << "    ... (" << (sorted_timeouts.size() - MAX_SHOW) << " more features)" << endl;
            break;
        }
        cout << "    Feature " << entry.first << ": " << entry.second << " timeout(s)" << endl;
        shown++;
    }
}


