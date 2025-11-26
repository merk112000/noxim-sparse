/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the processing element
 */

#include "ProcessingElement.h"
#include <fstream>
#include <sstream>
#include <iostream>

int ProcessingElement::randInt(int min, int max)
{
    return min +
	(int) ((double) (max - min + 1) * rand() / (RAND_MAX + 1.0));
}

void ProcessingElement::rxProcess()
{
    if (reset.read()) {
	ack_rx.write(0);
	current_level_rx = 0;
	if (memory_controller) {
	    memory_controller->reset();
	}
    } else {
	if (req_rx.read() == 1 - current_level_rx) {
	    Flit flit_tmp = flit_rx.read();
	    current_level_rx = 1 - current_level_rx;	// Negate the old value for Alternating Bit Protocol (ABP)
	    
	    // If this is a memory tile and we received a REQUEST HEAD flit
	    if (is_memory_tile && 
	        flit_tmp.packet_type == PACKET_TYPE_REQUEST && 
	        flit_tmp.flit_type == FLIT_TYPE_HEAD) {
	        handleIncomingRequest(flit_tmp);
	    }
	    
	    // If this is a compute PE and we received a RESPONSE HEAD flit, return credit
	    if (!is_memory_tile && 
	        flit_tmp.packet_type == PACKET_TYPE_RESPONSE && 
	        flit_tmp.flit_type == FLIT_TYPE_HEAD) {
	        returnCredit(flit_tmp.src_id);
	    }
	}
	ack_rx.write(current_level_rx);
    }
}

void ProcessingElement::txProcess()
{
    if (reset.read()) {
	req_tx.write(0);
	current_level_tx = 0;
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

    // Memory tiles generate RESPONSE packets
    if (is_memory_tile) {
        // No NI queue limit for memory tiles - allows unbounded growth for deadlock-free operation
        Packet packet;
        if (canShotResponse(packet)) {
            packet_queue.push(packet);
            transmittedAtPreviousCycle = true;
        } else {
            transmittedAtPreviousCycle = false;
        }
    }
    // Compute PEs generate REQUEST packets (trace-based or other traffic)
    else if(GlobalParams::traffic_distribution != TRAFFIC_HARDCODED) {
        Packet packet;
        if (canShot(packet)) {
            packet_queue.push(packet);
            transmittedAtPreviousCycle = true;
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
				int vc = randInt(0,GlobalParams::n_virtual_channels-1);
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


	if (ack_tx.read() == current_level_tx) {
	    if (!packet_queue.empty()) {
		Flit flit = nextFlit();	// Generate a new flit
		flit_tx->write(flit);	// Send the generated flit
		current_level_tx = 1 - current_level_tx;	// Negate the old value for Alternating Bit Protocol (ABP)
		req_tx.write(current_level_tx);
	    }
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

    flit.hub_relay_node = NOT_VALID;

    if (packet.size == packet.flit_left)
	flit.flit_type = FLIT_TYPE_HEAD;
    else if (packet.flit_left == 1)
	flit.flit_type = FLIT_TYPE_TAIL;
    else
	flit.flit_type = FLIT_TYPE_BODY;

    packet_queue.front().flit_left--;
    if (packet_queue.front().flit_left == 0)
	packet_queue.pop();

    return flit;
}

bool ProcessingElement::canShot(Packet & packet)
{
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
                    int vc = randInt(0,GlobalParams::n_virtual_channels-1);
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
    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
    
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
    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);

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
    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);

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
    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);

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

    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
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

    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
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

    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
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

    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
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

    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
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
    // Memory tile IDs for a 4x4 mesh (16 total tiles)
    // ID layout in 4x4 mesh (row-major: id = x + y * mesh_dim_x):
    //  0  1  2  3
    //  4  5  6  7
    //  8  9 10 11
    // 12 13 14 15
    //
    // Memory tiles (8 total - 2 intermediate tiles on each side):
    // Left side: 4, 8    Right side: 7, 11
    // Top side: 1, 2     Bottom side: 13, 14
    // Compute PEs (8 total): 0, 3, 5, 6, 9, 10, 12, 15
    
    static const int MEMORY_TILE_IDS[] = {1, 2, 4, 7, 8, 11, 13, 14};
    static const int NUM_MEMORY_TILES = 8;
    
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

        // Parse trace line: cycle src dst feature_id
        istringstream iss(line);
        TraceEvent event;
        
        if (iss >> event.cycle >> event.src >> event.dst >> event.feature_id) {
            // Sanity check: src should match local_id
            if (event.src != local_id) {
                cerr << "Error: PE " << local_id << " trace file line " << line_num 
                     << " has mismatched src=" << event.src << endl;
                continue;
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

bool ProcessingElement::canShotTrace(Packet & packet)
{
    // Memory tiles do not inject trace-based traffic
    if (isMemoryTile(local_id))
        return false;

    // If we've exhausted all trace events, no more injections
    if (next_event_idx >= trace_events.size())
        return false;

    // Get current cycle (absolute simulation time)
    uint64_t cur_cycle = static_cast<uint64_t>(
        sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
    
    // Adjust for reset/warmup time - trace cycles are relative to actual simulation start
    // Subtract reset_time to get the cycle relative to simulation start
    uint64_t sim_cycle = (cur_cycle > GlobalParams::reset_time) ? 
                         (cur_cycle - GlobalParams::reset_time) : 0;

    // Get the next event
    const TraceEvent& event = trace_events[next_event_idx];

    // Inject when simulation cycle reaches or passes the trace event cycle
    // This handles cases where NI queue was full and we missed exact cycle
    if (sim_cycle < event.cycle) {
        return false;  // Not time yet - wait
    }

    // Time to inject (at or past the scheduled cycle)
    // Check if we have credit for the destination memory tile
    if (!hasCredit(event.dst)) {
        return false;  // No credit available - wait
    }
    
    // Create the REQUEST packet
    double now = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    int vc = randInt(0, GlobalParams::n_virtual_channels - 1);
    
    // REQUEST packets are 2 flits (2 flits × 128 bits/flit = 256 bits = 32 bytes)
    const int REQUEST_SIZE_FLITS = 2;
    packet.make(local_id, event.dst, vc, now, REQUEST_SIZE_FLITS);
    packet.feature_id = event.feature_id;
    packet.packet_type = PACKET_TYPE_REQUEST;
    
    // Consume one credit for this memory tile
    consumeCredit(event.dst);

    // Advance to next event (will be done after successful transmission)
    // Note: We don't increment here - let txProcess do it after pushing to queue
    
    return true;
}

// Handle incoming REQUEST packet at memory tile
void ProcessingElement::handleIncomingRequest(const Flit& flit)
{
    if (!memory_controller) {
        cerr << "ERROR: handleIncomingRequest called but no memory controller!" << endl;
        return;
    }

    // Get current cycle when HEAD flit arrives
    uint64_t arrival_cycle = static_cast<uint64_t>(
        sc_time_stamp().to_double() / GlobalParams::clock_period_ps);

    // Process the request through DRAM model
    bool accepted = memory_controller->processRequest(flit.src_id, flit.feature_id, arrival_cycle);
    
    // CRITICAL: Even if dropped, we must eventually return credit to prevent deadlock!
    // The memory controller will track this and still generate a response
    // (even if it's delayed or represents a retry)
    if (!accepted && GlobalParams::verbose_mode > VERBOSE_LOW) {
        cout << "WARNING: MemTile[" << local_id << "] REQUEST from PE " 
             << flit.src_id << " @ cycle " << arrival_cycle 
             << " - memory controller queue pressure" << endl;
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
    MemoryRequest resp = memory_controller->getNextResponse();

    // Create RESPONSE packet (8 flits = 128 bytes)
    const int RESPONSE_SIZE_FLITS = 8;
    double now = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    int vc = randInt(0, GlobalParams::n_virtual_channels - 1);
    
    packet.make(local_id, resp.original_src_id, vc, now, RESPONSE_SIZE_FLITS);
    packet.feature_id = resp.feature_id;
    packet.packet_type = PACKET_TYPE_RESPONSE;

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
    
    cout << "MemTile[" << local_id << "] Statistics:" << endl;
    cout << "  Requests received: " << memory_controller->getTotalRequestsReceived() << endl;
    cout << "  Responses sent:    " << memory_controller->getTotalResponsesSent() << endl;
    cout << "  Requests dropped:  " << memory_controller->getTotalRequestsDropped() << endl;
    cout << "  Queue size:        " << memory_controller->getPendingCount() << endl;
}

// Initialize credits for all memory tiles
void ProcessingElement::initMemoryCredits()
{
    // Credits control max outstanding requests per memory tile
    const int CREDITS_PER_MEMORY_TILE = 64;
    
    // Get all memory tile IDs
    static const int MEMORY_TILE_IDS[] = {1, 2, 4, 7, 8, 11, 13, 14};
    static const int NUM_MEMORY_TILES = 8;
    
    for (int i = 0; i < NUM_MEMORY_TILES; i++) {
        memory_credits[MEMORY_TILE_IDS[i]] = CREDITS_PER_MEMORY_TILE;
    }
}

// Check if credits are available for a memory tile
bool ProcessingElement::hasCredit(int mem_tile_id)
{
    if (memory_credits.find(mem_tile_id) == memory_credits.end()) {
        return false;  // Not a memory tile
    }
    return memory_credits[mem_tile_id] > 0;
}

// Consume one credit when sending a REQUEST
void ProcessingElement::consumeCredit(int mem_tile_id)
{
    if (memory_credits.find(mem_tile_id) != memory_credits.end()) {
        memory_credits[mem_tile_id]--;
    }
}

// Return one credit when receiving a RESPONSE
void ProcessingElement::returnCredit(int src_mem_tile)
{
    if (memory_credits.find(src_mem_tile) != memory_credits.end()) {
        memory_credits[src_mem_tile]++;
    }
}

