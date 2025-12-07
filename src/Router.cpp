/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the router
 */

#include "Router.h"

// Helper function to create key for oracle tracking
// Note: For a given feature_id, dst_id is always the same, so we only key on feature_id
namespace {
    inline int makeOracleKey(int feature_id, int dst_id) {
        return feature_id;  // dst_id not needed - each feature maps to one destination
    }
}

inline int toggleKthBit(int n, int k)
{
	return (n ^ (1 << (k-1)));
}

void Router::process()
{
    rxProcess();   // RX first - receive flits and update buffer status
    txProcess();   // TX second - forward flits based on updated state
}

void Router::rxProcess()
{
    if (reset.read()) {
	TBufferFullStatus bfs;
	// Clear outputs and indexes of receiving protocol
	for (int i = 0; i < DIRECTIONS + 2; i++) {
	    ack_rx[i].write(true);  // READY/VALID: signal READY on reset
	    current_level_rx[i] = 0;  // kept for compatibility only, not used in protocol
	    buffer_full_status_rx[i].write(bfs);
	}
	routed_flits = 0;
	local_drained = 0;
    } 
    else 
    { 
	// READY/VALID protocol: req_rx is VALID, ack_rx is READY
	// A flit is transferred when VALID=1 and READY=1 in the same cycle
	for (int i = 0; i < DIRECTIONS + 2; i++) {
	    bool valid = req_rx[i].read();  // upstream VALID signal
	    bool ready = true;  // assume we're READY unless buffer is full
	    
	    if (valid) {
		// CRITICAL: Must check VC BEFORE reading flit data!
		// In proper READY/VALID protocol, we peek at control signals first
		// Since we can't peek VC without reading, we read but only accept if buffer has space
		Flit received_flit = flit_rx[i].read();
		int vc = received_flit.vc_id;

		if (!buffer[i][vc].IsFull()) 
		{
		    // Buffer has space - Accept the flit: VALID=1 and READY=1, so transfer occurs
		    ready = true;  // Signal READY
		    port_rx_busy[i]++;
		    buffer[i][vc].Push(received_flit);
		    LOG << " Flit " << received_flit << " collected from Input[" << i << "][" << vc <<"]" << endl;
		    power.bufferRouterPush();

		    if (received_flit.src_id == local_id)
			power.networkInterface();
		    
		    // Track oracle coalescing opportunities (stats only)
		    trackOracleCoalescing(received_flit);
		    
		    // CRITICAL: Update buffer status immediately after Push
		    // to reflect that this VC might now be full
		    TBufferFullStatus bfs_push;
		    for (int v=0; v<GlobalParams::n_virtual_channels; v++)
			bfs_push.mask[v] = buffer[i][v].IsFull();
		    buffer_full_status_rx[i].write(bfs_push);
		}
		else  // buffer full
		{
		    // Buffer full - Signal NOT READY
		    // CRITICAL: In SystemC, we already read the flit, but we don't store it
		    // The upstream should keep driving the SAME flit until handshake succeeds
		    // This is a SystemC limitation - ideally we'd peek VC before reading
		    ready = false;
		    LOG << " Flit " << received_flit << " buffer full Input[" << i << "][" << vc <<"], backpressure!" << endl;
		}
	    } else {
		// No VALID signal - always READY to accept
		ready = true;
	    }
	    
	    // Always write READY status to upstream
	    ack_rx[i].write(ready);
	    
	    // Update buffer full status for this port (per-VC granularity)
	    TBufferFullStatus bfs;
	    for (int vc=0;vc<GlobalParams::n_virtual_channels;vc++)
		bfs.mask[vc] = buffer[i][vc].IsFull();
	    buffer_full_status_rx[i].write(bfs);
	}
    }
}

void Router::txProcess()
{

  if (reset.read()) 
    {
      // Clear outputs and output registers
      for (int i = 0; i < DIRECTIONS + 2; i++) 
	{
	  req_tx[i].write(false);    // No VALID on reset
	  has_flit[i] = false;       // Output registers empty
	  current_level_tx[i] = 0;   // Legacy, not used
	}
    } 
  else 
    {
      // CRITICAL: Update buffer full status at START of txProcess to ensure
      // downstream sees current buffer state before making injection decisions
      // This mitigates SystemC non-deterministic process ordering
      for (int i = 0; i < DIRECTIONS + 2; i++) {
	  TBufferFullStatus bfs;
	  for (int vc = 0; vc < GlobalParams::n_virtual_channels; vc++) {
	      bfs.mask[vc] = buffer[i][vc].IsFull();
	  }
	  buffer_full_status_rx[i].write(bfs);
      }
      
 
      // ========================================================================
      // PHASE 1: Drive outputs from registers and handle handshake completion
      // ========================================================================
      for (int o = 0; o < DIRECTIONS + 2; o++) {
	  if (has_flit[o]) {
	      // We have a flit in the output register - drive VALID and data
	      flit_tx[o].write(out_reg[o]);
	      req_tx[o].write(true);  // Assert VALID
	      
	      // Check if handshake completes this cycle (VALID=1 && READY=1)
	      bool ready = ack_tx[o].read();
	      if (ready) {
		  // Handshake completed! Clear the output register
		  has_flit[o] = false;
		  
		  // Update stats for successful forwarding
		  Flit& flit = out_reg[o];
		  
		  /* Power & Stats ------------------------------------------------- */
		  if (o == DIRECTION_HUB) power.r2hLink();
		  else power.r2rLink();

		  power.bufferRouterPop();
		  power.crossBar();

		  if (o == DIRECTION_LOCAL) 
		  {
		      power.networkInterface();
		      LOG << "Consumed flit " << flit << endl;
		      stats.receivedFlit(sc_time_stamp().to_double() / GlobalParams::clock_period_ps, flit);
		      if (GlobalParams::max_volume_to_be_drained) 
		      {
			  if (drained_volume >= GlobalParams::max_volume_to_be_drained)
			      sc_stop();
			  else 
			  {
			      drained_volume++;
			      local_drained++;
			  }
		      }
		  }
		  /* End Power & Stats ------------------------------------------------- */
	      }
	  } else {
	      // No flit in output register - deassert VALID
	      req_tx[o].write(false);
	  }
      }
      
      // ========================================================================
      // PHASE 2: Reservation (find routes for HEAD flits)
      // ========================================================================
      for (int j = 0; j < DIRECTIONS + 2; j++) 
	{
	  int i = (start_from_port + j) % (DIRECTIONS + 2);

	  for (int k = 0;k < GlobalParams::n_virtual_channels; k++)
	  {
	      int vc = (start_from_vc[i]+k)%(GlobalParams::n_virtual_channels);
	      
	      if (!buffer[i][vc].IsEmpty()) 
	      {
		  Flit flit = buffer[i][vc].Front();
		  power.bufferRouterFront();

		  if (flit.flit_type == FLIT_TYPE_HEAD || flit.flit_type == FLIT_TYPE_HEAD_TAIL) 
		    {
		      // prepare data for routing
		      RouteData route_data;
		      route_data.current_id = local_id;
		      route_data.src_id = flit.src_id;
		      route_data.dst_id = flit.dst_id;
		      route_data.dir_in = i;
		      route_data.vc_id = flit.vc_id;

		      int o = route(route_data);

		      // manage special case of target hub not directly connected to destination
		      if (o>=DIRECTION_HUB_RELAY)
			  {
		      	Flit f = buffer[i][vc].Pop();
		      	f.hub_relay_node = o-DIRECTION_HUB_RELAY;
		      	buffer[i][vc].Push(f);
		      	o = DIRECTION_HUB;
			  }

		      TReservation r;
		      r.input = i;
		      r.vc = vc;

		      LOG << " checking availability of Output[" << o << "] for Input[" << i << "][" << vc << "] flit " << flit << endl;

		      int rt_status = reservation_table.checkReservation(r,o);

		      if (rt_status == RT_AVAILABLE) 
		      {
			  LOG << " reserving direction " << o << " for flit " << flit << endl;
			  reservation_table.reserve(r, o);
		      }
		      else if (rt_status == RT_ALREADY_SAME)
		      {
			  LOG << " RT_ALREADY_SAME reserved direction " << o << " for flit " << flit << endl;
		      }
		      else if (rt_status == RT_OUTVC_BUSY)
		      {
			  LOG << " RT_OUTVC_BUSY reservation direction " << o << " for flit " << flit << endl;
		      }
		      else if (rt_status == RT_ALREADY_OTHER_OUT)
		      {
			  LOG  << "RT_ALREADY_OTHER_OUT: another output previously reserved for the same flit " << endl;
		      }
		      else assert(false);
		    }
		}
	  }
	    start_from_vc[i] = (start_from_vc[i]+1)%GlobalParams::n_virtual_channels;
	}

      start_from_port = (start_from_port + 1) % (DIRECTIONS + 2);

      // ========================================================================
      // PHASE 3: Load output registers from input buffers (after handshakes completed)
      // ========================================================================
      for (int i = 0; i < DIRECTIONS + 2; i++) 
      { 
	  vector<pair<int,int> > reservations = reservation_table.getReservations(i);
	  
	  if (reservations.size() != 0)
	  {
	      int rnd_idx = rand() % reservations.size();
	      
	      // Try ALL reservations starting from random index
	      for (size_t attempt = 0; attempt < reservations.size(); attempt++) {
		  int curr_idx = (rnd_idx + attempt) % reservations.size();
		  
		  int o = reservations[curr_idx].first;
		  int vc = reservations[curr_idx].second;
		  
		  // Skip if output register already occupied (can't load new flit yet)
		  if (has_flit[o]) {
		      continue;  // Output register busy, try next reservation
		  }
		  
		  // Check if input buffer has a flit
		  if (!buffer[i][vc].IsEmpty())  
		  {
		      Flit flit = buffer[i][vc].Front();
		      
		      // Check downstream buffer space (per-VC backpressure)
		      bool vc_has_space = !buffer_full_status_tx[o].read().mask[vc];
		      
		      if (vc_has_space) 
		      {
			  LOG << "Loading Output[" << o << "] register from Input[" << i << "][" << vc << "], flit: " << flit << endl;
			  
			  // Record path for XY_PATH_REVERSE mode (only for REQUEST HEAD flits)
			  if (GlobalParams::routing_algorithm == ROUTING_XY_PATH_REVERSE &&
			      flit.packet_type == PACKET_TYPE_REQUEST &&
			      (flit.flit_type == FLIT_TYPE_HEAD || flit.flit_type == FLIT_TYPE_HEAD_TAIL) &&
			      o != DIRECTION_LOCAL) {
			      flit.recorded_path.push_back(local_id);
			  }
			  
		  // Oracle coalescing: Mark flit if this router saw duplicates
		  if (flit.packet_type == PACKET_TYPE_REQUEST &&
		      (flit.flit_type == FLIT_TYPE_HEAD || flit.flit_type == FLIT_TYPE_HEAD_TAIL)) {
		      
		      int key = makeOracleKey(flit.feature_id, flit.dst_id);
		      
		      auto itg = oracle_global.find(key);
		      auto itw = oracle_window.find(key);
		      bool seen_global_dups = (itg != oracle_global.end() && itg->second.count > 1);
		      bool seen_window_dups = (itw != oracle_window.end() && itw->second.count > 1);
		      
		      if (seen_global_dups || seen_window_dups) {
		          flit.oracle_coalesce_marked = true;
		      }
		  }			  // Load into output register
			  out_reg[o] = flit;
			  has_flit[o] = true;
			  
			  // Track output port activity
			  port_tx_busy[o]++;
			  
			  // Pop from input buffer
			  buffer[i][vc].Pop();
			  
			  // CRITICAL: Update buffer full status immediately after Pop!
			  // This prevents race condition where downstream sees stale "full" signal
			  // even though we just freed space by popping
			  TBufferFullStatus bfs;
			  for (int v=0; v<GlobalParams::n_virtual_channels; v++)
			      bfs.mask[v] = buffer[i][v].IsFull();
			  buffer_full_status_rx[i].write(bfs);

			  // Release reservation if this is TAIL
			  if (flit.flit_type == FLIT_TYPE_TAIL || flit.flit_type == FLIT_TYPE_HEAD_TAIL)
			  {
			      TReservation r;
			      r.input = i;
			      r.vc = vc;
			      reservation_table.release(r,o);
			  }
			  
			  // Track routed flits (not locally generated)
			  if (i != DIRECTION_LOCAL && o != DIRECTION_LOCAL) {
			      routed_flits++;
			  }

			  // Successfully loaded - break to try next input
			  break;
		      }
		  }
	      }  // End for loop over all reservations
	  }
      } // for loop directions

      if ((int)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps)%2==0)
	  reservation_table.updateIndex();
    }   
}

NoP_data Router::getCurrentNoPData()
{
    NoP_data NoP_data;

    for (int j = 0; j < DIRECTIONS; j++) {
	try {
		NoP_data.channel_status_neighbor[j].free_slots = free_slots_neighbor[j].read();
		NoP_data.channel_status_neighbor[j].available = (reservation_table.isNotReserved(j));
	}
	catch (int e)
	{
	    if (e!=NOT_VALID) assert(false);
	    // Nothing to do if an NOT_VALID direction is caught
	};
    }

    NoP_data.sender_id = local_id;

    return NoP_data;
}

void Router::perCycleUpdate()
{
    if (reset.read()) {
	for (int i = 0; i < DIRECTIONS + 1; i++)
	    free_slots[i].write(buffer[i][DEFAULT_VC].GetMaxBufferSize());
    } else {
        selectionStrategy->perCycleUpdate(this);

	power.leakageRouter();
	for (int i = 0; i < DIRECTIONS + 1; i++)
	{
	    for (int vc=0;vc<GlobalParams::n_virtual_channels;vc++)
	    {
		power.leakageBufferRouter();
		power.leakageLinkRouter2Router();
	    }
	}

	power.leakageLinkRouter2Hub();
	
	// Track buffer occupancy every cycle
	// Count both input buffers AND output registers (pipeline stage)
	total_observation_cycles++;
	buffer_samples++;
	for (int i = 0; i < DIRECTIONS + 2; i++) {
	    int occupancy = 0;
	    // Input buffers (per-VC storage)
	    for (int vc = 0; vc < GlobalParams::n_virtual_channels; vc++) {
		occupancy += buffer[i][vc].Size();
	    }
	    // Output register (1 flit pipeline stage per direction)
	    if (has_flit[i]) {
		occupancy += 1;
	    }
	    buffer_occupancy_sum[i] += occupancy;
	}
    }
}

vector<int> Router::nextDeltaHops(RouteData rd) {

	if (GlobalParams::topology == TOPOLOGY_MESH)
	{
		cout << "Mesh topologies are not supported for nextDeltaHops() ";
		assert(false);
	}
	// annotate the initial nodes
	int src = rd.src_id;
	int dst = rd.dst_id;

	int current_node = src;
	vector<int> direction; // initially is empty
	vector<int> next_hops;

	int sw = GlobalParams::n_delta_tiles/2; //sw: switch number in each stage
	int stg = log2(GlobalParams::n_delta_tiles);
	int c;
	//---From Source to stage 0 (return the sw attached to the source)---
	//Topology omega 
	if (GlobalParams::topology == TOPOLOGY_OMEGA) 	
	{
	if(current_node < (GlobalParams::n_delta_tiles/2))	
		 c = current_node;
	else if(current_node >= (GlobalParams::n_delta_tiles/2))	
		 c = (current_node - (GlobalParams::n_delta_tiles/2));		
	}
	//Other delta topologies: Butterfly and baseline
	else if ((GlobalParams::topology == TOPOLOGY_BUTTERFLY)||(GlobalParams::topology == TOPOLOGY_BASELINE))
	{
		 c =  (current_node >>1);
	}

		Coord temp_coord;
		temp_coord.x = 0;
		temp_coord.y = c;
		int N = coord2Id(temp_coord);

		next_hops.push_back(N);
		current_node = N;
	
	
   //---From stage 0 to Destination---
	int current_stage = 0;

	while (current_stage<stg-1)
	{
		Coord new_coord;
		int y = id2Coord(current_node).y;

		rd.current_id = current_node;
		direction = routingAlgorithm->route(this, rd);

		int bit_to_check = stg - current_stage - 1;

		int bit_checked = (y & (1 << (bit_to_check - 1)))>0 ? 1:0;

		// computes next node coords
		new_coord.x = current_stage + 1;
		if (bit_checked ^ direction[0])
			new_coord.y = toggleKthBit(y, bit_to_check);
		else
			new_coord.y = y;

		current_node = coord2Id(new_coord);
		next_hops.push_back(current_node);
		current_stage = id2Coord(current_node).x;
	}

	next_hops.push_back(dst);

	return next_hops;

}

vector < int > Router::routingFunction(const RouteData & route_data)
{
	if (GlobalParams::use_winoc)
	{
		// - If the current node C and the destination D are connected to an radiohub, use wireless
		// - If D is not directly connected to a radio hub, wireless
		// communication can still  be used if some intermediate node "I" in the routing
		// path is reachable from current node C.
		// - Since further wired hops will be required from I -> D, a threshold "winoc_dst_hops"
		// can be specified (via command line) to determine the max distance from the intermediate
		// node I and the destination D.
		// - NOTE: default threshold is 0, which means I=D, i.e., we explicitly ask the destination D to be connected to the
		// target radio hub
		if (hasRadioHub(local_id))
		{
			// Check if destination is directly connected to an hub
			if ( hasRadioHub(route_data.dst_id) &&
				 !sameRadioHub(local_id,route_data.dst_id) )
			{
                map<int, int>::iterator it1 = GlobalParams::hub_for_tile.find(route_data.dst_id);
                map<int, int>::iterator it2 = GlobalParams::hub_for_tile.find(route_data.current_id);

                if (connectedHubs(it1->second,it2->second))
                {
                    LOG << "Destination node " << route_data.dst_id << " is directly connected to a reachable RadioHub" << endl;
                    vector<int> dirv;
                    dirv.push_back(DIRECTION_HUB);
                    return dirv;
                }
			}
			// let's check whether some node in the route has an acceptable distance to the dst
            if (GlobalParams::winoc_dst_hops>0)
            {
                // TODO: for the moment, just print the set of nexts hops to check everything is ok
                LOG << "NEXT_DELTA_HOPS (from node " << route_data.src_id << " to " << route_data.dst_id << ") >>>> :";
                vector<int> nexthops;
                nexthops = nextDeltaHops(route_data);
                //for (int i=0;i<nexthops.size();i++) cout << "(" << nexthops[i] <<")-->";
                //cout << endl;
                for (int i=1;i<=GlobalParams::winoc_dst_hops;i++)
				{
                	int dest_position = nexthops.size()-1;
                	int candidate_hop = nexthops[dest_position-i];
					if ( hasRadioHub(candidate_hop) && !sameRadioHub(local_id,candidate_hop) ) {
						//LOG << "Checking candidate hop " << candidate_hop << " ... It's OK!" << endl;
						LOG << "Relaying to hub-connected node " << candidate_hop << " to reach destination " << route_data.dst_id << endl;
						vector<int> dirv;
						dirv.push_back(DIRECTION_HUB_RELAY+candidate_hop);
						return dirv;
					}
					//else
					// LOG << "Checking candidate hop " << candidate_hop << " ... NOT OK" << endl;
				}
            }
		}
	}
	// TODO: fix all the deprecated verbose mode logs
	if (GlobalParams::verbose_mode > VERBOSE_OFF)
		LOG << "Wired routing for dst = " << route_data.dst_id << endl;

	// not wireless direction taken, apply normal routing
	return routingAlgorithm->route(this, route_data);
}

int Router::route(const RouteData & route_data)
{

    if (route_data.dst_id == local_id)
	return DIRECTION_LOCAL;

    power.routing();
    vector < int >candidate_channels = routingFunction(route_data);

    power.selection();
    return selectionFunction(candidate_channels, route_data);
}

void Router::NoP_report() const
{
    NoP_data NoP_tmp;
	LOG << "NoP report: " << endl;

    for (int i = 0; i < DIRECTIONS; i++) {
	NoP_tmp = NoP_data_in[i].read();
	if (NoP_tmp.sender_id != NOT_VALID)
	    cout << NoP_tmp;
    }
}

//---------------------------------------------------------------------------

int Router::NoPScore(const NoP_data & nop_data,
			  const vector < int >&nop_channels) const
{
    int score = 0;

    for (unsigned int i = 0; i < nop_channels.size(); i++) {
	int available;

	if (nop_data.channel_status_neighbor[nop_channels[i]].available)
	    available = 1;
	else
	    available = 0;

	int free_slots =
	    nop_data.channel_status_neighbor[nop_channels[i]].free_slots;

	score += available * free_slots;
    }

    return score;
}

int Router::selectionFunction(const vector < int >&directions,
				   const RouteData & route_data)
{
    // not so elegant but fast escape ;)
    if (directions.size() == 1)
	return directions[0];

    return selectionStrategy->apply(this, directions, route_data);
}

void Router::configure(const int _id,
			    const double _warm_up_time,
			    const unsigned int _max_buffer_size,
			    GlobalRoutingTable & grt)
{
    local_id = _id;
    stats.configure(_id, _warm_up_time);

    start_from_port = DIRECTION_LOCAL;
    
    // Initialize link utilization tracking
    total_observation_cycles = 0;
    buffer_samples = 0;
    for (int i = 0; i < DIRECTIONS + 2; i++) {
        port_rx_busy[i] = 0;
        port_tx_busy[i] = 0;
        buffer_occupancy_sum[i] = 0;
    }
    
    // Initialize oracle coalescing stats
    oracle_global_total_heads = 0;
    oracle_global_coalesced_heads = 0;
    oracle_window_total_heads = 0;
    oracle_window_coalesced_heads = 0;
    oracle_inflight_total_heads = 0;
    oracle_inflight_coalesced_heads = 0;
  

    if (grt.isValid())
	routing_table.configure(grt, _id);

    reservation_table.setSize(DIRECTIONS+2);

    for (int i = 0; i < DIRECTIONS + 2; i++)
    {
	for (int vc = 0; vc < GlobalParams::n_virtual_channels; vc++)
	{
	    buffer[i][vc].SetMaxBufferSize(_max_buffer_size);
	    buffer[i][vc].setLabel(string(name())+"->buffer["+i_to_string(i)+"]");
	}
	start_from_vc[i] = 0;
    }


    if (GlobalParams::topology == TOPOLOGY_MESH)
    {
	int row = _id / GlobalParams::mesh_dim_x;
	int col = _id % GlobalParams::mesh_dim_x;

	for (int vc = 0; vc<GlobalParams::n_virtual_channels; vc++)
	{
	    if (row == 0)
	      buffer[DIRECTION_NORTH][vc].Disable();
	    if (row == GlobalParams::mesh_dim_y-1)
	      buffer[DIRECTION_SOUTH][vc].Disable();
	    if (col == 0)
	      buffer[DIRECTION_WEST][vc].Disable();
	    if (col == GlobalParams::mesh_dim_x-1)
	      buffer[DIRECTION_EAST][vc].Disable();
	}
    }

}

unsigned long Router::getRoutedFlits()
{
    return routed_flits;
}


int Router::reflexDirection(int direction) const
{
    if (direction == DIRECTION_NORTH)
	return DIRECTION_SOUTH;
    if (direction == DIRECTION_EAST)
	return DIRECTION_WEST;
    if (direction == DIRECTION_WEST)
	return DIRECTION_EAST;
    if (direction == DIRECTION_SOUTH)
	return DIRECTION_NORTH;

    // you shouldn't be here
    assert(false);
    return NOT_VALID;
}

int Router::getNeighborId(int _id, int direction) const
{
    assert(GlobalParams::topology == TOPOLOGY_MESH);

    Coord my_coord = id2Coord(_id); 

    switch (direction) {
    case DIRECTION_NORTH:
	if (my_coord.y == 0)
	    return NOT_VALID;
	my_coord.y--;
	break;
    case DIRECTION_SOUTH:
	if (my_coord.y == GlobalParams::mesh_dim_y - 1)
	    return NOT_VALID;
	my_coord.y++;
	break;
    case DIRECTION_EAST:
	if (my_coord.x == GlobalParams::mesh_dim_x - 1)
	    return NOT_VALID;
	my_coord.x++;
	break;
    case DIRECTION_WEST:
	if (my_coord.x == 0)
	    return NOT_VALID;
	my_coord.x--;
	break;
    default:
	LOG << "Direction not valid : " << direction;
	assert(false);
    }

    int neighbor_id = coord2Id(my_coord);

    return neighbor_id;
}

bool Router::inCongestion()
{
    for (int i = 0; i < DIRECTIONS; i++) {

	if (free_slots_neighbor[i]==NOT_VALID) continue;

	int flits = GlobalParams::buffer_depth - free_slots_neighbor[i];
	if (flits > (int) (GlobalParams::buffer_depth * GlobalParams::dyad_threshold))
	    return true;
    }

    return false;
}

void Router::ShowBuffersStats(std::ostream & out)
{
  for (int i=0; i<DIRECTIONS+2; i++)
      for (int vc=0; vc<GlobalParams::n_virtual_channels;vc++)
	    buffer[i][vc].ShowStats(out);
}


bool Router::connectedHubs(int src_hub, int dst_hub) {
    vector<int> &first = GlobalParams::hub_configuration[src_hub].txChannels;
    vector<int> &second = GlobalParams::hub_configuration[dst_hub].rxChannels;

    vector<int> intersection;

    for (unsigned int i = 0; i < first.size(); i++) {
        for (unsigned int j = 0; j < second.size(); j++) {
            if (first[i] == second[j])
                intersection.push_back(first[i]);
        }
    }

    if (intersection.size() == 0)
        return false;
    else
        return true;
}

void Router::printLinkUtilization() const
{
    if (total_observation_cycles == 0) {
        return;  // No data collected yet
    }
    
    const char* dir_names[] = {"North", "East", "South", "West", "Local", "Hub"};
    
    cout << "Router[" << local_id << "] Link Utilization:" << endl;
    
    // Print input port utilization
    cout << "  Input Ports:" << endl;
    for (int i = 0; i < DIRECTIONS + 2; i++) {
        double util = 100.0 * port_rx_busy[i] / total_observation_cycles;
        double avg_occupancy = (buffer_samples > 0) ? 
            (double)buffer_occupancy_sum[i] / buffer_samples : 0.0;
        
        cout << "    " << dir_names[i] << ": " 
             << fixed << setprecision(1) << util << "% "
             << "(" << port_rx_busy[i] << "/" << total_observation_cycles << " cyc), "
             << "Avg buf: " << fixed << setprecision(1) << avg_occupancy << " flits" << endl;
    }
    
    // Print output port utilization
    cout << "  Output Ports:" << endl;
    for (int i = 0; i < DIRECTIONS + 2; i++) {
        double util = 100.0 * port_tx_busy[i] / total_observation_cycles;
        
        cout << "    " << dir_names[i] << ": " 
             << fixed << setprecision(1) << util << "% "
             << "(" << port_tx_busy[i] << "/" << total_observation_cycles << " cyc)" << endl;
    }
}

// Oracle coalescing tracking - called when a flit enters the router
void Router::trackOracleCoalescing(const Flit &f)
{
    // Get current cycle (used by multiple oracles)
    uint64_t cur_cycle = (uint64_t)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
    
    // Build key: feature_id (dst_id is deterministic for each feature_id)
    int key = makeOracleKey(f.feature_id, f.dst_id);
    
    // ========================================================================
    // HANDLE RESPONSE PACKETS: Manage batch lifecycle
    // ========================================================================
    if (f.packet_type == PACKET_TYPE_RESPONSE) {
        if (f.flit_type == FLIT_TYPE_TAIL || f.flit_type == FLIT_TYPE_HEAD_TAIL) {
            auto it = oracle_inflight.find(key);
            if (it != oracle_inflight.end()) {
                // Check if this response is for the CURRENT batch or a PREVIOUS batch
                if (it->second.pending_old_responses > 0) {
                    // This response is for a PREVIOUS batch (coalesced request from old batch)
                    // Decrement pending counter but DON'T affect current batch
                    it->second.pending_old_responses--;
                    
                    // If no more old responses pending and current batch is also done, erase
                    if (it->second.pending_old_responses == 0 && it->second.outstanding == 0) {
                        oracle_inflight.erase(it);
                    }
                } else {
                    // This response is for the CURRENT batch
                    // This is the FIRST response - it ends the current batch
                    // Move outstanding count to pending_old_responses (except the first one)
                    uint32_t coalesced_count = it->second.outstanding - 1;
                    
                    if (coalesced_count > 0) {
                        // There were coalesced requests - their responses are still coming
                        // Keep entry alive but mark current batch as done
                        it->second.outstanding = 0;
                        it->second.pending_old_responses = coalesced_count;
                    } else {
                        // No coalesced requests - erase immediately
                        oracle_inflight.erase(it);
                    }
                }
            }
            // If entry not found, this is a response for an old batch that's fully cleaned up (OK)
        }
        return;  // Done processing response
    }
    
    // ========================================================================
    // HANDLE REQUEST PACKETS: Track coalescing opportunities
    // ========================================================================
    if (f.packet_type != PACKET_TYPE_REQUEST) {
        return;  // Not a request or response - ignore
    }
    
    if (f.flit_type != FLIT_TYPE_HEAD && f.flit_type != FLIT_TYPE_HEAD_TAIL) {
        return;  // Only track HEAD flits for requests
    }
    
    // Skip if already marked as coalesced at an upstream router
    if (f.oracle_coalesce_marked) {
        return;
    }
    
    // ========================================================================
    // GLOBAL ORACLE (no time window - unlimited time)
    // ========================================================================
    oracle_global_total_heads++;
    
    auto &entry = oracle_global[key];
    if (entry.count == 0) {
        // First time seeing this feature_id at this router
        entry.first_cycle = cur_cycle;
        entry.count = 1;
    } else {
        // This is a duplicate - could have been coalesced at this router
        entry.count++;
        oracle_global_coalesced_heads++;
    }
    
    // ========================================================================
    // WINDOWED ORACLE (300-cycle window)
    // ========================================================================
    static const uint64_t ORACLE_WINDOW = 300;
    
    oracle_window_total_heads++;
    
    auto &wentry = oracle_window[key];
    if (wentry.count == 0) {
        // First time seeing this feature_id at this router
        wentry.first_cycle = cur_cycle;
        wentry.count = 1;
    } else {
        if (cur_cycle <= wentry.first_cycle + ORACLE_WINDOW) {
            // Within window - can be coalesced
            wentry.count++;
            oracle_window_coalesced_heads++;
        } else {
            // Outside window - start a new window
            wentry.first_cycle = cur_cycle;
            wentry.count = 1;
        }
    }
    
    // ========================================================================
    // IN-FLIGHT ORACLE (until first response returns)
    // ========================================================================
    oracle_inflight_total_heads++;
    
    // Check if there's already an in-flight request for this feature
    auto it = oracle_inflight.find(key);
    if (it != oracle_inflight.end() && it->second.outstanding > 0) {
        // There is an ACTIVE batch (outstanding > 0) for this feature at this router
        // This request can be coalesced with it
        oracle_inflight_coalesced_heads++;
        it->second.outstanding++;  // Track how many were coalesced in this batch
    } else {
        // Either no entry exists, OR entry exists but current batch is done (outstanding == 0)
        // and we're just waiting for old responses to drain.
        // Either way, this is a MISS - start a NEW batch
        // NOTE: With XY routing (deterministic), responses arrive in-order, so pending_old_responses
        // will correctly track old batch responses without mixing with new batch responses.
        OracleInflightEntry inf;
        inf.first_cycle = cur_cycle;
        inf.outstanding = 1;
        inf.pending_old_responses = (it != oracle_inflight.end()) ? it->second.pending_old_responses : 0;
        oracle_inflight[key] = inf;
    }
}

// Print oracle coalescing statistics for this router
void Router::printOracleCoalescingStats() const
{
    cout << "Router[" << local_id << "] Oracle Coalescing Stats:" << endl;
    
    cout << "  GLOBAL (no window):" << endl;
    cout << "    Heads seen:        " << oracle_global_total_heads << endl;
    cout << "    Coalesced heads:   " << oracle_global_coalesced_heads << endl;
    if (oracle_global_total_heads > 0) {
        cout << "    Coalescing ratio:  "
             << fixed << setprecision(1)
             << (100.0 * oracle_global_coalesced_heads / oracle_global_total_heads)
             << "%" << endl;
    }
    
    cout << "  WINDOW (<= 300 cyc):" << endl;
    cout << "    Heads seen:        " << oracle_window_total_heads << endl;
    cout << "    Coalesced heads:   " << oracle_window_coalesced_heads << endl;
    if (oracle_window_total_heads > 0) {
        cout << "    Coalescing ratio:  "
             << fixed << setprecision(1)
             << (100.0 * oracle_window_coalesced_heads / oracle_window_total_heads)
             << "%" << endl;
    }
    
    cout << "  IN-FLIGHT (until reply):" << endl;
    cout << "    Heads seen:        " << oracle_inflight_total_heads << endl;
    cout << "    Coalesced heads:   " << oracle_inflight_coalesced_heads << endl;
    if (oracle_inflight_total_heads > 0) {
        cout << "    Coalescing ratio:  "
             << fixed << setprecision(1)
             << (100.0 * oracle_inflight_coalesced_heads / oracle_inflight_total_heads)
             << "%" << endl;
    }
}