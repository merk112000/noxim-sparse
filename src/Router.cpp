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

	// === CRITICAL: Flip multicast_allowed at the multicast root router ===
	// The response travels with multicast_allowed=false from memory until it reaches
	// the router (multicast_root_id) that actually coalesced THIS specific request.
	// At that router, we flip it to true so multicast starts there.
	// Upstream routers with coalesce entries will also multicast.
	// This fixes the "old vs new request" bug.
	if (GlobalParams::enable_selective_coalescing &&
	    received_flit.packet_type == PACKET_TYPE_RESPONSE) {
	    
	    if (!received_flit.multicast_allowed &&
	        received_flit.multicast_root_id == local_id) {
	        // We are the multicast root for THIS request - flip multicast on!
	        received_flit.multicast_allowed = true;
	        
	        if ((uint64_t)sc_time_stamp().to_double() / GlobalParams::clock_period_ps <= 200000) {
	            cout << "Router[" << local_id << "] MULTICAST ROOT: Flipping multicast_allowed=true for fid=" 
	                 << received_flit.feature_id << endl;
	        }
	    }
	}

	// Check if this is a multicast response that would go to McEngine
	bool will_use_mc_engine = false;
	bool mc_fifo_has_space = true;
	int mc_idx_for_head = -1;  // Pre-allocate McEntry for HEAD if needed
	
	if (GlobalParams::enable_selective_coalescing && 
	    received_flit.packet_type == PACKET_TYPE_RESPONSE &&
	    received_flit.multicast_allowed) {  // CRITICAL: Only if this specific response can multicast
	    int entry_idx = findCoalesceEntry(received_flit.feature_id);
	    if (entry_idx >= 0) {
	        will_use_mc_engine = true;
	        
	        if (received_flit.flit_type == FLIT_TYPE_HEAD) {
	            // For HEAD: Must check if McEngine has free slot
	            mc_idx_for_head = allocateMcEntry(received_flit.feature_id);
	            if (mc_idx_for_head < 0) {
	                // McEngine full - MUST backpressure, cannot downgrade to unicast
	                mc_fifo_has_space = false;
	            }
	            // Note: If allocated successfully, we'll use mc_idx_for_head later
	        } else if (received_flit.flit_type == FLIT_TYPE_BODY || 
	                   received_flit.flit_type == FLIT_TYPE_TAIL) {
	            // For BODY/TAIL: Check existing McEntry FIFO space
	            auto it = input_to_mc_entry.find(std::make_tuple(i, vc, received_flit.feature_id));
	            if (it != input_to_mc_entry.end()) {
	                int mc_idx = it->second;
	                McEntry &mc = mc_engine[mc_idx];
	                mc_fifo_has_space = (mc.fifo.size() < McEntry::MAX_FIFO_SIZE);
	            } else {
	                // BODY/TAIL arrived but no McEntry exists - protocol violation
	                mc_fifo_has_space = false;
	            }
	        }
	    }
	}
	
	// Backpressure logic:
	// - Unicast traffic: check buffer space
	// - Multicast traffic: check McEngine capacity (McEntry slot + FIFO space)
	// - CRITICAL: If multicast resource unavailable, MUST backpressure (never downgrade to unicast)
	bool can_accept = will_use_mc_engine ? mc_fifo_has_space : !buffer[i][vc].IsFull();

	if (can_accept) 
	{
	    // Buffer has space - Accept the flit: VALID=1 and READY=1, so transfer occurs
	    ready = true;  // Signal READY
	    port_rx_busy[i]++;
	    
	    // SELECTIVE COALESCING: Check if this request should be merged
	    // Handle BOTH single-flit (HEAD_TAIL) and multi-flit (HEAD) packets
	    bool drop_request = false;
	    if (GlobalParams::enable_selective_coalescing && 
	        received_flit.packet_type == PACKET_TYPE_REQUEST && 
	        (received_flit.flit_type == FLIT_TYPE_HEAD_TAIL || received_flit.flit_type == FLIT_TYPE_HEAD)) {
	        drop_request = handleRequestCoalescing(received_flit, i);
	    }
	    
	    // MULTICAST ENGINE: Detect RESPONSE packets that need multicasting
	    bool moved_to_mc_engine = false;
	    if (GlobalParams::enable_selective_coalescing && 
	        received_flit.packet_type == PACKET_TYPE_RESPONSE) {
		        int entry_idx = findCoalesceEntry(received_flit.feature_id);
		        if (entry_idx >= 0) {
		            CoalesceEntry &ce = coalesce_table[entry_idx];
		            std::vector<int> multicast_outputs = getMulticastOutputs(received_flit);
		            
		                /*cerr << "Router[" << local_id << "] RESPONSE fid=" << received_flit.feature_id 
		                     << " multicast_outputs.size()=" << multicast_outputs.size() << " ports=";
		                for (int p : multicast_outputs) cerr << p << " ";
		                cerr << endl;*/
		            
		            // Move ALL eligible responses to multicast engine, even if only 1 output
		            if (multicast_outputs.size() >= 1) {
		                // This response is eligible - move to MC engine
		                if (received_flit.flit_type == FLIT_TYPE_HEAD) {
		                    // HEAD flit - use pre-allocated McEntry from backpressure check
		                    if (mc_idx_for_head >= 0) {
		                        McEntry &mc = mc_engine[mc_idx_for_head];
		                        mc.src_memtile = received_flit.src_id;
                        mc.out_ports_needed.reset();
                        
                        // Use feature_id to deterministically assign VC (simple hash)
                        // This spreads multicasts across all VCs
                        int assigned_vc = received_flit.feature_id % GlobalParams::n_virtual_channels;
                        
                        for (int out : multicast_outputs) {
		                            mc.out_ports_needed.set(out);
		                            mc.port[out].needed = true;
		                            mc.port[out].next_flit_idx = 0;
		                            mc.port[out].vc = assigned_vc;
		                        }
                        
                        // Push HEAD to FIFO (guaranteed space - just allocated)
                        mc.fifo.push_back(received_flit);
                        input_to_mc_entry[std::make_tuple(i, vc, received_flit.feature_id)] = mc_idx_for_head;
                        moved_to_mc_engine = true;
                        ce.response_started = true;
		                    } else {
		                        cerr << "Router[" << local_id << "] FATAL: McEntry not pre-allocated for HEAD fid=" 
		                             << received_flit.feature_id << " but can_accept was true!" << endl;
		                        assert(false);
		                    }
		                } else {
		                    // BODY or TAIL flit - find existing McEntry using feature_id
		                    auto it = input_to_mc_entry.find(std::make_tuple(i, vc, received_flit.feature_id));
		                    if (it != input_to_mc_entry.end()) {
		                        int mc_idx = it->second;
		                        McEntry &mc = mc_engine[mc_idx];
		                        // Validate feature_id match
		                        if (mc.feature_id == received_flit.feature_id) {
		                            // Push to FIFO (space already verified in backpressure check)
		                            if (mc.fifo.size() < McEntry::MAX_FIFO_SIZE) {
		                                mc.fifo.push_back(received_flit);
		                                moved_to_mc_engine = true;
		                            } else {
		                                // CRITICAL: This should NEVER happen - we backpressured if FIFO full
		                                cerr << "Router[" << local_id << "] FATAL: McEngine[" << mc_idx 
		                                     << "] FIFO full but can_accept was true for fid=" 
		                                     << received_flit.feature_id << endl;
		                                assert(false);
		                            }
		                        } else {
		                            cerr << "Router[" << local_id << "] ERROR: feature_id mismatch! "
		                                << "McEntry[" << mc_idx << "] has fid=" << mc.feature_id
		                                << " but received fid=" << received_flit.feature_id << endl;
		                        }
		                    } else {
		                        // CRITICAL: BODY/TAIL but no McEntry - should have been backpressured
		                        cerr << "Router[" << local_id << "] FATAL: BODY/TAIL flit for fid=" 
		                             << received_flit.feature_id << " but no McEntry exists!" << endl;
		                        assert(false);
		                    }
		                }
		            }
		        }
		    }
		    
		    if (!drop_request && !moved_to_mc_engine) {
		        // Normal path: push flit to buffer
		        // CRITICAL: Multicast responses should NEVER reach here - they must go to McEngine or be backpressured
		        if (will_use_mc_engine) {
		            cerr << "Router[" << local_id << "] FATAL: Multicast response fid=" << received_flit.feature_id
		                 << " not moved to McEngine but also not backpressured!" << endl;
		            assert(false);
		        }
		        
		        buffer[i][vc].Push(received_flit);
		       // LOG << " Flit " << received_flit << " collected from Input[" << i << "][" << vc <<"]" << endl;
		        power.bufferRouterPush();

		        if (received_flit.src_id == local_id)
			    power.networkInterface();
		        
		        // Track oracle coalescing opportunities (stats only)
		        trackOracleCoalescing(received_flit);
		    }
		    
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
      // PHASE 0: Serve Multicast Engine (separate from reservation table)
      // ========================================================================
      serveMcEngine();
 
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

	  for (int k = 0; k < GlobalParams::n_virtual_channels; k++)
	  {
	      int vc = (start_from_vc[i] + k) % GlobalParams::n_virtual_channels;
	      
	      if (!buffer[i][vc].IsEmpty()) 
	      {
		  Flit flit = buffer[i][vc].Front();
		  power.bufferRouterFront();

		  if (flit.flit_type == FLIT_TYPE_HEAD || flit.flit_type == FLIT_TYPE_HEAD_TAIL) 
		    {
		      // Normal routing - unicast only (multicast handled by separate engine in Phase 0)
		      {
		          // UNICAST: Normal routing and reservation
		          RouteData route_data;
		          route_data.current_id = local_id;
		          route_data.src_id = flit.src_id;
		          route_data.dst_id = flit.dst_id;
		          route_data.dir_in = i;
		          route_data.vc_id = flit.vc_id;
		          route_data.packet_type = flit.packet_type;
		          route_data.recorded_path = flit.recorded_path;
		          route_data.multicast_dests.clear();

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

				  // DEADLOCK FIX: Check if multicast is using this (output, VC) before trying to reserve
				  bool mc_blocking = false;
				  if (GlobalParams::enable_selective_coalescing) {
				      mc_blocking = mc_vc_busy[o][vc];
				  }

				  int rt_status = mc_blocking ? RT_OUTVC_BUSY : reservation_table.checkReservation(r, o);		          if (rt_status == RT_AVAILABLE) 
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
	  }
	    start_from_vc[i] = (start_from_vc[i]+1) % GlobalParams::n_virtual_channels;
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
	      bool served_input_i = false;  // Prevent multiple pops from same input per cycle
	      
	      // Try ALL reservations starting from random index
	      for (size_t attempt = 0; attempt < reservations.size() && !served_input_i; attempt++) {
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
	      
	      // Check if downstream buffer has space
	      bool output_ready = !buffer_full_status_tx[o].read().mask[vc];
	      
	      // CRITICAL: Check if multicast started using this VC (prevent race)
	      // If multicast set mc_vc_busy in Phase 0 before we checked in Phase 2,
	      // we must abort to avoid collision
	      bool mc_using_vc = false;
	      if (GlobalParams::enable_selective_coalescing) {
	          mc_using_vc = mc_vc_busy[o][vc];
	      }
	      
	      if (output_ready && !mc_using_vc)
	      {
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
		              }
		              
		              // Load flit to output register
		              LOG << "Loading Output[" << o << "] register from Input[" << i << "][" << vc << "], flit: " << flit << endl;
		              
		              // CRITICAL: Set credit_count for RESPONSE packets based on coalesce table
		              // REMOVED: Multi-credit logic - each response returns exactly 1 credit
		              // since we now prevent duplicate PE coalescing
		              
		              out_reg[o] = flit;
		              has_flit[o] = true;
		              port_tx_busy[o]++;
		              
		              // Pop from input buffer
		              buffer[i][vc].Pop();
		              served_input_i = true;  // Prevent multiple pops from same input

		              
		              // CRITICAL: Update buffer full status immediately after Pop!
		              TBufferFullStatus bfs;
		              for (int v=0; v<GlobalParams::n_virtual_channels; v++)
		                  bfs.mask[v] = buffer[i][v].IsFull();
		              buffer_full_status_rx[i].write(bfs);

	              // Release reservation on TAIL
	              if (flit.flit_type == FLIT_TYPE_TAIL || flit.flit_type == FLIT_TYPE_HEAD_TAIL)
	              {
	                  TReservation r;
	                  r.input = i;
	                  r.vc = vc;
	                  reservation_table.release(r, o);
	                  
	                  // NOTE: Coalesce entries are ONLY freed by McEngine after all ports done.
	                  // Phase 3 (unicast path) should NEVER free coalesce entries because
	                  // all eligible responses go through McEngine (even single-port ones).
	              }
	              
	              // Track routed flits (not locally generated)
		              if (i != DIRECTION_LOCAL && o != DIRECTION_LOCAL) {
		                  routed_flits++;
		              }

		              // Successfully loaded - break to try next input
		              break;
		          }
	      }  // End if buffer not empty
	  }  // End for loop over all reservations
	  }  // End if reservations exist
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
        // Clean up stale coalesce entries (every cycle check)
        if (GlobalParams::enable_selective_coalescing) {
            cleanupStaleCoalesceEntries();
        }
        
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
                LOG << "Checking candidate hop " << candidate_hop << " ... It's OK!" << endl;
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
    // Check if we should deliver to local PE based ONLY on dst_id
    // multicast_dests is NOT used for routing - only for PE acceptance check
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
    
    // Initialize selective coalescing stats and table
    coalesce_requests_received = 0;
    coalesce_requests_merged = 0;
    coalesce_table_full_events = 0;
    coalesce_responses_multicast = 0;
    coalesce_entries_timed_out = 0;
    multicast_vc_rr_counter = 0;  // No longer used - VCs preserved from incoming traffic
    mc_rr_idx = 0;  // Round-robin starting index for multicast engine
    
    // Initialize multicast VC busy tracking
    for (int port = 0; port < DIRECTIONS + 2; port++) {
        for (int vc = 0; vc < MAX_VIRTUAL_CHANNELS; vc++) {
            mc_vc_busy[port][vc] = false;
        }
    }
    
    for (int i = 0; i < COALESCE_TABLE_SIZE; i++) {
        coalesce_table[i].valid = false;
    }
  

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

// ============================================================================
// SELECTIVE IN-ROUTER COALESCING IMPLEMENTATION
// ============================================================================

// Find existing coalescing entry by feature_id
int Router::findCoalesceEntry(int feature_id)
{
    for (int i = 0; i < COALESCE_TABLE_SIZE; i++) {
        if (coalesce_table[i].valid && coalesce_table[i].feature_id == feature_id) {
            return i;
        }
    }
    return -1;  // Not found
}

// Allocate new coalescing entry
int Router::allocateCoalesceEntry(int feature_id)
{
    for (int i = 0; i < COALESCE_TABLE_SIZE; i++) {
        if (!coalesce_table[i].valid) {
            coalesce_table[i].valid = true;
            coalesce_table[i].feature_id = feature_id;
            coalesce_table[i].inflight = false;
            coalesce_table[i].response_started = false;
            coalesce_table[i].requester_ports.reset();
            coalesce_table[i].requester_pe_counts.clear();
            coalesce_table[i].allocation_cycle = (uint64_t)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
            return i;
        }
    }
    return -1;  // Table full
}

// Free coalescing entry
void Router::freeCoalesceEntry(int entry_idx)
{
    if (entry_idx >= 0 && entry_idx < COALESCE_TABLE_SIZE) {
        coalesce_table[entry_idx].valid = false;
        coalesce_table[entry_idx].feature_id = -1;
        coalesce_table[entry_idx].inflight = false;
        coalesce_table[entry_idx].response_started = false;
        coalesce_table[entry_idx].requester_ports.reset();
        coalesce_table[entry_idx].requester_pe_counts.clear();
    }
}

// Handle request coalescing - returns true if request should be dropped (merged)
bool Router::handleRequestCoalescing(Flit &f, int input_dir)
{
    // Only coalesce if mechanism is enabled and hint is set
    if (!GlobalParams::enable_selective_coalescing || !f.coalesce_hint) {
        return false;  // Don't drop, forward normally
    }
    
    // CRITICAL: Check coalesce_allowed flag to enforce contiguous prefix
    // If a previous router disabled coalescing (table full), we must not coalesce here
    if (!f.coalesce_allowed) {
        return false;  // Don't drop, forward normally (no coalescing downstream)
    }
    
    // Only process HEAD or HEAD_TAIL flits
    if (f.flit_type != FLIT_TYPE_HEAD && f.flit_type != FLIT_TYPE_HEAD_TAIL) {
        return false;
    }
    
    coalesce_requests_received++;
    
    int entry_idx = findCoalesceEntry(f.feature_id);
    
    if (entry_idx >= 0) {
        // Entry exists - this is a duplicate request that arrived from a different port
        CoalesceEntry &entry = coalesce_table[entry_idx];

        // CRITICAL: Check if this PE has already coalesced for this feature
        // If the same PE sends multiple requests for the same feature, only the FIRST should coalesce
        // Subsequent requests from the same PE are treated as non-coalesceable (separate requests)
        if (entry.requester_pe_counts.count(f.src_id) > 0 && entry.requester_pe_counts[f.src_id] > 0) {
            // This PE already coalesced a request for this feature - DON'T merge again
            // Disable coalescing for THIS request so it gets its own response
        uint64_t cur_cycle_merge = (uint64_t)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
       /* if (cur_cycle_merge <= 200000) {
                cout << "[Router " << local_id << "] COALESCE: PE " << f.src_id 
                     << " already coalesced for fid=" << f.feature_id 
                     << " - treating as NEW request (no coalesce)" << "at" << cur_cycle_merge << endl;
            }*/
            f.coalesce_allowed = false;  // This request won't coalesce
            f.multicast_root_id = -1;
            return false;  // Don't drop, forward normally as separate request
        }
        
        // CRITICAL: Check if response has already started multicasting
        if (entry.response_started) {
            // Response already in flight - cannot merge this late request
            // IMPORTANT: Disable coalescing for THIS REQUEST downstream
            // This ensures its response will NOT multicast (per-request semantics)
          //  LOG << "Router[" << local_id << "] COALESCE: Late request for feature " << f.feature_id
            //    << " arrived after response started - disabling coalescing for THIS request" << endl;
            f.coalesce_allowed = false;  // This request's response must NOT multicast
            return false;  // DON'T DROP - forward normally
        }
        
        // Response not started yet and PE hasn't coalesced before - safe to merge
        // Record this input port (multiple requests can come from same port)
        const int NUM_PORTS = DIRECTIONS + 2;
        if (input_dir >= 0 && input_dir < NUM_PORTS) {
            entry.requester_ports.set(input_dir);
            // Track which PE requested from this port
            entry.port_to_pe_ids[input_dir].insert(f.src_id);
        }
        
        // Mark that this PE has coalesced (count = 1 always, since we block duplicates above)
        entry.requester_pe_counts[f.src_id] = 1;
        
        coalesce_requests_merged++;
        
     /*   uint64_t cur_cycle_merge = (uint64_t)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
        if (cur_cycle_merge <= 50000) {
            cerr << "Router[" << local_id << "] COALESCE MERGE: PE[" << f.src_id 
                 << "] fid=" << f.feature_id << " from port[" << input_dir 
                 << "] - Total PEs for this feature: " << entry.requester_pe_counts.size() << endl;
        }*/
        
        return true;  // DROP this request (it's merged)
    } else {
        // No existing entry - this is the first request for this feature at this router
        entry_idx = allocateCoalesceEntry(f.feature_id);
        
        if (entry_idx >= 0) {
            // Successfully allocated
            CoalesceEntry &entry = coalesce_table[entry_idx];
            entry.inflight = true;
            
            // CRITICAL: This router successfully created a coalescing entry for THIS request
            // Mark this router as the multicast root (closest to memory so far)
            // On reverse path, multicast will start at this router
            f.multicast_root_id = local_id;
            
            // Record this input port
            const int NUM_PORTS = DIRECTIONS + 2;
            if (input_dir >= 0 && input_dir < NUM_PORTS) {
                entry.requester_ports.set(input_dir);
                // Track which PE requested from this port
                entry.port_to_pe_ids[input_dir].insert(f.src_id);
            }
            
            // Record the PE ID with initial count of 1
            entry.requester_pe_counts[f.src_id] = 1;
            
         /*   uint64_t cur_cycle_new = (uint64_t)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
            if (cur_cycle_new <= 50000) {
                cerr << "Router[" << local_id << "] COALESCE NEW: PE[" << f.src_id 
                     << "] fid=" << f.feature_id << " from port[" << input_dir 
                     << "] - First request, allocated entry " << entry_idx << endl;
            }*/
            
            return false;  // Forward this request normally
        } else {
            // CRITICAL: Table full - STOP coalescing for this feature on all downstream routers
            // Set coalesce_allowed = false to enforce contiguous prefix property
            coalesce_table_full_events++;
            
            LOG << "Router[" << local_id << "] COALESCE: Table full for feature " 
                << f.feature_id << " - disabling coalescing for downstream routers" << endl;
            
            f.coalesce_allowed = false;  // Prevent any downstream router from coalescing this feature
            
            return false;  // Forward normally (no coalescing here or downstream)
        }
    }
}

// ============================================================================
// MULTICAST ENGINE IMPLEMENTATION (Phase 0)
// ============================================================================

void Router::serveMcEngine()
{
    // Serve multicast engine entries - send flits gradually to multiple outputs
    // Each port progresses independently through the FIFO
    // CRITICAL FIX: Multicast now uses reservation table to prevent deadlock
    // FAIRNESS: Use round-robin to avoid starving high-index entries
    for (int step = 0; step < MC_ENGINE_SIZE; step++) {
        int mc_idx = (mc_rr_idx + step) % MC_ENGINE_SIZE;
        McEntry &mc = mc_engine[mc_idx];
        
        if (!mc.valid) {
            continue;  // Entry not active
        }
        
        if (mc.fifo.empty()) {
            cerr << "Router[" << local_id << "] ERROR: McEngine[" << mc_idx << "] valid but FIFO empty for fid=" << mc.feature_id << endl;
            continue;
        }
        
        // PHASE A: Check if unicast is using VCs we need (multicast bypasses reservation table)
        // FAIRNESS: Use per-entry round-robin to fairly cycle through ports
        int start_port = mc.port_rr_start;
        for (int port_step = 0; port_step < DIRECTIONS + 2; port_step++) {
            int o = (start_port + port_step) % (DIRECTIONS + 2);
            if (!mc.port[o].needed || mc.port[o].done || mc.port[o].head_sent) {
                continue;  // Skip if not needed, done, or HEAD already sent
            }
            
            // Check if we're at the HEAD flit for this output
            int idx = mc.port[o].next_flit_idx;
            if (idx >= (int)mc.fifo.size()) {
                continue;
            }
            
            Flit f = mc.fifo[idx];
            if (f.flit_type != FLIT_TYPE_HEAD) {
                continue;  // Not at HEAD yet for this output
            }
            
            // Check if unicast OR another multicast has reserved this (output, VC)
            // If anyone is using it, we must wait
            bool vc_in_use = false;
            
            // Check 1: Is another multicast already using this (o, vc)?
            if (mc_vc_busy[o][mc.port[o].vc]) {
                vc_in_use = true;
            }
            
            // Check 2: Has unicast reserved this (o, vc)?
            if (!vc_in_use) {
                for (int check_input = 0; check_input < DIRECTIONS + 2; check_input++) {
                    vector<pair<int,int>> reservations = reservation_table.getReservations(check_input);
                    for (auto res : reservations) {
                        if (res.first == o && res.second == mc.port[o].vc) {
                            vc_in_use = true;
                            break;
                        }
                    }
                    if (vc_in_use) break;
                }
            }
            
            // Can only proceed if VC is free for both unicast and multicast
            mc.port[o].reserved = !vc_in_use;
        }
        
        // PHASE B: Try to send flits for each output
        // FAIRNESS: Use per-entry round-robin starting from same point as Phase A
        for (int port_step = 0; port_step < DIRECTIONS + 2; port_step++) {
            int o = (start_port + port_step) % (DIRECTIONS + 2);
            if (!mc.port[o].needed || mc.port[o].done) {
                continue;  // This output doesn't need packet or already done
            }
            
            int idx = mc.port[o].next_flit_idx;
            if (idx >= (int)mc.fifo.size()) {
                continue;  // This port is caught up with the FIFO
            }
            
            Flit f = mc.fifo[idx];
            bool is_head = (f.flit_type == FLIT_TYPE_HEAD);
            bool is_tail = (f.flit_type == FLIT_TYPE_TAIL);
            
            // HEAD flit: must have reservation
            if (is_head && !mc.port[o].reserved) {
                continue;  // Cannot send HEAD without reservation
            }
            
            // Wormhole rule: can't send BODY/TAIL before HEAD
            if (!is_head && !mc.port[o].head_sent) {
                continue;
            }
            
            // Check if we can send this flit to this output
            if (has_flit[o]) {
                continue;  // Output register occupied - try next cycle
            }
            
            if (buffer_full_status_tx[o].read().mask[mc.port[o].vc]) {
                continue;  // Downstream buffer full - try next cycle
            }
            
            // CRITICAL: Re-check reservation status before sending (prevent race)
            // Check both unicast and multicast to prevent conflicts
            if (is_head || !mc.port[o].head_sent) {
                bool vc_now_in_use = false;
                
                // Check 1: Did another multicast claim this VC between Phase A and B?
                if (mc_vc_busy[o][mc.port[o].vc]) {
                    vc_now_in_use = true;
                }
                
                // Check 2: Did unicast reserve this VC between Phase A and B?
                if (!vc_now_in_use) {
                    for (int check_input = 0; check_input < DIRECTIONS + 2; check_input++) {
                        vector<pair<int,int>> reservations = reservation_table.getReservations(check_input);
                        for (auto res : reservations) {
                            if (res.first == o && res.second == mc.port[o].vc) {
                                vc_now_in_use = true;
                                break;
                            }
                        }
                        if (vc_now_in_use) break;
                    }
                }
                
                if (vc_now_in_use) {
                    // VC became busy between Phase A and Phase B - abort this output
                    mc.port[o].reserved = false;
                    continue;
                }
            }
            
            // Ready to send! Load output register
            f.vc_id = mc.port[o].vc;
            
            // REMOVED: Multi-credit logic - each response returns exactly 1 credit
            // since we now prevent duplicate PE coalescing
            
            out_reg[o] = f;
            has_flit[o] = true;
            mc.port[o].next_flit_idx++;
            
            // Advance per-entry round-robin pointer for fairness across ports
            mc.port_rr_start = (o + 1) % (DIRECTIONS + 2);
            
            // Update state
            if (is_head) {
                mc.port[o].head_sent = true;
                // Mark this VC as busy on this output port (for HEAD flit)
                // This prevents unicast from trying to reserve it
                mc_vc_busy[o][mc.port[o].vc] = true;
            }
            if (is_tail) {
                mc.port[o].done = true;
                // Free this VC on this output port (for TAIL flit)
                // This allows unicast to reserve it again
                mc_vc_busy[o][mc.port[o].vc] = false;
            }
        }
        
        // Check if all needed ports are done
        bool all_done = true;
        for (int o = 0; o < DIRECTIONS + 2; o++) {
            if (mc.port[o].needed && !mc.port[o].done) {
                all_done = false;
                break;
            }
        }
        
        // If all ports done, free the entry
        if (all_done) {
              //  cerr << "Router[" << local_id << "] McEngine[" << mc_idx << "] ALL OUTPUTS DONE - freeing entry for fid=" << mc.feature_id << endl;
            
            // Free coalescing entry
            int ce_idx = findCoalesceEntry(mc.feature_id);
            if (ce_idx >= 0) {
                freeCoalesceEntry(ce_idx);
            }
            
            // Free McEntry
            freeMcEntry(mc_idx);
            
            // Track statistics
            coalesce_responses_multicast++;
        }
    }
    
    // Advance round-robin index for next cycle (fairness across all mc entries)
    mc_rr_idx = (mc_rr_idx + 1) % MC_ENGINE_SIZE;
}

void Router::printStuckMcEntries()
{
    if (!GlobalParams::enable_selective_coalescing) return;
    
    for (int i = 0; i < MC_ENGINE_SIZE; i++) {
        if (mc_engine[i].valid) {
            McEntry &mc = mc_engine[i];
            cerr << "Router[" << local_id << "] STUCK McEntry[" << i << "] fid=" << mc.feature_id 
                 << " FIFO=" << mc.fifo.size() << " ports: ";
            for (int o = 0; o < DIRECTIONS + 2; o++) {
                if (mc.port[o].needed) {
                    cerr << o << (mc.port[o].done ? "(done)" : "(STUCK)") << " ";
                }
            }
            cerr << endl;
        }
    }
    
    // Also check for stuck flits in regular buffers
    int total_buffered = 0;
    for (int i = 0; i < DIRECTIONS + 2; i++) {
        for (int vc = 0; vc < GlobalParams::n_virtual_channels; vc++) {
            int size = buffer[i][vc].Size();
            if (size > 0) {
                total_buffered += size;
            }
        }
    }
    if (total_buffered > 0) {
        cerr << "Router[" << local_id << "] Total buffered flits: " << total_buffered << endl;
    }
}

int Router::allocateMcEntry(int feature_id)
{
    for (int i = 0; i < MC_ENGINE_SIZE; i++) {
        if (!mc_engine[i].valid) {
            mc_engine[i].valid = true;
            mc_engine[i].feature_id = feature_id;
            mc_engine[i].src_memtile = -1;
            mc_engine[i].out_ports_needed.reset();
            mc_engine[i].fifo.clear();
            mc_engine[i].port_rr_start = 0;  // Initialize per-entry round-robin port index
            for (int o = 0; o < DIRECTIONS + 2; o++) {
                mc_engine[i].port[o].needed = false;
                mc_engine[i].port[o].head_sent = false;
                mc_engine[i].port[o].done = false;
                mc_engine[i].port[o].reserved = false;
                mc_engine[i].port[o].next_flit_idx = 0;
                mc_engine[i].port[o].vc = 0;  // Default (will be overwritten with preserved VC)
            }
            return i;
        }
    }
    return -1;  // Table full
}

void Router::freeMcEntry(int mc_idx)
{
    if (mc_idx >= 0 && mc_idx < MC_ENGINE_SIZE) {
        mc_engine[mc_idx].valid = false;
        mc_engine[mc_idx].feature_id = -1;
        mc_engine[mc_idx].fifo.clear();
        
        // Remove from input tracking map
        for (auto it = input_to_mc_entry.begin(); it != input_to_mc_entry.end(); ) {
            if (it->second == mc_idx) {
                it = input_to_mc_entry.erase(it);
            } else {
                ++it;
            }
        }
    }
}

int Router::findMcEntry(int feature_id)
{
    for (int i = 0; i < MC_ENGINE_SIZE; i++) {
        if (mc_engine[i].valid && mc_engine[i].feature_id == feature_id) {
            return i;
        }
    }
    return -1;
}

// Get list of output directions where this response should be multicast
// Returns empty vector for non-multicast (unicast) responses
std::vector<int> Router::getMulticastOutputs(const Flit &f)
{
    std::vector<int> outputs;
    
    // Only handle if mechanism is enabled
    if (!GlobalParams::enable_selective_coalescing) {
        return outputs;
    }
    
    // Only process RESPONSE packets
    if (f.packet_type != PACKET_TYPE_RESPONSE) {
        return outputs;
    }
    
    // CRITICAL: Per-request multicast permission
    // This response may only multicast if the original request had coalesce_allowed=true at memory
    // Prevents responses for "late" or "table-full" requests from incorrectly multicasting
    if (!f.multicast_allowed) {
        // This specific response must NOT multicast, even if coalesce entries exist
        return outputs;  // Empty = unicast via recorded_path
    }
    
    // CRITICAL: Contiguous prefix semantics
    // - Routers WITHOUT a coalesce entry: forward as unicast (using recorded_path)
    // - Routers WITH a coalesce entry: multicast to all requester ports
    // This ensures responses are unicast through non-coalescing region, then branch at first coalescing router
    
    int entry_idx = findCoalesceEntry(f.feature_id);
    
    if (entry_idx < 0) {
        // No coalesce entry at this router - forward as UNICAST using recorded_path
        // This handles the "before first coalescing router" region
        return outputs;  // Empty = unicast routing
    }
    
    // Found coalesce entry - MULTICAST to all ports where requests came from
    CoalesceEntry &entry = coalesce_table[entry_idx];
    
    // Send response back through the ports where requests came FROM
    // requester_ports contains the INPUT ports that had requests
    // Response goes back OUT through those same ports (reverse direction)
    const int NUM_PORTS = DIRECTIONS + 2;
    
    // Debug: Show detailed PE-to-port mapping
    uint64_t cur_cycle = (uint64_t)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
   /* if (cur_cycle <= 50000) {
        int total_pes = 0;
        cerr << "Router[" << local_id << "] MULTICAST fid=" << f.feature_id << " mapping:" << endl;
        for (int port = 0; port < NUM_PORTS; port++) {
            if (entry.requester_ports.test(port)) {
                cerr << "  Port[" << port << "] -> PEs: ";
                if (entry.port_to_pe_ids.count(port) > 0) {
                    for (int pe_id : entry.port_to_pe_ids[port]) {
                        cerr << pe_id << " ";
                        total_pes++;
                    }
                }
                cerr << endl;
            }
        }
        cerr << "  Total unique PEs expecting response: " << total_pes << endl;
    }*/
    
    for (int port = 0; port < NUM_PORTS; port++) {
        if (entry.requester_ports.test(port)) {
            outputs.push_back(port);
        }
    }
    
    // NOTE: Multicast statistics are tracked in Phase 3 when actually transmitting
    // NOTE: Do NOT free entry here - this function is called every cycle while HEAD waits
    // Entry will be freed in Phase 3a when TAIL is actually transmitted
    
    return outputs;
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
    // NOTE: Multicast handling is now done in txProcess via getMulticastOutputs()
    if (f.packet_type == PACKET_TYPE_RESPONSE) {
        
        if (f.flit_type == FLIT_TYPE_TAIL ) {
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

// Clean up coalesce table entries that have been stuck for more than 600 cycles
void Router::cleanupStaleCoalesceEntries()
{
    uint64_t cur_cycle = (uint64_t)(sc_time_stamp().to_double() / GlobalParams::clock_period_ps);
    const uint64_t TIMEOUT_CYCLES = 1000;
    
    for (int i = 0; i < COALESCE_TABLE_SIZE; i++) {
        if (!coalesce_table[i].valid) {
            continue;
        }
        
        uint64_t age = cur_cycle - coalesce_table[i].allocation_cycle;
        
        if (age > TIMEOUT_CYCLES) {
            // Entry has been sitting for more than 600 cycles without a response
            // This likely means the request was dropped or response was lost
            /*cerr << "*** Router[" << local_id << "] COALESCE TIMEOUT: Freeing stale entry " << i
                 << " for fid=" << coalesce_table[i].feature_id
                 << " age=" << age << " cycles" << endl;*/
            
            freeCoalesceEntry(i);
            coalesce_entries_timed_out++;
        }
    }
}

// Print selective coalescing statistics
void Router::printSelectiveCoalescingStats() const
{
    if (!GlobalParams::enable_selective_coalescing) {
        return;  // Feature not enabled
    }
    
    cout << "Router[" << local_id << "] Selective Coalescing Stats:" << endl;
    cout << "  Eligible requests received:  " << coalesce_requests_received << endl;
    cout << "  Requests merged (dropped):   " << coalesce_requests_merged << endl;
    cout << "  Table full events:           " << coalesce_table_full_events << endl;
    cout << "  Responses multicast:         " << coalesce_responses_multicast << endl;
    cout << "  Entries timed out (>600cyc): " << coalesce_entries_timed_out << endl;
    
    if (coalesce_requests_received > 0) {
        double merge_ratio = 100.0 * coalesce_requests_merged / coalesce_requests_received;
        cout << "  Merge ratio:                 " << fixed << setprecision(1) 
             << merge_ratio << "%" << endl;
    }
    
    // Count active entries
    int active_entries = 0;
    for (int i = 0; i < COALESCE_TABLE_SIZE; i++) {
        if (coalesce_table[i].valid) {
            active_entries++;
        }
    }
    cout << "  Active table entries:        " << active_entries << " / " 
         << COALESCE_TABLE_SIZE << endl;
}