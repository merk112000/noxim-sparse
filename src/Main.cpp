/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the top-level of Noxim
 */

#include "ConfigurationManager.h"
#include "NoC.h"
#include "GlobalStats.h"
#include "DataStructs.h"
#include "GlobalParams.h"

#include <csignal>

using namespace std;

// need to be globally visible to allow "-volume" simulation stop
unsigned int drained_volume;
NoC *n;

void signalHandler( int signum )
{
    cout << "\b\b  " << endl;
    cout << endl;
    cout << "Current Statistics:" << endl;
    cout << "(" << sc_time_stamp().to_double() / GlobalParams::clock_period_ps << " sim cycles executed)" << endl;
    GlobalStats gs(n);
    gs.showStats(std::cout, GlobalParams::detailed);
}

int sc_main(int arg_num, char *arg_vet[])
{
    signal(SIGQUIT, signalHandler);  

    // TEMP
    drained_volume = 0;

    // Handle command-line arguments
    cout << "\t--------------------------------------------" << endl; 
    cout << "\t\tNoxim - the NoC Simulator" << endl;
    cout << "\t\t(C) University of Catania" << endl;
    cout << "\t--------------------------------------------" << endl; 

    cout << "Catania V., Mineo A., Monteleone S., Palesi M., and Patti D. (2016) Cycle-Accurate Network on Chip Simulation with Noxim. ACM Trans. Model. Comput. Simul. 27, 1, Article 4 (August 2016), 25 pages. DOI: https://doi.org/10.1145/2953878" << endl;
    cout << endl;
    cout << endl;

    configure(arg_num, arg_vet);


    // Signals
    sc_clock clock("clock", GlobalParams::clock_period_ps, SC_PS);
    sc_signal <bool> reset;

    // NoC instance
    n = new NoC("NoC");

    n->clock(clock);
    n->reset(reset);

    // Trace signals
    sc_trace_file *tf = NULL;
    if (GlobalParams::trace_mode) {
	tf = sc_create_vcd_trace_file(GlobalParams::trace_filename.c_str());
	sc_trace(tf, reset, "reset");
	sc_trace(tf, clock, "clock");

	for (int i = 0; i < GlobalParams::mesh_dim_x; i++) {
	    for (int j = 0; j < GlobalParams::mesh_dim_y; j++) {
		char label[64];

		sprintf(label, "req(%02d)(%02d).east", i, j);
		sc_trace(tf, n->req[i][j].east, label);
		sprintf(label, "req(%02d)(%02d).west", i, j);
		sc_trace(tf, n->req[i][j].west, label);
		sprintf(label, "req(%02d)(%02d).south", i, j);
		sc_trace(tf, n->req[i][j].south, label);
		sprintf(label, "req(%02d)(%02d).north", i, j);
		sc_trace(tf, n->req[i][j].north, label);

		sprintf(label, "ack(%02d)(%02d).east", i, j);
		sc_trace(tf, n->ack[i][j].east, label);
		sprintf(label, "ack(%02d)(%02d).west", i, j);
		sc_trace(tf, n->ack[i][j].west, label);
		sprintf(label, "ack(%02d)(%02d).south", i, j);
		sc_trace(tf, n->ack[i][j].south, label);
		sprintf(label, "ack(%02d)(%02d).north", i, j);
		sc_trace(tf, n->ack[i][j].north, label);
	    }
	}
    }
    // Reset the chip and run the simulation
    reset.write(1);
    cout << "Reset for " << (int)(GlobalParams::reset_time) << " cycles... ";
    srand(GlobalParams::rnd_generator_seed);

    // fix clock periods different from 1ns
    //sc_start(GlobalParams::reset_time, SC_NS);
    sc_start(GlobalParams::reset_time * GlobalParams::clock_period_ps, SC_PS);

    reset.write(0);
    cout << " done! " << endl;
    cout << " Now running for " << GlobalParams:: simulation_time << " cycles..." << endl;
    
    // Run simulation with heartbeat every 100k cycles
    uint64_t heartbeat_interval = 10000;
    uint64_t stoppage_check_interval = 10;
    uint64_t total_cycles = GlobalParams::simulation_time;
    uint64_t cycles_run = 0;
    
    while (cycles_run < total_cycles) {
        uint64_t cycles_to_run = min(stoppage_check_interval, total_cycles - cycles_run);
        sc_start((double)(cycles_to_run * GlobalParams::clock_period_ps), SC_PS);
        cycles_run += cycles_to_run;
        
        // Print heartbeat statistics
        if (cycles_run % heartbeat_interval == 0) {
            cout << "\n========== Heartbeat @ " << cycles_run << " cycles ==========" << endl;
            
            // Show PE statistics
            for (int y = 0; y < GlobalParams::mesh_dim_y; y++) {
                for (int x = 0; x < GlobalParams::mesh_dim_x; x++) {
                    Coord coord;
                    coord.x = x;
                    coord.y = y;
                    int id = coord2Id(coord);
                    n->t[x][y]->pe->printHeartbeat(id, cycles_run);
                }
            }
           // cout << "========================================\n" << endl;
        }
        
        // Check for early termination every 100 cycles: all trace events sent and in-flight < 10
        if (n->allTraceEventsCompleted(32)) {
            cout << "\n*** EARLY TERMINATION: All trace events sent and in-flight requests < 10 ***" << endl;
            cout << "    Stopped at cycle " << cycles_run << " (configured: " << total_cycles << ")" << endl;
            break;
        }
    }

    // Close the simulation
    if (GlobalParams::trace_mode) sc_close_vcd_trace_file(tf);
    cout << "Noxim simulation completed.";
    cout << " (" << sc_time_stamp().to_double() / GlobalParams::clock_period_ps << " cycles executed)" << endl;
    cout << endl;
    
    // DEBUG: Print stuck McEngine entries before showing stats
    cerr << "\n=== Checking for stuck McEngine entries ===" << endl;
    for (int y = 0; y < GlobalParams::mesh_dim_y; y++) {
        for (int x = 0; x < GlobalParams::mesh_dim_x; x++) {
            n->t[x][y]->r->printStuckMcEntries();
        }
    }
    cerr << "===========================================\n" << endl;
    
//assert(false);
    // Show statistics
    GlobalStats gs(n);
    gs.showStats(std::cout, GlobalParams::detailed);
    
    // Show memory tile statistics
    cout << endl << "Memory Tile Statistics:" << endl;
    for (int y = 0; y < GlobalParams::mesh_dim_y; y++) {
        for (int x = 0; x < GlobalParams::mesh_dim_x; x++) {
            n->t[x][y]->pe->printMemoryStats();
        }
    }

    // Show end-to-end latency statistics for compute PEs
    cout << endl << "End-to-End Latency Statistics (REQUEST to RESPONSE):" << endl;
    for (int y = 0; y < GlobalParams::mesh_dim_y; y++) {
        for (int x = 0; x < GlobalParams::mesh_dim_x; x++) {
            n->t[x][y]->pe->printE2ELatencyStats();
        }
    }
    
    // Show stall breakdown statistics for compute PEs
    cout << endl << "Stall Breakdown Statistics (Memory vs NoC):" << endl;
    for (int y = 0; y < GlobalParams::mesh_dim_y; y++) {
        for (int x = 0; x < GlobalParams::mesh_dim_x; x++) {
            n->t[x][y]->pe->printStallStats();
        }
    }
    
    // Show timeout statistics for compute PEs (credits returned for missing responses)
   /* cout << endl << "Timeout Statistics (Missing Responses - Credits Returned):" << endl;
    for (int y = 0; y < GlobalParams::mesh_dim_y; y++) {
        for (int x = 0; x < GlobalParams::mesh_dim_x; x++) {
            n->t[x][y]->pe->printTimeoutStats();
        }
    }*/
    
    // Show router link utilization for memory tile routers
    cout << endl << "Memory Tile Router Link Utilization:" << endl;
    vector<int> memory_tile_ids = {1, 2, 3, 5, 9, 10, 14, 15, 19, 21, 22, 23};
    for (int tile_id : memory_tile_ids) {
        if (tile_id < GlobalParams::mesh_dim_x * GlobalParams::mesh_dim_y) {
            int x = tile_id % GlobalParams::mesh_dim_x;
            int y = tile_id / GlobalParams::mesh_dim_x;
            n->t[x][y]->r->printLinkUtilization();
            cout << endl;
        }
    }
    
    // Show router link utilization (focus on interior routers and hotspots)
    cout << endl << "Router Link Utilization (Interior and Critical Routers):" << endl;
    // Print routers in the center (high traffic convergence points)
    vector<pair<int,int>> critical_routers = {{1,1}, {1,2}, {1,3}, {2,1}, {2,2}, {2,3}, {3,1}, {3,2}, {3,3}};
    for (auto coord : critical_routers) {
        int x = coord.first;
        int y = coord.second;
        if (x < GlobalParams::mesh_dim_x && y < GlobalParams::mesh_dim_y) {
            n->t[x][y]->r->printLinkUtilization();
            cout << endl;
        }
    }
    
    // Show selective coalescing statistics (if enabled)
    if (GlobalParams::enable_selective_coalescing) {
        cout << endl << "===========================================================" << endl;
        cout << "Selective In-Router Coalescing Statistics" << endl;
        cout << "===========================================================" << endl;
        for (int y = 0; y < GlobalParams::mesh_dim_y; y++) {
            for (int x = 0; x < GlobalParams::mesh_dim_x; x++) {
                n->t[x][y]->r->printSelectiveCoalescingStats();
                cout << endl;
            }
        }
    }
    
    // Show oracle coalescing statistics for all routers
    cout << endl << "===========================================================" << endl;
    cout << "Oracle Coalescing Statistics (Request Deduplication Potential)" << endl;
    cout << "===========================================================" << endl;
    for (int y = 0; y < GlobalParams::mesh_dim_y; y++) {
        for (int x = 0; x < GlobalParams::mesh_dim_x; x++) {
            n->t[x][y]->r->printOracleCoalescingStats();
            cout << endl;
        }
    }


    if ((GlobalParams::max_volume_to_be_drained > 0) &&
	(sc_time_stamp().to_double() / GlobalParams::clock_period_ps - GlobalParams::reset_time >=
	 GlobalParams::simulation_time)) {
	cout << endl
         << "WARNING! the number of flits specified with -volume option" << endl
	     << "has not been reached. ( " << drained_volume << " instead of " << GlobalParams::max_volume_to_be_drained << " )" << endl
         << "You might want to try an higher value of simulation cycles" << endl
	     << "using -sim option." << endl;

#ifdef TESTING
	cout << endl
         << " Sum of local drained flits: " << gs.drained_total << endl
	     << endl
         << " Effective drained volume: " << drained_volume;
#endif

    }

#ifdef DEADLOCK_AVOIDANCE
	cout << "***** WARNING: DEADLOCK_AVOIDANCE ENABLED!" << endl;
#endif
    return 0;
}
