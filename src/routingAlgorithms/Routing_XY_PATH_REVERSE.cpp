#include "Routing_XY_PATH_REVERSE.h"

RoutingAlgorithmsRegister Routing_XY_PATH_REVERSE::routingAlgorithmsRegister("XY_PATH_REVERSE", getInstance());

Routing_XY_PATH_REVERSE * Routing_XY_PATH_REVERSE::routing_XY_PATH_REVERSE = 0;

Routing_XY_PATH_REVERSE * Routing_XY_PATH_REVERSE::getInstance() {
	if ( routing_XY_PATH_REVERSE == 0 )
		routing_XY_PATH_REVERSE = new Routing_XY_PATH_REVERSE();
    
	return routing_XY_PATH_REVERSE;
}

vector<int> Routing_XY_PATH_REVERSE::route(Router * router, const RouteData & routeData)
{
    vector<int> directions;
    
    // Check if this is a RESPONSE packet with a recorded path
    if (routeData.packet_type == PACKET_TYPE_RESPONSE &&
        !routeData.recorded_path.empty()) {
        
        // RESPONSE packet - use recorded path in reverse
        // Find current router in the recorded path and route to previous hop
        int current_id = routeData.current_id;
        const vector<int>& path = routeData.recorded_path;
        
        // Find our position in the path
        int next_hop_id = -1;
        for (int i = 0; i < (int)path.size(); i++) {
            if (path[i] == current_id) {
                // Found ourselves - next hop is the previous router in the path
                if (i > 0) {
                    next_hop_id = path[i - 1];
                } else {
                    // We're the first router in recorded path, destination is the src PE
                    next_hop_id = routeData.dst_id;
                }
                break;
            }
        }
        
        // If we haven't found ourselves yet, we might be beyond the recorded path
        // In this case, route to the last router in the path
        if (next_hop_id == -1 && !path.empty()) {
            next_hop_id = path.back();
        }
        
        // If still no next hop (shouldn't happen), fall through to XY
        if (next_hop_id != -1) {
            // Determine direction to next hop
            Coord current = id2Coord(current_id);
            Coord next = id2Coord(next_hop_id);
            
            if (next.x > current.x)
                directions.push_back(DIRECTION_EAST);
            else if (next.x < current.x)
                directions.push_back(DIRECTION_WEST);
            else if (next.y > current.y)
                directions.push_back(DIRECTION_SOUTH);
            else if (next.y < current.y)
                directions.push_back(DIRECTION_NORTH);
            else
                directions.push_back(DIRECTION_LOCAL);  // We've arrived
                
            return directions;
        }
    }
    
    // REQUEST packet or no recorded path - use standard XY routing
    Coord current = id2Coord(routeData.current_id);
    Coord destination = id2Coord(routeData.dst_id);

    if (destination.x > current.x)
       directions.push_back(DIRECTION_EAST);
    else if (destination.x < current.x)
        directions.push_back(DIRECTION_WEST);
    else if (destination.y > current.y)
        directions.push_back(DIRECTION_SOUTH);
    else
        directions.push_back(DIRECTION_NORTH);

    return directions;
}
