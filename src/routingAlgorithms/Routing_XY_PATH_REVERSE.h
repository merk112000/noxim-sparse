#ifndef __NOXIMROUTING_XY_PATH_REVERSE_H__
#define __NOXIMROUTING_XY_PATH_REVERSE_H__

#include "RoutingAlgorithm.h"
#include "RoutingAlgorithms.h"
#include "../Router.h"

using namespace std;

class Routing_XY_PATH_REVERSE : RoutingAlgorithm {
	public:
		vector<int> route(Router * router, const RouteData & routeData);

		static Routing_XY_PATH_REVERSE * getInstance();

	private:
		Routing_XY_PATH_REVERSE(){};
		~Routing_XY_PATH_REVERSE(){};

		static Routing_XY_PATH_REVERSE * routing_XY_PATH_REVERSE;
		static RoutingAlgorithmsRegister routingAlgorithmsRegister;
};

#endif
