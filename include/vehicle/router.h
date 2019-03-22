#ifndef CITY_SIMULATOR_ROUTER
#define CITY_SIMULATOR_ROUTER

#include <vector>
#include <random>

namespace CitySimulator {
	class Drivable;
	class Route;
	class Lane;
	class LaneLink;

	class Router {
	public:
		std::vector<Drivable *> getRoutes(const Route *route, std::mt19937 &rnd);

		Drivable * getNextDrivable(const std::vector<Drivable *> *route, double dis, std::vector<Drivable *>::iterator iDrviable, Lane * curLane, LaneLink * curLaneLink);
	};
}

#endif