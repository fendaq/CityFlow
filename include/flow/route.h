#ifndef CITY_SIMULATOR_ROUTE_H
#define CITY_SIMULATOR_ROUTE_H

#include <vector>

namespace CitySimulator {
	class Road;

	class Route {
	private:
		std::vector<Road *> route;
	public:
		explicit Route() = default;

		explicit Route(const std::vector<Road *> &_route) : route(_route) { }

		std::vector<Road *> getRoute() const;
	};
}
#endif //CITY_SIMULATOR_ROUTE_H
