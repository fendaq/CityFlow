#include "flow/route.h"
#include "roadnet/roadnet.h"

namespace CitySimulator {
	std::vector<Road *> Route::getRoute() const{
		return route;
	}
}

