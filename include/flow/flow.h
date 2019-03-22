#ifndef CITY_SIMULATOR_FLOW_H
#define CITY_SIMULATOR_FLOW_H

#include "vehicle/vehicle.h"
#include "flow/route.h"

#include <string>
#include <list>

namespace CitySimulator {
	class Engine;

	class Flow {
	private:
		VehicleInfo vehicleTemplate;
		double interval;
		double nowTime = 0;
		double currentTime = 0;
		int startTime = 0;
		int endTime = -1;
		int cnt = 0;
		Engine &engine;
		std::string id;

	public:
		Flow(const VehicleInfo &_template, const Route &_route, double timeInterval, Engine &_engine, int startTime,
			 int endTime, const std::string &_id) : vehicleTemplate(_template), interval(timeInterval),
													startTime(startTime), endTime(endTime), engine(_engine), id(_id) {
			assert(timeInterval >= 1 || (startTime == endTime));
			vehicleTemplate.route = _route;
			nowTime = interval;
		}

		void nextStep(double timeInterval);

		std::string getId() const;

		void reset();

	};
}

#endif //CITY_SIMULATOR_FLOW_H
