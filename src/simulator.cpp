#include "engine/engine.h"

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace py = pybind11;
using namespace py::literals;

PYBIND11_MODULE(cityflow, m) {
    py::class_<CitySimulator::Engine>(m, "Engine")
        .def(py::init<const std::string&, int>(),
            "config_file"_a,
            "thread_num"_a=1
        )
        .def("next_step", &CitySimulator::Engine::nextStep)
        .def("get_vehicle_count", &CitySimulator::Engine::getVehicleCount)
        .def("get_lane_vehicle_count", &CitySimulator::Engine::getLaneVehicleCount)
        .def("get_lane_waiting_vehicle_count", &CitySimulator::Engine::getLaneWaitingVehicleCount)
		.def("get_lane_vehicles", &CitySimulator::Engine::getLaneVehicles)
        .def("get_vehicle_speed", &CitySimulator::Engine::getVehicleSpeed)
        .def("get_vehicle_distance", &CitySimulator::Engine::getVehicleDistance)
        .def("get_current_time", &CitySimulator::Engine::getCurrentTime)
        .def("set_tl_phase", &CitySimulator::Engine::setTrafficLightPhase)
        .def("push_vehicle", (void (CitySimulator::Engine::*)(const std::map<std::string, double>&, const std::vector<std::string>&)) &CitySimulator::Engine::pushVehicle)
        .def("reset", &CitySimulator::Engine::reset);
}