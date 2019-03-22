#ifndef CITY_SIMULATOR_ENGINE_H
#define CITY_SIMULATOR_ENGINE_H

#include "utility/json/json.h"
#include "flow/flow.h"
#include "vehicle/vehicle.h"
#include "roadnet/roadnet.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/thread/thread.hpp>
#include <boost/ref.hpp>

#include <utility>
#include <vector>
#include <map>
#include <set>
#include <random>
#include <string>
#include <fstream>


namespace CitySimulator {

    class Engine {
    private:
        static bool vehicleCmp(const std::pair<Vehicle *, double> &a, const std::pair<Vehicle *, double> &b) {
            return a.second < b.second;
        }

        std::map<int, std::pair<Vehicle *, int>> vehiclePool;
        std::vector<std::set<Vehicle *>> threadVehiclePool;
        std::vector<std::set<Road *>> threadRoadPool;
        std::vector<std::set<Intersection *>> threadIntersectionPool;
        std::vector<Flow> flows;
        RoadNet roadNet;
        int threadNum;
        double interval;
        bool saveReplay;
        bool warnings;
        std::vector<std::pair<Vehicle *, double>> pushBuffer;
        Json::Value jsonRoot;
        std::string stepLog;

        size_t step = 0;
        size_t activeVehicleCount = 0;
        boost::mutex lock;
        boost::barrier startBarrier, endBarrier;
        std::vector<boost::thread> threadPool;
        bool finished = false;
        std::ofstream logOut;

        bool rlTrafficLight;
        bool laneChange;
        int manuallyPushCnt = 0;

        std::list<std::pair<Vehicle *, ControlInfo>> shadowBuffer;
        std::list<Vehicle *> finishLaneChangeBuffer;

    private:
        void vehicleControl(Vehicle &vehicle, std::vector<std::pair<Vehicle *, double>> &buffer);

        void applyAction();

        void updateAction();

        void updateLocation();

        void setLeader();

        void threadController(std::set<Vehicle *> &vehicles, std::set<Road *> &roads,
                              std::set<Intersection *> &intersections);

        void threadGetAction(std::set<Vehicle *> &vehicles);

        void threadUpdate(std::set<Vehicle *> &vehicles);

        void threadSetLeader(std::set<Road *> &roads, std::set<Intersection *> &intersections);

        void threadUpdateLocation(std::set<Road *> &roads, std::set<Intersection *> &intersections);

        void threadNotifyCross(std::set<Intersection *> &intersections);

        void threadInitSegments(std::set<Road *> &roads);

        void handleWaiting();

        void handleLaneChange(Vehicle &vehicle, const ControlInfo &controlInfo);

        void updateLog();

        void pushShadow();

        void finishLaneChange();

        bool checkWarning();

        bool loadRoadNet(const std::string &jsonFile);

        bool loadFlow(const std::string &jsonFilename);

        std::vector<Vehicle*> getRunningVehicle() const;

    public:
        std::mt19937 rnd;

        Engine(const std::string &configFile, int threadNum);

        double getInterval() const { return interval; }

        bool hasLaneChange() const { return laneChange; }

        bool loadConfig(const std::string &configFile);

        void notifyCross();

        void nextStep();

        bool checkPriority(int priority);

        void pushVehicle(Vehicle *const vehicle);

        void setLogFile(const std::string &jsonFile, const std::string &logFile);

        void initSegments();

        ~Engine();

        // RL related api

        void pushVehicle(const std::map<std::string, double> &info, const std::vector<std::string> &roads);

        size_t getVehicleCount() const;

        std::map<std::string, int> getLaneVehicleCount() const;

        std::map<std::string, int> getLaneWaitingVehicleCount() const;

        std::map<std::string, std::vector<std::string>> getLaneVehicles();

        std::map<std::string, double> getVehicleSpeed() const;

        std::map<std::string, double> getVehicleDistance() const;

        double getCurrentTime() const;

        void setTrafficLightPhase(const std::string &id, int phaseIndex);

        void reset();
    };

}

#endif //CITY_SIMULATOR_ENGINE_H
