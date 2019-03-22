#include "engine/engine.h"
#include "utility/json/json.h"
#include "utility/utility.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <fstream>
#include <iostream>

namespace CitySimulator {

    Engine::Engine(const std::string &configFile, int threadNum): threadNum(threadNum), startBarrier(threadNum+1), endBarrier(threadNum+1)
    {
        for (int i = 0; i < threadNum; i++) {
            threadVehiclePool.emplace_back();
            threadRoadPool.emplace_back();
            threadIntersectionPool.emplace_back();
        }

        for (int i = 0; i < threadNum; i++)
            threadPool.emplace_back(&Engine::threadController, this, boost::ref(threadVehiclePool[i]),
                                    boost::ref(threadRoadPool[i]), boost::ref(threadIntersectionPool[i]));
        
        bool success = loadConfig(configFile);
        if (!success) {
            std::cerr << "load config failed!" << std::endl;
        }
    }
    

    bool Engine::loadConfig(const std::string &configFile) {
        Json::Value root;

        std::ifstream jsonfile(configFile, std::ifstream::binary);
        if (!jsonfile.is_open()) {
            std::cerr << "cannot open config file!" << std::endl;
            return false;
        }
        jsonfile >> root;
        jsonfile.close();

        interval = root["interval"].asDouble();
        warnings = root["warning"].asBool();
        rlTrafficLight = root["rlTrafficLight"].asBool();
        laneChange = root["laneChange"].asBool();

        int seed = root["seed"].asInt();
        rnd.seed(seed);
        
        std::string dir = root["dir"].asString();
        std::string roadnetFile = root["roadnetFile"].asString();
        std::string flowFile = root["flowFile"].asString();
        if (!loadRoadNet(dir + roadnetFile)) {
            std::cerr << "loading roadnet file error!" << std::endl;
            return false;
        }
        if (!loadFlow(dir + flowFile)) {
            std::cerr << "loading flow file error!" << std::endl;
            return false;
        }
        
        if (warnings) checkWarning();

        saveReplay = root["saveReplay"].asBool();
        if (saveReplay) {
            setLogFile(dir + root["roadnetLogFile"].asString(), dir + root["replayLogFile"].asString());
        }

        stepLog = "";
        return true;
    }

    bool Engine::loadRoadNet(const std::string &jsonFile) {
        bool ans = roadNet.loadFromJson(jsonFile);
        int cnt = 0;
        for (auto &road : roadNet.getRoads()) {
            threadRoadPool[cnt].emplace(&road);
            cnt = (cnt + 1) % threadNum;
        }
        for (auto &intersection : roadNet.getIntersections()) {
            threadIntersectionPool[cnt].emplace(&intersection);
            cnt = (cnt + 1) % threadNum;
        }
        jsonRoot["static"] = roadNet.convertToJson();
        return ans;
    }

    bool Engine::loadFlow(const std::string &jsonFilename) {
        Json::Value root;

        std::ifstream jsonFile(jsonFilename, std::ifstream::binary);
        if (!jsonFile.is_open()) {
            std::cerr << "cannot open flow file" << std::endl;
            return false;
        }
        jsonFile >> root;
        jsonFile.close();

        for (int i = 0; i < (int) root.size(); i++) {
            Json::Value &flow = root[i];
            std::vector<Road *> roads;
            Json::Value routes = flow["route"];
            roads.reserve(routes.size());
            for (auto &route: routes)
                roads.push_back(roadNet.getRoadById(route.asString()));
            Route route(roads);

            VehicleInfo vehicleTemplate;
            Json::Value vehicle = flow["vehicle"];
            vehicleTemplate.len = vehicle["length"].asDouble();
            vehicleTemplate.width = vehicle["width"].asDouble();
            vehicleTemplate.maxPosAcc = vehicle["maxPosAcc"].asDouble();
            vehicleTemplate.maxNegAcc = vehicle["maxNegAcc"].asDouble();
            vehicleTemplate.usualPosAcc = vehicle["usualPosAcc"].asDouble();
            vehicleTemplate.usualNegAcc = vehicle["usualNegAcc"].asDouble();
            vehicleTemplate.minGap = vehicle["minGap"].asDouble();
            vehicleTemplate.maxSpeed = vehicle["maxSpeed"].asDouble();
            vehicleTemplate.headwayTime = vehicle["headwayTime"].asDouble();
            vehicleTemplate.dis = 0;
            int startTime = flow.isMember("startTime") ? flow["startTime"].asInt() : 0;
            int endTime = flow.isMember("endTime") ? flow["endTime"].asInt() : -1;
            Flow newFlow(vehicleTemplate, route, flow["interval"].asDouble(), *this, startTime, endTime,
                         "flow_" + std::to_string(i));
            flows.push_back(newFlow);
        }
        return true;
    }

    bool Engine::checkWarning() {
        bool result = true;

        if (interval < 0.2 || interval > 1.5) {
            std::cerr << "Deprecated time interval, recommended interval between 0.2 and 1.5" << std::endl;
            result = false;
        }

        for (auto &road : roadNet.getRoads())
            for (auto &lane : road.getLanes())
                if (lane.getLength() < 50) {
                    std::cerr << "Deprecated road length, recommended road length at least 50 meters" << std::endl;
                    result = false;
                }

        for (auto &road : roadNet.getRoads())
            for (auto &lane : road.getLanes())
                if (lane.getMaxSpeed() > 30) {
                    std::cerr << "Deprecated road max speed, recommended max speed at most 30 meters/s" << std::endl;
                    result = false;
                }

        return result;
    }

    void Engine::vehicleControl(Vehicle &vehicle, std::vector<std::pair<Vehicle *, double>> &buffer) {
        ControlInfo controlInfo = vehicle.getNextSpeed(interval);
        double nextSpeed = controlInfo.speed;
        double deltaDis, speed = vehicle.getSpeed();
        if (nextSpeed < 0) {
            deltaDis = 0.5 * speed * speed / vehicle.getMaxNegAcc();
            nextSpeed = 0;
        } else {
            deltaDis = (speed + nextSpeed) * interval / 2;
        }
        vehicle.setSpeed(nextSpeed);
        vehicle.setDeltaDistance(deltaDis);
        if (vehicle.hasPartner()) {
            vehicle.getPartner()->setSpeed(nextSpeed);
            vehicle.getPartner()->setDeltaDistance(deltaDis);
        }
        if (!vehicle.hasSetEnd() && vehicle.getChangedDrivable() != nullptr) {
            if (vehicle.isJump()) {
                buffer.emplace_back(&vehicle,
                                    vehicle.getDistance() + deltaDis - vehicle.getCurDrivable()->getLength() -
                                    vehicle.getJumpDistance());
                vehicle.resetJump();
            } else
                buffer.emplace_back(&vehicle, vehicle.getDistance() + deltaDis - vehicle.getCurDrivable()->getLength());
        }
        if (laneChange) handleLaneChange(vehicle, controlInfo);
    }

    void Engine::threadUpdateLocation(std::set<CitySimulator::Road *> &roads,
                                      std::set<CitySimulator::Intersection *> &intersections) {
        startBarrier.wait();
        for (auto &road : roads)
            for (auto &lane : road->getLanes()) {
                while (!lane.getVehicles().empty()) {
                    Lane o = lane;
                    Vehicle *vehicle = lane.getFirstVehicle();
                    bool check = false;
                    if ((vehicle->getChangedDrivable()) != nullptr || vehicle->hasSetEnd()) {
                        lane.popVehicle();
                        check = true;
                    }
                    if (vehicle->hasSetEnd()) {
                        boost::lock_guard<boost::mutex> guard(lock);
                        auto iter = vehiclePool.find(vehicle->getPriority());
                        threadVehiclePool[iter->second.second].erase(vehicle);
                        delete vehicle;
                        iter = vehiclePool.erase(iter);
                        activeVehicleCount--;
                    } else if (!check)
                        break;

                }
            }
        for (auto &intersection : intersections)
            for (auto &roadLink: intersection->getRoadLinks())
                for (auto &laneLink : roadLink.getLaneLinks()) {
                    while (!laneLink.getVehicles().empty()) {
                        LaneLink o = laneLink;
                        Vehicle *vehicle = laneLink.getFirstVehicle();
                        bool check = false;
                        if ((vehicle->getChangedDrivable()) != nullptr || vehicle->hasSetEnd()) {
                            laneLink.popVehicle();
                            check = true;
                        }
                        if (vehicle->hasSetEnd()) {
                            boost::lock_guard<boost::mutex> guard(lock);
                            auto iter = vehiclePool.find(vehicle->getPriority());
                            threadVehiclePool[iter->second.second].erase(vehicle);
                            delete vehicle;
                            iter = vehiclePool.erase(iter);
                            activeVehicleCount--;
                        } else if (!check)
                            break;

                    }
                }
        endBarrier.wait();
    }

    void Engine::threadController(std::set<Vehicle *> &vehicles, std::set<Road *> &roads,
                                  std::set<Intersection *> &intersections) {
        while (!finished) {
            threadNotifyCross(intersections);
            if (laneChange) threadInitSegments(roads);
            threadGetAction(vehicles);
            threadUpdateLocation(roads, intersections);
            threadSetLeader(roads, intersections);
            threadUpdate(vehicles);
        }
    }

    void Engine::threadNotifyCross(std::set<Intersection *> &intersections) {
        //TODO: iterator for laneLink
        startBarrier.wait();
        for (auto &intersection : intersections)
            for (auto &roadLink : intersection->getRoadLinks())
                for (auto &laneLink : roadLink.getLaneLinks())
                    for (auto &cross : laneLink.getCrosses())
                        cross->clearNotify();

        for (auto &intersection : intersections)
            for (auto &roadLink : intersection->getRoadLinks())
                for (auto &laneLink : roadLink.getLaneLinks()) {
                    // XXX: no cross in laneLink?
                    auto crosses = laneLink.getCrosses();
                    auto rIter = crosses.rbegin();

                    // first check the vehicle on the end lane
                    Vehicle *vehicle = laneLink.getEndLane()->getLastVehicle();
                    if (vehicle && (LaneLink *) (vehicle->getPrevDrivable()) == &laneLink) {
                        double vehDistance = vehicle->getDistance() - vehicle->getLen();
                        while (rIter != crosses.rend()) {
                            double crossDistance = laneLink.getLength() - (*rIter)->getDistanceByLane(&laneLink);
                            if (crossDistance + vehDistance < (*rIter)->getLeaveDistance()) {
                                (*rIter)->notify(&laneLink, vehicle, -(vehicle->getDistance() + crossDistance));
                                ++rIter;
                            } else break;
                        }
                    }

                    // check each vehicle on laneLink
                    for (auto &linkVehicle : laneLink.getVehicles()) {
                        double vehDistance = linkVehicle->getDistance();

                        while (rIter != crosses.rend()) {
                            double crossDistance = (*rIter)->getDistanceByLane(&laneLink);
                            if (vehDistance > crossDistance) {
                                if (vehDistance - crossDistance - linkVehicle->getLen() <=
                                    (*rIter)->getLeaveDistance()) {
                                    (*rIter)->notify(&laneLink, linkVehicle, crossDistance - vehDistance);
                                } else break;
                            } else {
                                (*rIter)->notify(&laneLink, linkVehicle, crossDistance - vehDistance);
                            }
                            ++rIter;
                        }
                    }

                    // check vehicle on the incoming lane
                    vehicle = laneLink.getStartLane()->getFirstVehicle();
                    if (vehicle && (LaneLink *) (vehicle->getNextDrivable()) == &laneLink && laneLink.isAvailable()) {
                        double vehDistance = laneLink.getStartLane()->getLength() - vehicle->getDistance();
                        while (rIter != crosses.rend()) {
                            (*rIter)->notify(&laneLink, vehicle, vehDistance + (*rIter)->getDistanceByLane(&laneLink));
                            ++rIter;
                        }
                    }
                }
        endBarrier.wait();
    }

    void Engine::threadInitSegments(std::set<Road *> &roads) {
        startBarrier.wait();
        for (auto &road : roads) {
            for (auto &lane : road->getLanes()) {
                lane.initSegments();
            }
        }
        endBarrier.wait();
    }


    void Engine::threadGetAction(std::set<Vehicle *> &vehicles) {
        std::vector<std::pair<Vehicle *, double>> buffer;
        startBarrier.wait();
        for (auto vehicle: vehicles)
            if (vehicle->isRunning()) vehicleControl(*vehicle, buffer);
        {
            boost::lock_guard<boost::mutex> guard(lock);
            pushBuffer.insert(pushBuffer.end(), buffer.begin(), buffer.end());
        }
        endBarrier.wait();
    }

    void Engine::threadUpdate(std::set<Vehicle *> &vehicles) {
        startBarrier.wait();
        for (auto vehicle: vehicles)
            if (vehicle->isRunning()) vehicle->update();
        endBarrier.wait();
    }

    void Engine::threadSetLeader(std::set<Road *> &roads, std::set<Intersection *> &intersections) {
        startBarrier.wait();
        for (auto &road : roads)
            for (auto &lane : road->getLanes()) {
                Vehicle *leader = nullptr;
                for (auto &vehicle : lane.getVehicles()) {
                    vehicle->setLeader(leader);
                    leader = vehicle;
                }
            }
        for (auto &intersection : intersections)
            for (auto &roadLink : intersection->getRoadLinks())
                for (auto &laneLink : roadLink.getLaneLinks()) {
                    Vehicle *leader = nullptr;
                    for (auto &vehicle : laneLink.getVehicles()) {
                        vehicle->setLeader(leader);
                        leader = vehicle;
                    }
                }
        endBarrier.wait();
    }

    void Engine::applyAction() {
        startBarrier.wait();
        endBarrier.wait();
    }

    void Engine::updateLocation() {
        startBarrier.wait();
        endBarrier.wait();
        std::sort(pushBuffer.begin(), pushBuffer.end(), vehicleCmp);
        for (auto &vehicle_pair : pushBuffer) {
            Vehicle *vehicle = vehicle_pair.first;
            Drivable *drivable = vehicle->getChangedDrivable();
            if (drivable != nullptr) {
                drivable->pushVehicle(vehicle);
                if (drivable->isLaneLink()) {
                    vehicle->setEnterLaneLinkTime(step);
                } else {
                    vehicle->setEnterLaneLinkTime(std::numeric_limits<int>::max());
                }
            }
        }
        pushBuffer.clear();
        if (laneChange) {
            pushShadow();
            finishLaneChange();
        }
    }

    void Engine::updateAction() {
        startBarrier.wait();
        endBarrier.wait();
    }

    void Engine::handleWaiting() {
        for (auto &road : roadNet.getRoads())
            for (auto &lane : road.getLanes()) {
                auto &buffer = lane.getWaitingBuffer();
                if (buffer.empty()) continue;
                auto &vehicle = buffer.front();
                if (lane.available(vehicle)) {
                    vehicle->setRunning(true);
                    activeVehicleCount += 1;
                    lane.pushVehicle(vehicle);
                    buffer.pop_front();
                }
            }
    }

    void Engine::updateLog() {
        std::string result;
        for (auto &vehicle: getRunningVehicle()) {
//        for (auto &vehicle_pair: vehiclePool) {
//            auto &vehicle = vehicle_pair.second.first;
            if (!vehicle->isRunning() || vehicle->isEnd() || (vehicle->hasPartner() && !vehicle->isReal()))
                continue;
            Point pos = vehicle->getPoint();
            Point dir = vehicle->getCurDrivable()->getDirectionByDistance(vehicle->getDistance());

            result.append(
                    double2string(pos.x) + " " + double2string(pos.y) + " " + double2string(atan2(dir.y, dir.x)) + ",");
        }
        result.append(";");

        for (Road &road : roadNet.getRoads()) {
            if (road.getEndIntersection().isVirtualIntersection())
                continue;
            result.append(road.getId());
            for (Lane &lane : road.getLanes()) {
                bool can_go = true;
                for (LaneLink *laneLink : lane.getLaneLinks()) {
                    if (!laneLink->isAvailable()) {
                        can_go = false;
                        break;
                    }
                }
                result.append(can_go ? " g" : " r");
            }
            result.append(",");
        }
        logOut << result << std::endl;
    }

    void Engine::setLeader() {
        startBarrier.wait();
        endBarrier.wait();
    }

    void Engine::notifyCross() {
        startBarrier.wait();
        endBarrier.wait();
    }

    void Engine::nextStep() {
        for (auto &flow : flows)
            flow.nextStep(interval);
        handleWaiting();
        notifyCross();
        if (laneChange) initSegments();
        applyAction();
        updateLocation();
        setLeader();
        updateAction();

        if (!rlTrafficLight) {
            std::vector<Intersection> &intersections = roadNet.getIntersections();
            for (auto &intersection : intersections)
                intersection.getTrafficLight().passTime(interval);
        }

        if (saveReplay) {
            updateLog();
        }

        step += 1;
    }

    void Engine::initSegments() {
        startBarrier.wait();
        endBarrier.wait();
    }

    bool Engine::checkPriority(int priority) {
        return vehiclePool.find(priority) != vehiclePool.end();
    }

    void Engine::pushVehicle(Vehicle *const vehicle) {
        size_t threadIndex = rnd() % threadNum;
        vehiclePool.emplace(vehicle->getPriority(), std::make_pair(vehicle, threadIndex));
        threadVehiclePool[threadIndex].insert(vehicle);
        ((Lane *) vehicle->getCurDrivable())->pushWaitingVehicle(vehicle);
    }

    size_t Engine::getVehicleCount() const {
        return activeVehicleCount;
    }

    std::map<std::string, int> Engine::getLaneVehicleCount() const {
        std::map<std::string, int> ret;
        for (const Road &road : roadNet.getRoads()) {
            for (const Lane &lane : road.getLanes()) {
                ret.emplace(lane.getId(), lane.getVehicleCount());
            }
        }
        return ret;
    }

    std::map<std::string, int> Engine::getLaneWaitingVehicleCount() const {
        std::map<std::string, int> ret;
        for (const Road &road : roadNet.getRoads()) {
            for (const Lane &lane : road.getLanes()) {
                int cnt = 0;
                for (auto &vehicle : lane.getVehicles()) {
                    if (vehicle->getSpeed() < 0.1) { //TODO: better waiting critera
                        cnt += 1;
                    }
                }
                ret.emplace(lane.getId(), cnt);
            }
        }
        return ret;
    }

    std::map<std::string, std::vector<std::string>> Engine::getLaneVehicles() {
        std::map<std::string, std::vector<std::string>> ret;
        for (Road &road : roadNet.getRoads()) {
            for (Lane &lane : road.getLanes()) {
                std::vector<std::string> vehicles;
                for (auto &vehicle : lane.getVehicles()) {
                    vehicles.push_back(vehicle->getId());
                }
                ret.emplace(lane.getId(), vehicles);
            }
        }
        return ret;
    }

    std::map<std::string, double> Engine::getVehicleSpeed() const {
        std::map<std::string, double> ret;
        for (auto &vehicle_pair: vehiclePool) {
            auto &vehicle = vehicle_pair.second.first;
            if (!vehicle->isRunning()) continue;
            ret.emplace(vehicle->getId(), vehicle->getSpeed());
        }
        return ret;
    }

    std::map<std::string, double> Engine::getVehicleDistance() const {
        std::map<std::string, double> ret;
        for (auto &vehicle_pair: vehiclePool) {
            auto &vehicle = vehicle_pair.second.first;
            if (!vehicle->isRunning()) continue;
            ret.emplace(vehicle->getId(), vehicle->getDistance());
        }
        return ret;
    }

    double Engine::getCurrentTime() const {
        return step * interval;
    }

    void Engine::pushVehicle(const std::map<std::string, double> &info, const std::vector<std::string> &roads) {
        VehicleInfo vehicleTemplate;
        std::map<std::string, double>::const_iterator it;
        if ((it = info.find("speed")) != info.end()) vehicleTemplate.speed = it->second;
        if ((it = info.find("length")) != info.end()) vehicleTemplate.len = it->second;
        if ((it = info.find("width")) != info.end()) vehicleTemplate.width = it->second;
        if ((it = info.find("maxPosAcc")) != info.end()) vehicleTemplate.maxPosAcc = it->second;
        if ((it = info.find("maxNegAcc")) != info.end()) vehicleTemplate.maxNegAcc = it->second;
        if ((it = info.find("usualPosAcc")) != info.end()) vehicleTemplate.usualPosAcc = it->second;
        if ((it = info.find("usualNegAcc")) != info.end()) vehicleTemplate.usualNegAcc = it->second;
        if ((it = info.find("minGap")) != info.end()) vehicleTemplate.minGap = it->second;
        if ((it = info.find("maxSpeed")) != info.end()) vehicleTemplate.maxSpeed = it->second;
        if ((it = info.find("headwayTime")) != info.end()) vehicleTemplate.headwayTime = it->second;

        std::vector<Road *> routes;
        routes.reserve(roads.size());
        for (auto &road: roads) routes.emplace_back(roadNet.getRoadById(road));
        Route route(routes);
        vehicleTemplate.route = route;

        Vehicle *vehicle = new Vehicle(vehicleTemplate, rnd(), "manually_pushed_" + std::to_string(manuallyPushCnt++),
                                       *this);
        int priority = vehicle->getPriority();
        while (checkPriority(priority)) priority = rnd();
        vehicle->setPriority(priority);
        pushVehicle(vehicle);
    }

    void Engine::setTrafficLightPhase(const std::string &id, int phaseIndex) {
        if (!rlTrafficLight) {
            std::cerr << "please set rlTrafficLight to true to enable traffic light control" << std::endl;
            return;
        }
        roadNet.getIntersectionById(id)->getTrafficLight().setPhase(phaseIndex);
    }

    void Engine::reset() {
        for (auto &vehiclePair : vehiclePool) delete vehiclePair.second.first;
        for (auto &pool : threadVehiclePool) pool.clear();
        vehiclePool.clear();
        roadNet.reset();
        for (auto &flow : flows) flow.reset();
        step = 0;
        activeVehicleCount = 0;
    }

    Engine::~Engine() {
        logOut.close();
        finished = true;
        for (int i = 0; i < (laneChange ? 6 : 5); ++i) {
            startBarrier.wait();
            endBarrier.wait();
        }
        for (auto &thread : threadPool) thread.join();
        for (auto &vehiclePair : vehiclePool) delete vehiclePair.second.first;
    }

    void Engine::handleLaneChange(Vehicle &vehicle, const ControlInfo &controlInfo) {
        if (fabs(controlInfo.changingSpeed) < eps) return;
        if (!vehicle.hasPartner()) {
            Vehicle *shadow = new Vehicle(vehicle, vehicle.getPriority(), vehicle.getId() + "_shadow", *this);
            shadow->setParent(&vehicle);
            vehicle.setShadow(shadow);
            shadow->setLane(controlInfo.nextLane);
            shadow->setOffset(0);
            shadowBuffer.emplace_back(shadow, controlInfo);
        }
        vehicle.setOffset(vehicle.getOffset() + controlInfo.changingSpeed * interval);
        if (fabs(
                vehicle.getOffset()) >=
            (vehicle.getCurDrivable()->getWidth() + controlInfo.nextLane->getWidth()) / 2.0 - eps) {
            finishLaneChangeBuffer.emplace_back(&vehicle);
        }
    }

    void Engine::finishLaneChange() {
        for (auto vehicle : finishLaneChangeBuffer) {
            Vehicle *shadow = vehicle->getPartner();
            shadow->finishChanging();
            shadow->setId(vehicle->getId());
            auto &vehs2 = ((Lane *) vehicle->getCurLane())->getSegment(vehicle->getSegmentIndex())->getVehicles();
            for (auto iter = vehs2.begin(); iter != vehs2.end(); ++iter) {
                if ((*(*iter)) == vehicle) {
                    vehs2.erase(iter);
                    break;
                }
            }
            auto &vehs = ((Lane *) vehicle->getCurLane())->getVehicles();
            for (auto iter = vehs.begin(); iter != vehs.end(); ++iter) {
                if ((*iter) == vehicle) {
                    vehs.erase(iter);
                    break;
                }
            }
            auto iter = vehiclePool.find(vehicle->getPriority());
            assert(iter != vehiclePool.end());
            int threadIndex = iter->second.second;
            threadVehiclePool[threadIndex].erase(vehicle);
            vehiclePool.erase(iter);
            delete vehicle;
        }
        finishLaneChangeBuffer.clear();
    }

    void Engine::pushShadow() {
        for (auto &shadowPair : shadowBuffer) {
            Vehicle *shadow = shadowPair.first;
            ControlInfo controlInfo = shadowPair.second;
            int priority = shadow->getPriority();
            while (checkPriority(priority)) priority = rnd();
            shadow->setPriority(priority);
            size_t threadIndex = rnd() % threadNum;
            vehiclePool.emplace(shadow->getPriority(), std::make_pair(shadow, threadIndex));
            threadVehiclePool[threadIndex].insert(shadow);
            auto segment = controlInfo.nextLane->getSegment(shadow->getSegmentIndex());
            auto &vehs = controlInfo.nextLane->getVehicles();
            std::list<Vehicle *>::iterator it;
            bool mark = false;
            if (!vehs.empty())
                for (auto iter = vehs.begin(); iter != vehs.end(); ++iter) {
                    if ((*iter)->getDistance() <= shadow->getDistance()) {
                        it = controlInfo.nextLane->getVehicles().insert(iter, shadow);
                        mark = true;
                        break;
                    }
                }
            if (vehs.empty()) {
                vehs.push_back(shadow);
                it = vehs.begin();
            } else if (!mark) {
                controlInfo.nextLane->getVehicles().insert(vehs.end(), shadow);
                it = vehs.end();
                --it;
            }

            auto &vehs2 = segment->getVehicles();
            mark = false;

            while (vehs2.begin() != vehs2.end() && **vehs2.begin() == NULL)
                vehs2.erase(vehs2.begin());
            for (auto iter = vehs2.begin(); iter != vehs2.end(); ++iter) {
                if ((**iter)->getDistance() <= shadow->getDistance()) {
                    mark = true;
                    vehs2.insert(iter, it);
                    break;
                }
            }
            if (vehs2.empty())
                vehs2.push_back(it);
            else if (!mark) {
                vehs2.insert(vehs2.end(), it);
            }
        }
        shadowBuffer.clear();
    }

    void Engine::setLogFile(const std::string &jsonFile, const std::string &logFile) {
        std::ofstream jsonOut(jsonFile);
        Json::StreamWriterBuilder builder;
        builder.settings_["indentation"] = "";
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(jsonRoot, &jsonOut);
        jsonOut.close();

        logOut.open(logFile);
    }

    std::vector<Vehicle *> Engine::getRunningVehicle() const {
        std::vector<Vehicle *> ret;
        ret.reserve(activeVehicleCount);
        for (auto &lane:roadNet.getLanes()) {
            auto vehicles = lane->getVehicles();
            ret.insert(ret.end(), vehicles.begin(), vehicles.end());
        }
        for (auto &laneLink:roadNet.getLaneLinks()) {
            auto vehicles = laneLink->getVehicles();
            ret.insert(ret.end(), vehicles.begin(), vehicles.end());
        }
        return ret;
    }
}
