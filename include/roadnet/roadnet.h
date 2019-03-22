#ifndef CITY_SIMULATOR_ROADNET_H
#define CITY_SIMULATOR_ROADNET_H

#include "roadnet/trafficlight.h"
#include "utility/utility.h"
#include "utility/json/json.h"

#include <vector>
#include <string>
#include <cmath>
#include <list>
#include <map>
#include <queue>

namespace CitySimulator {
    class RoadNet;

    class Intersection;

    class Road;

    class Lane;

    class LaneLink;

    class Vehicle;

    class Cross;

    class Segment {
        friend Lane;
    public:
        //Vehicle * tryChange = nullptr;

        Segment() = default;

        Segment(size_t index, Lane *belongLane, double startPos, double endPos) : index(index), belongLane(belongLane),
                                                                                  startPos(startPos), endPos(endPos) { }

        inline double getStartPos() const { return this->startPos; }

        inline double getEndPos() const { return this->endPos; }

        inline size_t getIndex() const { return this->index; }

        inline const std::list<std::list<Vehicle *>::iterator> &getVehicles() const { return this->vehicles; }

        inline std::list<std::list<Vehicle *>::iterator> &getVehicles() { return this->vehicles; }

        inline std::list<Vehicle *>::iterator getPrevVehicleIter() const { return this->prev_vehicle_iter; }

    private:
        size_t index = 0;
        Lane *belongLane = nullptr;
        double startPos = 0;
        double endPos = 0;
        std::list<std::list<Vehicle *>::iterator> vehicles;
        std::list<Vehicle *>::iterator prev_vehicle_iter;
    };

    class Intersection {
        friend class RoadNet;

        friend class RoadLink;

        friend class Road;

        friend class TrafficLight;

    private:
        std::string id;
        bool isVirtual;
        double width = 0.0;
        Point point;
        TrafficLight trafficLight;
        std::vector<Road *> roads;
        std::vector<RoadLink> roadLinks;
        std::vector<Cross> crosses;

        void initCrosses();

    public:
        inline std::string getId() const { return this->id; }

        inline const TrafficLight &getTrafficLight() const { return trafficLight; }

        inline TrafficLight &getTrafficLight() { return trafficLight; }

        inline const std::vector<Road *> &getRoads() const { return this->roads; }

        inline std::vector<Road *> &getRoads() { return this->roads; }

        inline const std::vector<RoadLink> &getRoadLinks() const { return this->roadLinks; }

        inline std::vector<RoadLink> &getRoadLinks() { return this->roadLinks; }

        inline bool isVirtualIntersection() const { return this->isVirtual; }

        void reset();
    };

    class Cross {
        friend class RoadNet;

        friend class Intersection;

    private:
        LaneLink *laneLinks[2];
        Vehicle *notifyVehicles[2];
        double notifyDistances[2];
        double distanceOnLane[2];
        double leaveDistance = 0, arriveDistance = 30; // TODO
        double ang;
        double safeDistances[2];

    public:
        inline double getLeaveDistance() const { return leaveDistance; }

        inline double getArriveDistance() const { return arriveDistance; }

        void notify(LaneLink *laneLink, Vehicle *vehicle, double notifyDistance);

        bool canPass(const Vehicle *vehicle, LaneLink *laneLink,
                     double distanceToLaneLinkStart) const; // XXX: change to LaneLink based?

        inline void clearNotify() { notifyVehicles[0] = notifyVehicles[1] = nullptr; }

        inline Vehicle *getFoeVehicle(LaneLink *laneLink) const {
            assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
            return laneLink == laneLinks[0] ? notifyVehicles[1] : notifyVehicles[0];
        }

        inline double getDistanceByLane(LaneLink *laneLink) const {
            // XXX: lanelink not in cross?
            assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
            return laneLink == laneLinks[0] ? distanceOnLane[0] : distanceOnLane[1];
        }

        inline double getNotifyDistanceByLane(LaneLink *laneLink) const {
            // XXX: lanelink not in cross?
            assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
            return laneLink == laneLinks[0] ? notifyDistances[0] : notifyDistances[1];
        }

        inline double getSafeDistanceByLane(LaneLink *laneLink) {
            assert(laneLink == laneLinks[0] || laneLink == laneLinks[1]);
            return laneLink == laneLinks[0] ? safeDistances[0] : safeDistances[1];
        }

        inline double getAng() const {
            return ang;
        }

        inline LaneLink *getLaneLink(int i) const { return laneLinks[i]; }

        void reset();
    };

    class Road {
        friend class RoadNet;

        friend class Lane;

    private:
        std::string id;
        Intersection *startIntersection = nullptr;
        Intersection *endIntersection = nullptr;
        std::vector<Lane> lanes;
        std::vector<Point> points;

        void initLanesPoints();

    public:
        inline std::string getId() const { return this->id; }

        inline const Intersection &getStartIntersection() const { return *(this->startIntersection); }

        inline const Intersection &getEndIntersection() const { return *(this->endIntersection); }

        inline const std::vector<Lane> &getLanes() const { return this->lanes; }

        inline std::vector<Lane> &getLanes() { return this->lanes; }


        void buildSegmentationByInterval(double interval);

        void reset();
    };

    class Drivable {
        friend class RoadNet;

    public:
        enum DrivableType {
            LANE = 0, LANELINK = 1
        };

    protected:
        double length;
        double width;
        double maxSpeed;
        std::list<Vehicle *> vehicles;
        std::vector<Point> points;
        DrivableType drivableType;

    public:
        inline const std::list<Vehicle *> &getVehicles() const { return vehicles; }

        inline std::list<Vehicle *> &getVehicles() { return vehicles; }

        inline double getLength() const { return length; }

        inline double getWidth() const { return width; }

        inline double getMaxSpeed() const { return maxSpeed; }

        inline size_t getVehicleCount() const { return vehicles.size(); }

        inline DrivableType getDrivableType() const { return drivableType; }

        inline bool isLane() const { return drivableType == LANE; }

        inline bool isLaneLink() const { return drivableType == LANELINK; }

        inline Vehicle *getFirstVehicle() const {
            if (!vehicles.empty()) return vehicles.front();
            return nullptr;
        }

        inline Vehicle *getLastVehicle() const {
            if (!vehicles.empty()) return vehicles.back();
            return nullptr;
        }

        Point getPointByDistance(double dis) const;

        Point getDirectionByDistance(double dis) const;

        inline void pushVehicle(Vehicle *vehicle) { vehicles.push_back(vehicle); }

        inline void popVehicle() { vehicles.pop_front(); }
    };

    class Lane : public Drivable {

        friend class RoadNet;

        friend class Road;

    private:
        int laneIndex;
        std::vector<Segment> segments;
        std::vector<LaneLink *> laneLinks;
        Road *belongRoad = nullptr;
        std::deque<Vehicle *> waitingBuffer;

    public:
        Lane();

        Lane(double width, double maxSpeed, int laneIndex, Road *belongRoad);

        inline std::string getId() const {
            return belongRoad->getId() + '_' + std::to_string(getLaneIndex());
        }

        inline Road *getBelongRoad() const { return this->belongRoad; }

        bool available(const Vehicle *vehicle) const;

        bool canEnter(const Vehicle *vehicle) const;

        inline int getLaneIndex() const { return this->laneIndex; }

        inline const std::vector<LaneLink *> &getLaneLinks() const { return this->laneLinks; }

        inline std::vector<LaneLink *> &getLaneLinks() { return this->laneLinks; }

        inline Intersection *getStartIntersection() const {
            return belongRoad->startIntersection;
        }

        inline Intersection *getEndIntersection() const {
            return belongRoad->endIntersection;
        }

        std::vector<LaneLink *> getLaneLinksToRoad(Road *road) const;

        void reset();

        /* waiting buffer */
        inline const std::deque<Vehicle *> &getWaitingBuffer() const { return waitingBuffer; }

        inline std::deque<Vehicle *> &getWaitingBuffer() { return waitingBuffer; }

        void pushWaitingVehicle(Vehicle *vehicle) {
            waitingBuffer.emplace_back(vehicle);
        }

        /* segmentation */
        void buildSegmentation(size_t numSegs);

        void initSegments();

        inline const Segment *getSegment(size_t index) const { return &segments[index]; }

        inline Segment *getSegment(size_t index) { return &segments[index]; }

        inline const std::vector<Segment> &getSegments() const { return segments; }

        inline std::vector<Segment> &getSegments() { return segments; }

        inline size_t getSegmentNum() { return segments.size(); }
    };


    enum RoadLinkType {
        go_straight = 3, turn_left = 2, turn_right = 1
    };

    class RoadLink {
        friend class RoadNet;

        friend class LaneLink;

    private:
        Intersection *intersection = nullptr;
        Road *startRoad = nullptr;
        Road *endRoad = nullptr;
        RoadLinkType type;
        std::vector<LaneLink> laneLinks;
        int index;
    public:
        inline const std::vector<LaneLink> &getLaneLinks() const { return this->laneLinks; }

        inline std::vector<LaneLink> &getLaneLinks() { return this->laneLinks; }

        inline Road *getStartRoad() const { return this->startRoad; }

        inline Road *getEndRoad() const { return this->endRoad; }

        inline bool isAvailable() const {
            return this->intersection->trafficLight.getCurrentPhase().roadLinkAvailable[this->index];
        }

        inline bool isTurn() const {
            return type == turn_left || type == turn_right;
        }

        void reset();
    };

    class LaneLink : public Drivable {

        friend class RoadNet;

        friend class Intersection;

    private:
        RoadLink *roadLink = nullptr;
        Lane *startLane = nullptr;
        Lane *endLane = nullptr;
        std::vector<Cross *> crosses;

    public:
        LaneLink() {
            width = 4;
            maxSpeed = 10000; //TODO
            drivableType = LANELINK;
        }

        inline RoadLink *getRoadLink() const { return this->roadLink; }

        inline RoadLinkType getRoadLinkType() const { return this->roadLink->type; }

        inline const std::vector<Cross *> &getCrosses() const { return this->crosses; }

        inline std::vector<Cross *> &getCrosses() { return this->crosses; }

        inline Lane *getStartLane() const { return startLane; }

        inline Lane *getEndLane() const { return endLane; }

        inline bool isAvailable() const { return roadLink->isAvailable(); }

        bool isTurn() const { return roadLink->isTurn(); }

        void reset();
    };


    class RoadNet {
    private:
        std::vector<Road> roads;
        std::vector<Intersection> intersections;
        std::map<std::string, Road *> roadMap;
        std::map<std::string, Intersection *> interMap;
    public:
        bool loadFromJson(std::string jsonFileName);

        Json::Value convertToJson();

        inline const std::vector<Road> &getRoads() const { return this->roads; }

        inline std::vector<Road> &getRoads() { return this->roads; }

        inline const std::vector<Intersection> &getIntersections() const { return this->intersections; }

        inline std::vector<Intersection> &getIntersections() { return this->intersections; }

        inline Road *getRoadById(const std::string &id) const {
            return roadMap.count(id) > 0 ? roadMap.at(id) : nullptr;
        }

        inline Intersection *getIntersectionById(const std::string &id) const {
            return interMap.count(id) > 0 ? interMap.at(id) : nullptr;
        }

        std::vector<const LaneLink *> getLaneLinks() const;

        std::vector<LaneLink *> getLaneLinks();

        std::vector<const Lane *> getLanes() const;

        std::vector<Lane *> getLanes();

        void reset();
    };
}


#endif //CITY_SIMULATOR_ROADNET_H
