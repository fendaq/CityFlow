#ifndef CITY_SIMULATOR_VEHICLE
#define CITY_SIMULATOR_VEHICLE

#include "utility/utility.h"
#include "flow/route.h"
#include "vehicle/router.h"

#include <vector>
#include <string>
#include <utility>

namespace CitySimulator {
	class Lane;

	class LaneLink;

	class Intersection;

	class Route;

	class Cross;

	class Drivable;

	class Engine;

	class Point;

	struct VehicleInfo {
		double dis = 0;
		double speed = 0;
		double len = 5;
		double width = 2;
		double maxPosAcc = 4.5;
		double maxNegAcc = 4.5;
		double usualPosAcc = 2.5;
		double usualNegAcc = 2.5;
		double minGap = 2;
		double maxSpeed = 16.66667;
		double headwayTime = 1;
		double yieldDistance = 5;
		double turnSpeed = 8.3333;
		Route route; // XXX: what if use pointer instead? point to flow.route may change place, why?
	};

	class Vehicle {
	private:
		struct Buffer {
			bool isDisSet = false;
			bool isSpeedSet = false;
			bool isIDrivableSet = false;
			bool isEndSet = false;
			bool isLeaderSet = false;
			bool isEnterLaneLinkTimeSet = false;
			bool isCurLaneSet = false;
			bool isCurLaneLinkSet = false;
			bool isBlockerSet = false;
			double dis;
			double speed;
			std::vector<Drivable *>::iterator iDrivable;
			bool end;
			Vehicle *leader;
			Vehicle *blocker;
			size_t enterLaneLinkTime;
			Lane * curLane;
			LaneLink * curLaneLink;
		};

		struct LaneChangeInfo {
			bool changed = false;
			short partnerType = 0; //0 for no partner; 1 for real vehicle; 2 for shadow vehicle;
			Vehicle *partner = nullptr;
			double offset = 0;
			double bufferLength = 20;
			int segmentIndex = 0;
			Lane * target;
		} laneChangeInfo;

		struct ControllerInfo {
			Lane *curLane;
			LaneLink *curLaneLink;
			double approachingIntersectionDistance;
			double gap;
			std::vector<Drivable *>::iterator iDrivable;
			size_t enterLaneLinkTime;
			Vehicle *leader; // previous vehicle
			Vehicle *blocker;
			std::vector<Drivable *> wholeRoute;
			const Route *route;
			bool end;
			bool running = false;
			Router router;
            bool isJump = false;
            double jumpDistance = 0.0;
			ControllerInfo(const Route *_route, std::mt19937 &rnd);
		} controllerInfo;

		VehicleInfo vehicleInfo;
		Buffer buffer;

		int priority;
		std::string id;

		Engine &engine;

	public:
		bool isStraightHold = false;

		Vehicle(const Vehicle &vehicle, int _priority, const std::string &_id, Engine &engine);

		Vehicle(const VehicleInfo &init, int priority, const std::string &id, Engine &engine);

		void setDeltaDistance(double dis);

		void setSpeed(double speed);

		void setIDrivable(std::vector<Drivable *>::iterator iDrivable);

        void setCurLane(Lane * curLane) ;

        void setCurLaneLink(LaneLink * curLaneLink);

		void setEnd(bool end);

		void setLeader(Vehicle *leader);

		void setEnterLaneLinkTime(size_t enterLaneLinkTime);

		void setBlocker(Vehicle *blocker);

		bool hasSetEnd() const { return buffer.isEndSet; }

		void update();

		void setPriority(int priority) { this->priority = priority; }

		inline std::string getId() const { return id; }

		inline double getSpeed() const { return vehicleInfo.speed; }

		inline double getLen() const { return vehicleInfo.len; }

		inline double getWidth() const { return vehicleInfo.width; }

		inline double getDistance() const { return vehicleInfo.dis; }

		Point getPoint() const;

		inline double getMaxPosAcc() const { return vehicleInfo.maxPosAcc; }

		inline double getMaxNegAcc() const { return vehicleInfo.maxNegAcc; }

		inline double getUsualPosAcc() const { return vehicleInfo.usualPosAcc; }

		inline double getUsualNegAcc() const { return vehicleInfo.usualNegAcc; }

		inline double getMinGap() const { return vehicleInfo.minGap; }

		inline double getYieldDistance() const { return vehicleInfo.yieldDistance; }

		inline double getTurnSpeed() const { return vehicleInfo.turnSpeed; }

		inline Vehicle *getBlocker() const { return controllerInfo.blocker; }

		Drivable *getCurDrivable() ;

		inline Drivable *getNextDrivable(int i = 1) const {
			return controllerInfo.iDrivable + i >= controllerInfo.wholeRoute.end() ? nullptr
				: *(controllerInfo.iDrivable + i);
		}

		inline Drivable *getPrevDrivable(int i = 1) const {
			return controllerInfo.iDrivable - i < controllerInfo.wholeRoute.begin() ? nullptr
				: *(controllerInfo.iDrivable - i);
		}

		inline int getPriority() const { return priority; }

		std::pair<Point, Point> getCurPos() const;

		ControlInfo getNextSpeed(double interval);

		Drivable *getChangedDrivable() const ;

		inline bool isEnd() const { return controllerInfo.iDrivable == controllerInfo.wholeRoute.end(); }

		bool isIntersectionRelated() const;

		double getBrakeDistanceAfterAccel(double acc, double dec, double interval) const;

		inline double getMinBrakeDistance() const { return 0.5 * vehicleInfo.speed * vehicleInfo.speed / vehicleInfo.maxNegAcc; }

		inline double getUsualBrakeDistance() const { return 0.5 * vehicleInfo.speed * vehicleInfo.speed / vehicleInfo.usualNegAcc; }

		double getNoCollisionSpeed(double vL, double dL, double vF, double dF, double gap, double interval,
			double targetGap) const;

		double getCarFollowSpeed(double interval);

		double getStopBeforeSpeed(double distance, double interval) const;

		int getReachSteps(double distance, double targetSpeed, double acc) const;

		int getReachStepsOnLaneLink(double distance, LaneLink* laneLink) const;

        double getDistanceUntilSpeed(double speed, double acc) const;

		bool canYield(double dist) const;

		Vehicle *getLeader();

		inline double getEnterLaneLinkTime() const { return controllerInfo.enterLaneLinkTime; }

		inline double getHeadwayTime() const { return vehicleInfo.headwayTime; }

		inline double getMaxSpeed() const { return vehicleInfo.maxSpeed; }

		inline double getApproachingIntersectionDistance() const { return 0.0; }

		double getIntersectionRelatedSpeed(double interval);

		inline bool isRunning() const { return controllerInfo.running; }

		inline void setRunning(bool isRunning = true) { controllerInfo.running = isRunning; }

		inline bool hasPartner() const { return laneChangeInfo.partnerType > 0; }

		inline bool isReal() const { return laneChangeInfo.partnerType != 2; }

		inline int getSegmentIndex() const { return laneChangeInfo.segmentIndex; }

		inline void setSegmentIndex(int segmentIndex) { laneChangeInfo.segmentIndex = segmentIndex; }

		inline void setShadow(Vehicle *veh) { laneChangeInfo.partnerType = 1, laneChangeInfo.partner = veh; }

		inline void setParent(Vehicle *veh) { laneChangeInfo.partnerType = 2, laneChangeInfo.partner = veh; }

		int toChange(double intervalm, double nextSpeed) ;

		void finishChanging();

		inline void setOffset(double offset) { laneChangeInfo.offset = offset; }

		inline double getOffset() const { return laneChangeInfo.offset; }

		inline Vehicle *getPartner() const { return laneChangeInfo.partner; }

		bool checkSegment(Lane *lan, size_t index,double interval, double nextSpeed) const;

		void setLane(Lane * nextLane);

		bool tryChange();

		bool isChanged();

		Lane * getCurLane();

		inline void setId(const std::string & id) { this->id = id; }

		inline bool isJump(){return controllerInfo.isJump;}

		inline void resetJump() {controllerInfo.isJump = false;}

		inline double getJumpDistance() { return controllerInfo.jumpDistance;}

		bool isLaneChanged(Lane * curLane, LaneLink * originLaneLink);

		inline auto & getiDrivable() {
		    return controllerInfo.iDrivable;
        }

        double findNextGap(double dis, Lane * lane, int segmentIndex);
	};
}

#endif