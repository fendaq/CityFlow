#include "vehicle/vehicle.h"
#include "roadnet/roadnet.h"
#include "engine/engine.h"

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cassert>
#include <limits>
#include <random>

namespace CitySimulator {

    Vehicle::ControllerInfo::ControllerInfo(const CitySimulator::Route *_route, std::mt19937 &rnd) : route(_route) {
        wholeRoute = router.getRoutes(route, rnd);
        iDrivable = wholeRoute.begin();
        leader = (*iDrivable)->getLastVehicle();
        curLane = (Lane *) (*iDrivable);
        if ((iDrivable + 1) != wholeRoute.end())
            curLaneLink = (LaneLink *) (*(iDrivable + 1));
        enterLaneLinkTime = std::numeric_limits<int>::max();
    }

    Vehicle::Vehicle(const Vehicle &vehicle, int _priority, const std::string &_id, Engine &engine) : laneChangeInfo(
            vehicle.laneChangeInfo), controllerInfo(vehicle.controllerInfo), vehicleInfo(vehicle.vehicleInfo),
                                                                                                      buffer(vehicle.buffer),
                                                                                                      priority(
                                                                                                              _priority),
                                                                                                      id(_id),
                                                                                                      engine(engine) {
        controllerInfo.iDrivable = controllerInfo.wholeRoute.begin() +
                                   (vehicle.controllerInfo.iDrivable - vehicle.controllerInfo.wholeRoute.begin());
        controllerInfo.approachingIntersectionDistance =
                vehicleInfo.maxSpeed * vehicleInfo.maxSpeed / vehicleInfo.usualNegAcc / 2 +
                vehicleInfo.maxSpeed * engine.getInterval() * 2;
    }

    Vehicle::Vehicle(const VehicleInfo &init, int priority, const std::string &id, Engine &engine) : controllerInfo(
            &(init.route), engine.rnd), vehicleInfo(init), engine(engine) {
        controllerInfo.approachingIntersectionDistance =
                vehicleInfo.maxSpeed * vehicleInfo.maxSpeed / vehicleInfo.usualNegAcc / 2 +
                vehicleInfo.maxSpeed * engine.getInterval() * 2;
        this->priority = priority;
        this->id = id;
    }

    void Vehicle::setDeltaDistance(double dis) {
        if (!buffer.isDisSet) {
            buffer.dis = dis + vehicleInfo.dis;
            buffer.isDisSet = true;
            if (buffer.dis > getCurDrivable()->getLength()) {
                buffer.dis = buffer.dis - getCurDrivable()->getLength();
                Drivable *nextDrivable = controllerInfo.router.getNextDrivable(&controllerInfo.wholeRoute, buffer.dis,
                                                                               controllerInfo.iDrivable,
                                                                               controllerInfo.curLane,
                                                                               controllerInfo.curLaneLink);
                if ((*controllerInfo.iDrivable)->isLaneLink()) {
                    setIDrivable(controllerInfo.iDrivable + 1);
                    setCurLane((Lane *) nextDrivable);
                    if (controllerInfo.iDrivable + 2 == controllerInfo.wholeRoute.end())
                        laneChangeInfo.changed = false;
                    else
                        laneChangeInfo.changed = isLaneChanged((Lane *) nextDrivable,
                                                               (LaneLink *) *(controllerInfo.iDrivable + 2));
                    if (laneChangeInfo.changed)
                        int ij = 0;
                    return;
                }
                if (controllerInfo.iDrivable + 1 == controllerInfo.wholeRoute.end()) {
                    setIDrivable(controllerInfo.iDrivable + 1);
                    setEnd(true);
                    return;
                }
                if (nextDrivable->isLaneLink()) {
                    setIDrivable(controllerInfo.iDrivable + 1);
                    setCurLaneLink((LaneLink *) nextDrivable);
                } else {
                    controllerInfo.isJump = true;
                    controllerInfo.jumpDistance = (*(controllerInfo.iDrivable + 1))->getLength();
                    setIDrivable(controllerInfo.iDrivable + 2);
                    setCurLane((Lane *) nextDrivable);
                }
            }
        } else {
            if (buffer.dis != dis + vehicleInfo.dis)
                buffer.dis = min2double(buffer.dis, dis + vehicleInfo.dis);
        }
    }

    void Vehicle::setSpeed(double speed) {
        if (!buffer.isSpeedSet) {
            buffer.speed = speed;
            buffer.isSpeedSet = true;
        } else {
            buffer.speed = min2double(buffer.speed, speed);
        }
    }

    Drivable *Vehicle::getChangedDrivable() const {
        if (!(buffer.isIDrivableSet && !buffer.isEndSet))
            return nullptr;
        if (buffer.isCurLaneSet)
            return buffer.curLane;
        else
            return buffer.curLaneLink;
    }

    void Vehicle::setIDrivable(std::vector<Drivable *>::iterator iDrivable) {
        buffer.iDrivable = iDrivable;
        buffer.isIDrivableSet = true;
    }

    void Vehicle::setCurLane(Lane *curLane) {
        buffer.curLane = curLane;
        buffer.isCurLaneSet = true;
    }

    void Vehicle::setCurLaneLink(LaneLink *curLaneLink) {
        buffer.curLaneLink = curLaneLink;
        buffer.isCurLaneLinkSet = true;
    }

    void Vehicle::setEnd(bool end) {
        buffer.end = end;
        buffer.isEndSet = true;
    }

    void Vehicle::setLeader(Vehicle *leader) {
        buffer.leader = leader;
        buffer.isLeaderSet = true;
    }

    void Vehicle::setEnterLaneLinkTime(size_t enterLaneLinkTime) {
        buffer.enterLaneLinkTime = enterLaneLinkTime;
        buffer.isEnterLaneLinkTimeSet = true;
    }

    void Vehicle::setBlocker(Vehicle *blocker) {
        buffer.blocker = blocker;
        buffer.isBlockerSet = true;
    }

    Point Vehicle::getPoint() const {
        if (fabs(laneChangeInfo.offset) < eps) {
            if ((*controllerInfo.iDrivable)->getDrivableType() == Drivable::LANE)
                return (controllerInfo.curLane)->getPointByDistance(vehicleInfo.dis);
            else
                return (controllerInfo.curLaneLink)->getPointByDistance(vehicleInfo.dis);
        } else {
            Point origin = controllerInfo.curLane->getPointByDistance(vehicleInfo.dis);
            Point next;
            double percentage;
            std::vector<Lane> &lans = controllerInfo.curLane->getBelongRoad()->getLanes();
            if (laneChangeInfo.offset > 0) {
                next = lans[controllerInfo.curLane->getLaneIndex() + 1].getPointByDistance(vehicleInfo.dis);
                percentage = 2 * laneChangeInfo.offset / (controllerInfo.curLane->getWidth() +
                                                          lans[controllerInfo.curLane->getLaneIndex() + 1].getWidth());
            } else {
                next = lans[controllerInfo.curLane->getLaneIndex() - 1].getPointByDistance(vehicleInfo.dis);
                percentage = -2 * laneChangeInfo.offset / (controllerInfo.curLane->getWidth() +
                                                           lans[controllerInfo.curLane->getLaneIndex() - 1].getWidth());
            }
            Point cur;
            cur.x = next.x * percentage + origin.x * (1 - percentage);
            cur.y = next.y * percentage + origin.y * (1 - percentage);
            return cur;
        }
    }

    void Vehicle::update() { // TODO: use something like reflection?
        if (buffer.isDisSet) {
            vehicleInfo.dis = buffer.dis;
            buffer.isDisSet = false;
        }
        if (buffer.isSpeedSet) {
            vehicleInfo.speed = buffer.speed;
            buffer.isSpeedSet = false;
        }
        if (buffer.isIDrivableSet) {
            controllerInfo.iDrivable = buffer.iDrivable;
            buffer.isIDrivableSet = false;
        }
        if (buffer.isEndSet) {
            controllerInfo.end = buffer.end;
            buffer.isEndSet = false;
        }
        if (buffer.isLeaderSet) {
            controllerInfo.leader = buffer.leader;
            buffer.isLeaderSet = false;
        }
        if (buffer.isEnterLaneLinkTimeSet) {
            controllerInfo.enterLaneLinkTime = buffer.enterLaneLinkTime;
            buffer.isEnterLaneLinkTimeSet = false;
        }
        if (buffer.isCurLaneSet) {
            controllerInfo.curLane = buffer.curLane;
            buffer.isCurLaneSet = false;
        }
        if (buffer.isCurLaneLinkSet) {
            controllerInfo.curLaneLink = buffer.curLaneLink;
            buffer.isCurLaneLinkSet = false;
        }
        if (buffer.isBlockerSet) {
            controllerInfo.blocker = buffer.blocker;
            buffer.isBlockerSet = false;
        } else {
            controllerInfo.blocker = nullptr;
        }
        if ((*controllerInfo.iDrivable)->isLane() &&
            controllerInfo.curLane->getBelongRoad() != ((Lane *) *controllerInfo.iDrivable)->getBelongRoad())
            int ij = 0;
    }


    std::pair<Point, Point> Vehicle::getCurPos() const {
        std::pair<Point, Point> ret;
        ret.first = (*controllerInfo.iDrivable)->getPointByDistance(vehicleInfo.dis);
        Point direction = (*controllerInfo.iDrivable)->getDirectionByDistance(vehicleInfo.dis);
        Point tail(ret.first);
        tail.x -= direction.x * vehicleInfo.len;
        tail.y -= direction.y * vehicleInfo.len;
        ret.second = tail;
        return ret;
    }

    Vehicle *Vehicle::getLeader() {
        if (controllerInfo.leader != nullptr) {
            controllerInfo.gap =
                    controllerInfo.leader->getDistance() - controllerInfo.leader->getLen() - vehicleInfo.dis;
            return controllerInfo.leader;
        } else {
            Drivable *drivable = nullptr;
            Vehicle *candidateLeader = nullptr;
            double candidateGap = 0;
            double dis = (*controllerInfo.iDrivable)->getLength() - vehicleInfo.dis;
            for (int i = 1; i < controllerInfo.wholeRoute.size(); ++i) {
                drivable = getNextDrivable(i);
                if (drivable == nullptr)
                    return nullptr;

                if (drivable->isLaneLink()) { // if laneLink, check all laneLink start from previous lane, because lanelinks may overlap 
                    for (auto laneLink : ((LaneLink *) drivable)->getStartLane()->getLaneLinks()) {
                        if ((candidateLeader = laneLink->getLastVehicle()) != nullptr) {
                            candidateGap = dis + candidateLeader->getDistance() - candidateLeader->getLen();
                            if (controllerInfo.leader == nullptr || candidateGap < controllerInfo.gap) {
                                controllerInfo.leader = candidateLeader;
                                controllerInfo.gap = candidateGap;
                            }
                        }
                    }
                    if (controllerInfo.leader) return controllerInfo.leader;
                } else {
                    if ((controllerInfo.leader = drivable->getLastVehicle()) != nullptr) {
                        controllerInfo.gap =
                                dis + controllerInfo.leader->getDistance() - controllerInfo.leader->getLen();
                        return controllerInfo.leader;
                    }
                }

                dis += drivable->getLength();
                if (dis > vehicleInfo.maxSpeed * vehicleInfo.maxSpeed / vehicleInfo.usualNegAcc / 2 +
                          vehicleInfo.maxSpeed * engine.getInterval() * 2)
                    return nullptr;
            }
            return nullptr;
        }
    }

    double Vehicle::getNoCollisionSpeed(double vL, double dL, double vF, double dF, double gap, double interval,
                                        double targetGap) const {
        double c = vF * interval / 2 + targetGap - 0.5 * vL * vL / dL - gap;
        if (c > 0) return -100;
        double a = 0.5 / dF;
        double b = 0.5 * interval;
        return 0.5 / a * (sqrt(b * b - 4 * a * c) - b);
    }

    // should be move to seperate CarFollowing (Controller?) class later?
    double Vehicle::getCarFollowSpeed(double interval) {
        Vehicle *leader = getLeader();
        if (leader == nullptr) return vehicleInfo.maxSpeed;

        // collision free
        double v = getNoCollisionSpeed(leader->getSpeed(), leader->getMaxNegAcc(), vehicleInfo.speed,
                                       vehicleInfo.maxNegAcc, controllerInfo.gap, interval, 0);

        // safe distance
        // get relative decel (mimic real scenario)
        double assumeDecel = 0, leaderSpeed = leader->getSpeed();
        if (vehicleInfo.speed > leaderSpeed) {
            assumeDecel = vehicleInfo.speed - leaderSpeed;
        }
        v = min2double(v, getNoCollisionSpeed(leader->getSpeed(), leader->getUsualNegAcc(), vehicleInfo.speed,
                                              vehicleInfo.usualNegAcc, controllerInfo.gap, interval,
                                              vehicleInfo.minGap));
        v = min2double(v,
                       (controllerInfo.gap + (leaderSpeed - assumeDecel / 2) * interval -
                        vehicleInfo.speed * interval / 2) / (vehicleInfo.headwayTime + interval / 2));

        return v;
    }

    double Vehicle::getStopBeforeSpeed(double distance, double interval) const {
        assert(distance >= 0);
        if (getBrakeDistanceAfterAccel(vehicleInfo.usualPosAcc, vehicleInfo.usualNegAcc, interval) < distance)
            return vehicleInfo.speed + vehicleInfo.usualPosAcc * interval;
        double takeInterval = 2 * distance / (vehicleInfo.speed + eps) / interval;
        if (takeInterval >= 1) {
            return vehicleInfo.speed - vehicleInfo.speed / (int) takeInterval;
        } else {
            return vehicleInfo.speed - vehicleInfo.speed / takeInterval;
        }
    }

    int Vehicle::getReachSteps(double distance, double targetSpeed, double acc) const {
        if (distance <= 0) {
            return 0;
        }
        if (vehicleInfo.speed > targetSpeed) {
            return std::ceil(distance / vehicleInfo.speed);
        }
        double distanceUntilTargetSpeed = getDistanceUntilSpeed(targetSpeed, acc);
        double interval = engine.getInterval();
        if (distanceUntilTargetSpeed > distance) {
            return std::ceil(
                    (std::sqrt(vehicleInfo.speed * vehicleInfo.speed + 2 * acc * distance) - vehicleInfo.speed) / acc /
                    interval);
        } else {
            return std::ceil((targetSpeed - vehicleInfo.speed) / acc / interval) +
                   std::ceil((distance - distanceUntilTargetSpeed) / targetSpeed / interval);
        }
    }

    int Vehicle::getReachStepsOnLaneLink(double distance, LaneLink *laneLink) const {
        return getReachSteps(distance, laneLink->isTurn() ? vehicleInfo.turnSpeed : vehicleInfo.maxSpeed,
                             vehicleInfo.usualPosAcc);
    }

    double Vehicle::getDistanceUntilSpeed(double speed, double acc) const {
        if (speed <= vehicleInfo.speed) return 0;
        double interval = engine.getInterval();
        int stage1steps = std::floor((speed - vehicleInfo.speed) / acc / interval);
        double stage1speed = vehicleInfo.speed + stage1steps * acc / interval;
        double stage1dis = (vehicleInfo.speed + stage1speed) * (stage1steps * interval) / 2;
        return stage1dis + (stage1speed < speed ? ((stage1speed + speed) * interval / 2) : 0);
    }

    bool Vehicle::canYield(double dist) const {
        return (dist > 0 && getMinBrakeDistance() < dist - vehicleInfo.yieldDistance)
               || (dist < 0 && dist + vehicleInfo.len < 0);
    }

    bool Vehicle::isIntersectionRelated() const {
        if ((*controllerInfo.iDrivable)->isLaneLink())
            return true;
        if ((*controllerInfo.iDrivable)->isLane()) {
            Drivable *drivable = getNextDrivable();
            if (drivable && drivable->isLaneLink() && (*controllerInfo.iDrivable)->getLength() - vehicleInfo.dis <=
                                                      controllerInfo.approachingIntersectionDistance) {
                return true;
            }
        }
        return false;
    }

    double Vehicle::getBrakeDistanceAfterAccel(double acc, double dec, double interval) const {
        double currentSpeed = vehicleInfo.speed;
        double nextSpeed = currentSpeed + acc * interval;
        return (currentSpeed + nextSpeed) * interval / 2 + (nextSpeed * nextSpeed / dec / 2);
    }

    ControlInfo Vehicle::getNextSpeed(double interval) { // TODO: pass as parameter or not?
        if ((*controllerInfo.iDrivable)->isLane() &&
            controllerInfo.curLane->getBelongRoad() != ((Lane *) *controllerInfo.iDrivable)->getBelongRoad())
            int ij = 0;
        Drivable *drivable = *controllerInfo.iDrivable;
        double v = vehicleInfo.maxSpeed;
        v = min2double(v, vehicleInfo.speed + vehicleInfo.maxPosAcc * interval); // TODO: random???

        v = min2double(v, drivable->getMaxSpeed());

        // car follow
        v = min2double(v, getCarFollowSpeed(interval));

        if (isIntersectionRelated()) {

            v = min2double(v, getIntersectionRelatedSpeed(interval));
        }
        v = max2double(v, vehicleInfo.speed - vehicleInfo.maxNegAcc * interval);
        ControlInfo controlInfo;
        controlInfo.speed = v;
        int change;
        if (engine.hasLaneChange()) {
            change = toChange(interval, v);
            if (change == 3) {//wait until can change
                if (getCurLane()->getLaneLinks().size() == 0 ||
                    (getCurLane()->getLaneLinks()[0]->getRoadLinkType() == 3 &&
                     controllerInfo.curLane->getLength() - vehicleInfo.dis >
                     1 * laneChangeInfo.bufferLength))
                    controlInfo.speed = v;
                else {
                    controlInfo.speed = max2double(0, vehicleInfo.speed - vehicleInfo.maxNegAcc * interval);
                }
                return controlInfo;
            }

            if ((*controllerInfo.iDrivable)->isLane() && (laneChangeInfo.partnerType == 1 || change > 0)) {
                if (change == 1) {
                    controlInfo.changingSpeed = (*controllerInfo.iDrivable)->getWidth() / (4);
                    controlInfo.nextLane = &(controllerInfo.curLane->getBelongRoad()->getLanes()[
                            controllerInfo.curLane->getLaneIndex() + 1]);
                } else {
                    controlInfo.changingSpeed = -(*controllerInfo.iDrivable)->getWidth() / (4);
                    controlInfo.nextLane = &(controllerInfo.curLane->getBelongRoad()->getLanes()[
                            controllerInfo.curLane->getLaneIndex() - 1]);
                }
                if (controllerInfo.curLane->getLength() - vehicleInfo.dis < laneChangeInfo.bufferLength * 3)
                    controlInfo.speed = max2double(0, vehicleInfo.speed - vehicleInfo.maxNegAcc * interval);
            }
//            if (laneChangeInfo.segmentIndex < controllerInfo.curLane->getSegmentNum() - 1) {
//                auto t = controllerInfo.curLane->getSegment(laneChangeInfo.segmentIndex + 1)->tryChange;
//
//                if (t != nullptr &&
//                    t->getSegmentIndex() ==
//                    laneChangeInfo.segmentIndex + 1) {
//                    Vehicle *leader = t;
//                    double v = getNoCollisionSpeed(leader->getSpeed(), leader->getMaxNegAcc(), vehicleInfo.speed,
//                                                   vehicleInfo.maxNegAcc, controllerInfo.gap, interval, 0);
//                    if (v != -100)
//                        controlInfo.speed = min2double(controlInfo.speed, v);
//                }
//            }

        }
        return controlInfo;
    }

    double Vehicle::getIntersectionRelatedSpeed(double interval) {
        double v = vehicleInfo.maxSpeed;
        Drivable *nextDrivable = getNextDrivable();
        LaneLink *laneLink = nullptr;
        if (nextDrivable && nextDrivable->isLaneLink()) {
            laneLink = (LaneLink *) nextDrivable;
            if (!laneLink->isAvailable() || !laneLink->getEndLane()->canEnter(
                    this)) { // not only the first vehicle should follow intersection logic
                if (getMinBrakeDistance() > (*controllerInfo.iDrivable)->getLength() - vehicleInfo.dis) {
                    // TODO: what if it cannot brake before red light?
                } else {
                    v = min2double(v, getStopBeforeSpeed((*controllerInfo.iDrivable)->getLength() - vehicleInfo.dis,
                                                         interval));
                    return v;
                }
            }
            if (laneLink->isTurn()) {
                v = min2double(v, vehicleInfo.turnSpeed); // TODO: define turn speed
            }
        }
        if (laneLink == nullptr && (*controllerInfo.iDrivable)->isLaneLink())
            laneLink = controllerInfo.curLaneLink;
        double distanceToLaneLinkStart = (*controllerInfo.iDrivable)->isLane() ? -(
                (*controllerInfo.iDrivable)->getLength() - vehicleInfo.dis) : vehicleInfo.dis;
        double distanceOnLaneLink;
        for (auto &cross : laneLink->getCrosses()) {
            distanceOnLaneLink = cross->getDistanceByLane(laneLink);
            if (distanceOnLaneLink < distanceToLaneLinkStart)
                continue;
            if (!cross->canPass(this, laneLink, distanceToLaneLinkStart)) {
                v = min2double(v, getStopBeforeSpeed(
                        distanceOnLaneLink - distanceToLaneLinkStart - vehicleInfo.yieldDistance,
                        interval)); // TODO: headway distance
                setBlocker(cross->getFoeVehicle(laneLink));
                break;
            }
        }
        return v;
    }

    bool Vehicle::checkSegment(Lane *lan, size_t index, double interval, double nextSpeed) const {
        if (index >= lan->getSegmentNum() || index < 0)
            return true;
        Segment *seg = lan->getSegment(index);
        std::list<std::list<Vehicle *>::iterator> &vehicles = seg->getVehicles();
        if (!vehicles.empty())
            for (auto veh: vehicles) {
                if ((*veh)->getDistance() > this->getDistance()) {
                    double front_gap =
                            (*veh)->getDistance() - this->getDistance() - (*veh)->getLen() - nextSpeed * interval;
                    if (front_gap < 0)
                        return false;
                    double v = getNoCollisionSpeed((*veh)->getSpeed(), (*veh)->getUsualNegAcc(), nextSpeed,
                                                   vehicleInfo.usualNegAcc, front_gap, interval,
                                                   0);
                    if (v == -100 || nextSpeed - v > vehicleInfo.maxNegAcc * interval)
                        return false;
                } else {
                    double back_gap =
                            this->getDistance() - (*veh)->getDistance() - this->getLen() + nextSpeed * interval -
                            (*veh)->getSpeed() * interval - 0.5 * (*veh)->getMaxPosAcc() * interval * interval;
                    if (back_gap < 0)
                        return false;
                    double v = getNoCollisionSpeed(this->getSpeed(), this->getUsualNegAcc(),
                                                   (*veh)->getSpeed() + (*veh)->getMaxPosAcc() * interval,
                                                   (*veh)->getMaxNegAcc(), back_gap,
                                                   interval, 0);
                    if (v == -100 ||
                        (*veh)->getSpeed() + (*veh)->getMaxPosAcc() * interval - v > (*veh)->getMaxNegAcc() * interval)
                        return false;
                }
            }
        return true;
    }

    int Vehicle::toChange(double interval,
                          double nextSpeed) {//0 for no change, 1 for left; 2 for right; 3 for wait until can change
        if (laneChangeInfo.partnerType == 2)
            return 0;
        if ((*controllerInfo.iDrivable)->getDrivableType() != Drivable::LANE)
            return 0;
        if (fabs(laneChangeInfo.offset) > eps) {
            if (laneChangeInfo.offset > 0)
                return 1;
            else
                return 2;
        }

        auto curLane = controllerInfo.curLane;
        auto &segmentIndex = laneChangeInfo.segmentIndex;
        std::vector<Lane> &lanes = curLane->getBelongRoad()->getLanes();
        if (laneChangeInfo.changed) {
            if (controllerInfo.leader != nullptr && tryChange()) {
                if (curLane->getLaneIndex() - 1 >= 0 && lanes.size() > 0 &&
                    (controllerInfo.iDrivable + 1 != controllerInfo.wholeRoute.end()) &&
                    lanes[curLane->getLaneIndex() - 1].getLaneLinks()[0]->getRoadLinkType() ==
                    ((Lane *) (*controllerInfo.iDrivable))->getLaneLinks()[0]->getRoadLinkType()) {
                    if (checkSegment(&lanes[curLane->getLaneIndex() - 1], segmentIndex, interval, nextSpeed) &&
                        checkSegment(
                                &lanes[curLane->getLaneIndex() - 1], segmentIndex - 1, interval, nextSpeed) &&
                        checkSegment(
                                &lanes[curLane->getLaneIndex() - 1], segmentIndex + 1, interval, nextSpeed)) {
                        return 2;
                    }
                } else if (curLane->getLaneIndex() + 1 < lanes.size() &&
                           lanes[curLane->getLaneIndex() + 1].getLaneLinks()[0]->getRoadLinkType() ==
                           ((Lane *) (*controllerInfo.iDrivable))->getLaneLinks()[0]->getRoadLinkType()) {
                    if (checkSegment(&lanes[curLane->getLaneIndex() + 1], segmentIndex, interval, nextSpeed) &&
                        checkSegment(
                                &lanes[curLane->getLaneIndex() + 1], segmentIndex - 1, interval, nextSpeed) &&
                        checkSegment(
                                &lanes[curLane->getLaneIndex() + 1], segmentIndex + 1, interval, nextSpeed)) {
                        return 1;
                    }
                }
                //controllerInfo.curLane->getSegment(laneChangeInfo.segmentIndex)->tryChange = this;
            }
            if (curLane->getLength() - vehicleInfo.dis - vehicleInfo.speed * interval <
                laneChangeInfo.bufferLength * 4) {
                if (curLane->getLaneIndex() - 1 >= 0 && lanes.size() > 0 &&
                    (controllerInfo.iDrivable + 1 != controllerInfo.wholeRoute.end()) &&
                    lanes[curLane->getLaneIndex() - 1].getLaneLinks()[0]->getRoadLinkType() ==
                    ((Lane *) (*controllerInfo.iDrivable))->getLaneLinks()[0]->getRoadLinkType()) {
                    if (checkSegment(&lanes[curLane->getLaneIndex() - 1], segmentIndex, interval, nextSpeed) &&
                        checkSegment(
                                &lanes[curLane->getLaneIndex() - 1], segmentIndex - 1, interval, nextSpeed) &&
                        checkSegment(
                                &lanes[curLane->getLaneIndex() - 1], segmentIndex + 1, interval, nextSpeed)) {
                        return 2;
                    } else
                        return 3;
                } else if (curLane->getLaneIndex() + 1 < lanes.size()) {
                    if (checkSegment(&lanes[curLane->getLaneIndex() + 1], segmentIndex, interval, nextSpeed) &&
                        checkSegment(
                                &lanes[curLane->getLaneIndex() + 1], segmentIndex - 1, interval, nextSpeed) &&
                        checkSegment(
                                &lanes[curLane->getLaneIndex() + 1], segmentIndex + 1, interval, nextSpeed)) {
                        return 1;
                    } else
                        return 3;
                }
                Lane *l = (Lane *) *controllerInfo.iDrivable;
                int ij = 0;
            }
        } else if (controllerInfo.leader != nullptr && tryChange()) {
            if ((*controllerInfo.iDrivable)->getLength() - vehicleInfo.dis < 6 * laneChangeInfo.bufferLength ||
                vehicleInfo.dis < laneChangeInfo.bufferLength * 2)
                return 0;
            if (curLane->getLaneIndex() - 1 >= 0 && !lanes.empty()) {
                if (checkSegment(&lanes[curLane->getLaneIndex() - 1], segmentIndex, interval, nextSpeed) &&
                    checkSegment(
                            &lanes[curLane->getLaneIndex() - 1], segmentIndex - 1, interval, nextSpeed) && checkSegment(
                        &lanes[curLane->getLaneIndex() - 1], segmentIndex + 1, interval, nextSpeed)) {
                    return 2;
                }
            }
            if (curLane->getLaneIndex() + 1 < lanes.size()) {
                if (checkSegment(&lanes[curLane->getLaneIndex() + 1], segmentIndex, interval, nextSpeed) &&
                    checkSegment(
                            &lanes[curLane->getLaneIndex() + 1], segmentIndex - 1, interval, nextSpeed) && checkSegment(
                        &lanes[curLane->getLaneIndex() + 1], segmentIndex + 1, interval, nextSpeed)) {
                    return 1;
                }
            }
            //controllerInfo.curLane->getSegment(laneChangeInfo.segmentIndex)->tryChange = this;
        }
        //if (controllerInfo.curLane->getSegment(laneChangeInfo.segmentIndex)->tryChange == this)
        //    controllerInfo.curLane->getSegment(laneChangeInfo.segmentIndex)->tryChange = nullptr;
        return 0;
    }

    double Vehicle::findNextGap(double dis, Lane *lane, int segmentIndex) {
        auto &vehs = lane->getSegment(segmentIndex)->getVehicles();
        double last = 0.0, first = lane->getLength() / lane->getSegments().size();
        for (auto &veh : vehs) {
            if ((*veh)->getDistance() < dis && (*veh)->getDistance() > last)
                last = (*veh)->getDistance();
            if ((*veh)->getDistance() > dis && (*veh)->getDistance() < first)
                first = (*veh)->getDistance();
        }
        return (first - last);
    }

    bool Vehicle::tryChange() {
        double gap = controllerInfo.leader->getDistance() - vehicleInfo.dis;
        if (vehicleInfo.speed > controllerInfo.leader->getSpeed()) {
            if (gap / (vehicleInfo.speed - controllerInfo.leader->getSpeed()) < 5.0)
                return true;
            return false;
        } else if (vehicleInfo.speed == 0) {
            double nextGap = 0.0;
            if (controllerInfo.curLane->getLaneIndex() > 0)
                nextGap = max2double(nextGap, findNextGap(vehicleInfo.dis,
                                                          &controllerInfo.curLane->getBelongRoad()->getLanes()[
                                                                  controllerInfo.curLane->getLaneIndex() - 1],
                                                          laneChangeInfo.segmentIndex));
            if (controllerInfo.curLane->getLaneIndex() < controllerInfo.curLane->getBelongRoad()->getLanes().size() - 1)
                nextGap = max2double(nextGap, findNextGap(vehicleInfo.dis,
                                                          &controllerInfo.curLane->getBelongRoad()->getLanes()[
                                                                  controllerInfo.curLane->getLaneIndex() + 1],
                                                          laneChangeInfo.segmentIndex));
            if (gap + getMinGap() < nextGap)
                return true;
        }
        return false;
    }

    void Vehicle::finishChanging() {
        if ((*controllerInfo.iDrivable)->isLane()) {
            if (controllerInfo.iDrivable + 1 == controllerInfo.wholeRoute.end())
                laneChangeInfo.changed = false;
            else
                laneChangeInfo.changed = isLaneChanged(controllerInfo.curLane,
                                                       (LaneLink *) *(controllerInfo.iDrivable + 1));
        }
        laneChangeInfo.partnerType = 0;
        laneChangeInfo.offset = 0;
    }

    void Vehicle::setLane(Lane *nextLane) {
        this->controllerInfo.curLane = nextLane;
    }

    bool Vehicle::isChanged() {
        return laneChangeInfo.changed;
    }

    Drivable *Vehicle::getCurDrivable() {
        if ((*controllerInfo.iDrivable)->isLane())
            return controllerInfo.curLane;
        else
            return controllerInfo.curLaneLink;

    }

    Lane *Vehicle::getCurLane() {
        return controllerInfo.curLane;
    }

    bool Vehicle::isLaneChanged(Lane *curLane, LaneLink *originLaneLink) {
        if (curLane->getLaneLinks().empty())
            return false;
        for (auto &laneLink : curLane->getLaneLinks())
            if (laneLink->getRoadLinkType() == originLaneLink->getRoadLinkType())
                return false;
        return true;
    }
}
