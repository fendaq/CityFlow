#include "vehicle/router.h"
#include "utility/utility.h"
#include "flow/route.h"
#include "roadnet/roadnet.h"
#include <random>
#include <cassert>
#include <limits>
#include <cstdlib>

namespace CitySimulator {

    std::vector<Drivable *> Router::getRoutes(const Route *route, std::mt19937 &rnd) {
        std::vector<Drivable *> routes;
        std::vector<Road *> roads = route->getRoute();
        Lane *curLane = nullptr, *candidateLane = nullptr;
        bool connected = false;
        int laneDiff = std::numeric_limits<int>::max();
        LaneLink *candidateLaneLink = nullptr;

        //initial
        Road *r1 = roads[0];
        Road *r2 = roads[1];
        std::vector<Lane> &lanes = r1->getLanes();
        std::vector<int> randoms = generateRandomIndices(lanes.size(), rnd);
        for (size_t i = 0; i < lanes.size(); ++i)
            if (!lanes[randoms[i]].getLaneLinksToRoad(r2).empty()) {
                curLane = &lanes[randoms[i]];
                connected = true;
                break;
            }
        assert(connected);
        routes.emplace_back(curLane);

        for (size_t i = 1; i < roads.size(); ++i) {
            std::vector<LaneLink *> laneLinks = curLane->getLaneLinksToRoad(roads[i]);
            connected = false;
            laneDiff = std::numeric_limits<int>::max();
            for (size_t j = 0; j < laneLinks.size(); ++j) {
                Lane *nextLane = laneLinks[j]->getEndLane();
                if (i == roads.size() - 1 || !nextLane->getLaneLinksToRoad(roads[i + 1]).empty()) {
                    if (std::abs(int (nextLane->getBelongRoad()->getLanes().size() * curLane->getLaneIndex() -
                                 curLane->getBelongRoad()->getLanes().size() * nextLane->getLaneIndex())) < laneDiff) {
                        laneDiff = std::abs(int (nextLane->getBelongRoad()->getLanes().size() * curLane->getLaneIndex() -
                                            curLane->getBelongRoad()->getLanes().size() * nextLane->getLaneIndex()));
                        candidateLaneLink = laneLinks[j];
                        candidateLane = nextLane;
                        connected = true;
                    }
                }
            }
            assert(connected);
            curLane = candidateLane;
            routes.push_back(candidateLaneLink);
            routes.push_back(candidateLane);
        }

        return routes;
    }

    Drivable *Router::getNextDrivable(const std::vector<Drivable *> *route, double dis,
                                      std::vector<Drivable *>::iterator iDrivable, Lane *curLane,
                                      LaneLink *curLaneLink) {
        if ((*iDrivable)->isLaneLink()) {
            return curLaneLink->getEndLane();
        }
        if ((*iDrivable)->isLane() && iDrivable + 1 != route->end() && iDrivable + 2 != route->end() &&
            curLane->getLaneIndex() != ((Lane *) *iDrivable)->getLaneIndex()) {
            bool connected = false;
            auto &laneLinks = curLane->getLaneLinks();
            int laneDiff = std::numeric_limits<int>::max();
            LaneLink * candidateLaneLink = laneLinks[0];
            for (auto & laneLink : laneLinks) {
                Lane *nextLane = laneLink->getEndLane();
                bool check = false;
                if (nextLane->getBelongRoad() != ((Lane *) * (iDrivable + 2))->getBelongRoad())
                    continue;
                if (!nextLane->getLaneLinks().empty() && (iDrivable + 3 != route->end()))
                    for (auto & laneLink: nextLane->getLaneLinks())
                        if (laneLink->getRoadLinkType() == ((LaneLink *)(*(iDrivable + 3)))->getRoadLinkType())
                            check = true;
                if (!check && (iDrivable + 3 != route->end()))
                    continue;
                if (std::abs(int(nextLane->getBelongRoad()->getLanes().size() * curLane->getLaneIndex() -
                                 curLane->getBelongRoad()->getLanes().size() * nextLane->getLaneIndex())) < laneDiff) {
                        laneDiff = std::abs(int(nextLane->getBelongRoad()->getLanes().size() * curLane->getLaneIndex() -
                                            curLane->getBelongRoad()->getLanes().size() * nextLane->getLaneIndex()));
                        candidateLaneLink = laneLink;
                        connected = true;
                    }
                }
                if (connected) {
                    if (dis < candidateLaneLink->getLength()) {
                        return candidateLaneLink;
                    } else {
                        return candidateLaneLink->getEndLane();
                    }
                } else
                    int ij = 0;
            }
         else {
            if (iDrivable + 1 != route->end() && dis > (*(iDrivable + 1))->getLength()) {
                return ((Lane *)*(iDrivable + 2));
            } else {
                return ((LaneLink *) *(iDrivable + 1));
            }
        }
        return *iDrivable;
    }
}
