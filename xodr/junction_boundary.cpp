#include "junction.h"
#include "polyline.h"
#include "constants.h"

#include <CGAL/intersections.h>
#include <CGAL/Cartesian.h>

#include <optional>
#include <spdlog/spdlog.h>

using namespace CGAL;
typedef Cartesian<double>  Kernel;

namespace LM
{
    struct ContactInfoWithCoordinate
    {
        ConnectionInfo info;
        odr::Vec2D pos;
        odr::Vec2D hdgInto;

        ContactInfoWithCoordinate(const ConnectionInfo& connInfo):
            info(connInfo)
        {
            RoadEndpoint incoming, outgoing;
            RoadEndpoint::FromConnInfo(connInfo, incoming, outgoing);
            if (incoming.contact != odr::RoadLink::ContactPoint_None)
            {
                pos = incoming.origin;
                hdgInto = incoming.forward;
            }
            else
            {
                pos = outgoing.origin;
                hdgInto = odr::negate(outgoing.forward);
            }
        }

        bool operator==(const ContactInfoWithCoordinate& other) const
        {
            return info == other.info;
        }

        bool operator!=(const ContactInfoWithCoordinate& other) const
        {
            return info != other.info;
        }

        bool operator<(const ContactInfoWithCoordinate& rhs) const
        {
            return info < rhs.info;
        }
    };

    typedef std::tuple<double, double, double, double> SerializedSegment_2;
    SerializedSegment_2 Serialize(Segment_2<Kernel> s)
    {
        return std::make_tuple(s.source().x(), s.source().y(), s.target().x(), s.source().y());
    }

    std::vector<odr::BoundarySegment> Junction::CalcBoundary() const
    {
        std::vector<odr::BoundarySegment> result;
        if (formedFrom.size() < 2)
        {
            return result;
        }
        std::vector<ContactInfoWithCoordinate> formedFromWithCoordinate;
        for (const auto& connInfo : formedFrom)
        {
            formedFromWithCoordinate.push_back(ContactInfoWithCoordinate(connInfo));
        }

        ContactInfoWithCoordinate origin = formedFromWithCoordinate.back();
        std::vector<ContactInfoWithCoordinate> sortedConnInfo = { origin };
        formedFromWithCoordinate.pop_back();

        while (!formedFromWithCoordinate.empty())
        {
            double rightMostTurnAngle = M_PI; // right-most turn: min angle
            int rightMostIndex = -1;

            for (int i = 0; i != formedFromWithCoordinate.size(); ++i)
            {
                const auto& ff = formedFromWithCoordinate[i];
                auto turnAngle = odr::angle(origin.hdgInto, odr::sub(ff.pos, origin.pos));
                if (turnAngle < rightMostTurnAngle)
                {
                    rightMostTurnAngle = turnAngle;
                    rightMostIndex = i;
                }
            }

            assert(rightMostIndex >= 0);
            sortedConnInfo.push_back(formedFromWithCoordinate[rightMostIndex]);
            formedFromWithCoordinate.erase(formedFromWithCoordinate.begin() + rightMostIndex);
        }
        
        // Discretize (roughly) all connecting road boundaries inside junction
        std::vector<Segment_2<Kernel>> allConnectingRoadBounds;
        std::unordered_map<Segment_2<Kernel>, std::pair<int, odr::Road>> roughSegmentToOwner;
        for (const auto& connecting : connectingRoads)
        {
            for (int side : {-1, 1})
            {
                auto border = connecting->generated.get_road_boundary(side, 0.5);
                if (!border.empty())
                {
                    for (int i = 0; i < border.size() - 1; ++i)
                    {
                        CGAL::Point_2<Kernel> p1(border[i][0], border[i][1]);
                        CGAL::Point_2<Kernel> p2(border[i + 1][0], border[i + 1][1]);
                        Segment_2<Kernel> segment(p1, p2);
                        allConnectingRoadBounds.push_back(segment);
                        roughSegmentToOwner.emplace(segment, std::make_pair(side, connecting->generated));
                    }
                }
            }
        }

        for (int i = 0; i != sortedConnInfo.size(); ++i)
        {
            const auto& info1 = sortedConnInfo[i].info;
            auto road1 = info1.road.lock();

            odr::Vec2D p1;
            if (info1.contact == odr::RoadLink::ContactPoint_Start)
            {
                p1 = road1->generated.get_boundary_xy(1, 0);
                result.emplace_back(odr::BoundarySegment{ road1->ID(), -1, 0, 0, odr::BoundarySegmentType::Joint });
            }
            else
            {
                p1 = road1->generated.get_boundary_xy(-1, road1->Length());
                result.emplace_back(odr::BoundarySegment{ road1->ID(), 1, road1->Length(), road1->Length(), odr::BoundarySegmentType::Joint });
            }

            const auto& info2 = sortedConnInfo[(i + 1) % sortedConnInfo.size()].info;
            auto road2 = info2.road.lock();

            odr::Vec2D p2;
            if (info2.contact == odr::RoadLink::ContactPoint_Start)
            {
                p2 = road2->generated.get_boundary_xy(-1, 0);
            }
            else
            {
                p2 = road2->generated.get_boundary_xy(1, road2->Length());
            }

            spdlog::trace("{},{} on road {} | {},{} on road {}", p1[0], p1[1], info1.road.lock()->ID(),
                p2[0], p2[1], info2.road.lock()->ID());

            const auto dist = odr::euclDistance(p1, p2);
            const auto dir = odr::mut(1 / dist, odr::sub(p2, p1));
            const auto outwardsDir = odr::Vec2D{ dir[1], -dir[0] };
            const auto tan = Direction_2< Kernel>(outwardsDir[0], outwardsDir[1]);
            int nProbs = std::ceil(dist / 1.0);

            std::set<std::string> fineConnectingRoadIDs;
            std::vector<Segment_2<Kernel>> fineConnectingRoadBounds;
            std::map<SerializedSegment_2, std::tuple<std::string, int, double, double>> fineConnectingBoundsToOrigin; // trace back to origin

            // Rough pass
            for (int j = 0; j != nProbs; ++j)
            {
                auto ratio = static_cast<double>(j + 1) / (nProbs + 1);
                auto probPos = odr::add(odr::mut(ratio, p2), odr::mut(1 - ratio, p1));
                Point_2 <Kernel> probP(probPos[0], probPos[1]);
                Line_2< Kernel > probLine(probP, tan);

                double farthestDist = -DBL_MAX;
                std::optional<std::pair<int, odr::Road>> furthestRoadAndSide; // road to fine-grain compute

                for (const auto& roadBound : allConnectingRoadBounds)
                {
                    Object result = intersection(roadBound, probLine);
                    if (const CGAL::Point_2<Kernel>* pm = object_cast<Point_2<Kernel>>(&result))
                    {
                        auto intersectingInfo = roughSegmentToOwner.at(roadBound);
                        auto connectingRoadID = intersectingInfo.second.id;
                        odr::Vec2D iPoint{ pm->x(), pm->y() };
                        auto prob2I = odr::sub(iPoint, probPos);
                        auto signedDistance = odr::dot(prob2I, outwardsDir);
                        if (signedDistance > farthestDist)
                        {
                            furthestRoadAndSide.emplace(intersectingInfo);
                            farthestDist = signedDistance;
                        }
                    }
                }

                if (furthestRoadAndSide.has_value())
                {
                    auto connectingRoadID = furthestRoadAndSide->second.id;
                    if (fineConnectingRoadIDs.find(connectingRoadID) == fineConnectingRoadIDs.end()) 
                    {
                        fineConnectingRoadIDs.emplace(connectingRoadID);

                        const auto connectingRoad = furthestRoadAndSide->second;
                        int nPoints = std::ceil(connectingRoad.length / 0.1);
                        for (int i = 0; i < nPoints; ++i)
                        {
                            double s1 = static_cast<double>(i) / nPoints * connectingRoad.length;
                            double s2 = static_cast<double>(i + 1) / nPoints * connectingRoad.length;
                            auto p1 = connectingRoad.get_boundary_xy(furthestRoadAndSide->first, s1);
                            auto p2 = connectingRoad.get_boundary_xy(furthestRoadAndSide->first, s2);
                            CGAL::Point_2<Kernel> cp1(p1[0], p1[1]), cp2(p2[0], p2[1]);
                            auto segment_2 = Segment_2<Kernel>(cp1, cp2);
                            fineConnectingRoadBounds.push_back(segment_2);
                            fineConnectingBoundsToOrigin.emplace(Serialize(segment_2), std::make_tuple(
                                furthestRoadAndSide->second.id, furthestRoadAndSide->first, s1, s2));
                        }
                    }
                }
            }

            // Fine pass
            odr::BoundarySegment currOutmostRoad;
            for (int j = 0; j != nProbs; ++j)
            {
                auto ratio = static_cast<double>(j + 1) / (nProbs + 1);
                auto probPos = odr::add(odr::mut(ratio, p2), odr::mut(1 - ratio, p1));
                Point_2 <Kernel> probP(probPos[0], probPos[1]);
                Line_2< Kernel > probLine(probP, tan);

                //odr::Vec2D outmostPoint;
                double farthestDist = -DBL_MAX;
                std::string probeOutmostRoad;
                double sOnOutmostRoad;
                int sideOnOutmostRoad;
                for (const auto& roadBound : fineConnectingRoadBounds)
                {
                    Object result = intersection(roadBound, probLine);
                    if (const CGAL::Point_2<Kernel>* pm = object_cast<Point_2<Kernel>>(&result))
                    {
                        odr::Vec2D iPoint{ pm->x(), pm->y() };
                        auto prob2I = odr::sub(iPoint, probPos);
                        auto signedDistance = odr::dot(prob2I, outwardsDir);
                        if (signedDistance > farthestDist)
                        {
                            //outmostPoint = iPoint;
                            farthestDist = signedDistance;
                            auto detail = fineConnectingBoundsToOrigin.at(Serialize(roadBound));
                            probeOutmostRoad = std::get<0>(detail);
                            auto distFromSource = std::sqrt(CGAL::squared_distance(*pm, roadBound.source()));
                            auto distFromTarget = std::sqrt(CGAL::squared_distance(*pm, roadBound.target()));
                            sideOnOutmostRoad = std::get<1>(detail);
                            double sOfSource = std::get<2>(detail);
                            double sOfTarget = std::get<3>(detail);
                            sOnOutmostRoad = (sOfSource * distFromTarget + sOfTarget * distFromSource) /
                                (distFromSource + distFromTarget);
                        }
                    }
                }

                if (!probeOutmostRoad.empty())
                {
                    if (probeOutmostRoad != currOutmostRoad.road)
                    {
                        // outmost shifts, record history
                        if (!currOutmostRoad.road.empty())
                        {
                            result.push_back(currOutmostRoad);
                        }
                        currOutmostRoad.road = probeOutmostRoad;
                        currOutmostRoad.side = sideOnOutmostRoad;
                        currOutmostRoad.sBegin = sOnOutmostRoad;
                        currOutmostRoad.sEnd = sOnOutmostRoad;
                    }
                    else
                    {
                        // outmost continues
                        currOutmostRoad.sEnd = sOnOutmostRoad;
                    }
                }
                else
                {
                    spdlog::warn("No intersection found in fine path between {} and {}", road1->ID(), road2->ID());
                }
            }

            if (!currOutmostRoad.road.empty())
            {
                result.push_back(currOutmostRoad);
            }
            // TODO: add joint
        }
        
        return result;
    }

    std::vector<odr::BoundarySegment> DirectJunction::CalcCavity()
    {
        auto interfaceProvider = InterfaceProvider();
        auto interfaceProvideRoad = interfaceProvider->road.lock();

        typedef std::pair<int, odr::RoadLink::ContactPoint> roadID_Contact;
        typedef std::pair<int, int> laneRange;
        typedef int rank_t;

        std::map<roadID_Contact, ConnectionInfo> linkedIDToInfo;
        for (auto info : formedFrom)
        {
            int id = std::stoi(info.road.lock()->ID());
            linkedIDToInfo.emplace(std::make_pair(id, info.contact), info);
        }

        std::map<roadID_Contact, laneRange> linkedRoadToLaneRange;
        std::set<laneRange> laneRanges;
        for (auto id_conn : generated.id_to_connection)
        {
            auto connectingID = std::stoi(id_conn.second.connecting_road);
            int maxLinkedLane = INT_MIN; // left-most on interface provider
            int minLinkedLane = INT_MAX; // right-most on interface provider
            for (auto laneLink : id_conn.second.lane_links)
            {
                maxLinkedLane = std::max(maxLinkedLane, laneLink.from);
                minLinkedLane = std::min(minLinkedLane, laneLink.from);
            }
            roadID_Contact idAndContact = std::make_pair(connectingID, 
                static_cast<odr::RoadLink::ContactPoint>(id_conn.second.contact_point));
            laneRange range = std::make_pair(maxLinkedLane, minLinkedLane);
            linkedRoadToLaneRange.emplace(idAndContact, range);
            laneRanges.emplace(range);
        }

        std::vector<std::pair<rank_t, roadID_Contact>> sortedLinkedRoad; // (rank, linked road ID)
        
        for (auto id_range : linkedRoadToLaneRange)
        {
            rank_t rank = std::distance(laneRanges.begin(), laneRanges.find(id_range.second));
            sortedLinkedRoad.emplace_back(std::make_pair(rank, id_range.first));
        }
        std::sort(sortedLinkedRoad.begin(), sortedLinkedRoad.end());

        std::vector<rank_t> numAtRank(laneRanges.size(), 0);
        for (auto rank_road : sortedLinkedRoad)
        {
            numAtRank[rank_road.first]++;
        }

        std::vector<roadID_Contact> strictlySortedLinkedRoad; // right to left
        for (int i = 0; i != sortedLinkedRoad.size();)
        {
            rank_t rank = sortedLinkedRoad[i].first;
            int roadsAtRank = numAtRank[rank];
            if (roadsAtRank == 1)
            {
                // No need to discriminate
                strictlySortedLinkedRoad.push_back(sortedLinkedRoad[i].second);
            }
            else
            {
                int discriminateBegin = i;
                int discriminateEnd = i + roadsAtRank;
                
                // First, add everyone in random order
                for (int a = discriminateBegin; a != discriminateEnd; ++a)
                {
                    strictlySortedLinkedRoad.push_back(sortedLinkedRoad[a].second);
                }

                // Second, sort pairwise
                std::sort(strictlySortedLinkedRoad.begin() + discriminateBegin, 
                    strictlySortedLinkedRoad.begin() + discriminateEnd,
                    [&linkedIDToInfo, &interfaceProvider](roadID_Contact aID, roadID_Contact bID)
                    {
                        auto infoA = linkedIDToInfo.at(aID);
                        auto infoB = linkedIDToInfo.at(bID);
                        // Compute min intersect S1 between A's left & B's right (up to min(lenA, lenB))
                        // Compute min intersect S2 between A's right & B's left (up to min(lenA, lenB))
                        // If S1 < S2, A on right of B
                        double sLA, sRB, sRA, sLB;
                        bool aLbR = bordersIntersect(interfaceProvider->contact, infoA, 1, infoB, -1, sLA, sRB);
                        bool aRbL = bordersIntersect(interfaceProvider->contact, infoA, -1, infoB, 1, sRA, sLB);
                        if (aLbR == aRbL)
                        {
                            if (aLbR)
                            {
                                return (infoA.contact == odr::RoadLink::ContactPoint_Start) != (sRA < sLA);
                            }
                            spdlog::error("Overlap zone between roads {} & {} can't be determined. Neither boundary pair intersects.",
                                infoA.road.lock()->ID(), infoB.road.lock()->ID());
                            return false; // weak ordering
                        }
                        return aLbR;
                    });
            }
            i += roadsAtRank;
        }

        for (auto road : strictlySortedLinkedRoad)
        {
            spdlog::trace("Sort result (relative to InterfaceProvider R->L): {} {}", road.first, 
                road.second == odr::RoadLink::ContactPoint_Start ? "start" : "end");
        }

        std::vector<odr::BoundarySegment> rtn;
        if (strictlySortedLinkedRoad.empty())
        {
            return rtn;
        }

        int interfaceSign = interfaceProvider->contact == odr::RoadLink::ContactPoint_End ? 1 : -1;
        const double vertexStep = 0.1;
        for (int i = 0; i < strictlySortedLinkedRoad.size() - 1; ++i)
        {
            auto infoA = linkedIDToInfo.at(strictlySortedLinkedRoad[i]);
            auto infoB = linkedIDToInfo.at(strictlySortedLinkedRoad[i + 1]);
            auto roadA = infoA.road.lock();
            auto roadB = infoB.road.lock();
            auto rangeA = linkedRoadToLaneRange.at(strictlySortedLinkedRoad[i]);
            auto rangeB = linkedRoadToLaneRange.at(strictlySortedLinkedRoad[i + 1]);
            auto maxA = rangeA.first;
            auto minB = rangeB.second;

            double iSA, iSB;
            bool cavityFound = false;
            int aSide = interfaceProvider->contact == infoA.contact ? -1 : 1;
            int bSide = interfaceProvider->contact == infoB.contact ? 1 : -1;
            if (minB == maxA)
            {
                cavityFound = bordersIntersect(interfaceProvider->contact, infoA, 1,
                    infoB, -1, iSA, iSB);
            }
            else
            {
                // I point right at the interface
                auto aNearB = roadA->generated.get_boundary_xy(
                    interfaceSign * (infoA.contact == odr::RoadLink::ContactPoint_Start ? 1 : -1),
                    infoA.contact == odr::RoadLink::ContactPoint_Start ? 0 : infoA.road.lock()->Length());
                auto bNearA = roadB->generated.get_boundary_xy(
                    interfaceSign * (infoB.contact == odr::RoadLink::ContactPoint_Start ? -1 : 1),
                    infoB.contact == odr::RoadLink::ContactPoint_Start ? 0 : infoB.road.lock()->Length());
                if (odr::euclDistance(aNearB, bNearA) < 0.01)
                {
                    cavityFound = true;
                    iSA = infoA.contact == odr::RoadLink::ContactPoint_Start ? 0 : roadA->Length();
                    iSB = infoB.contact == odr::RoadLink::ContactPoint_Start ? 0 : roadB->Length();
                }
            }

            if (cavityFound)
            {
                roadA->generated.HideBorderMarkingForDJ(infoA.contact, aSide, iSA);
                roadB->generated.HideBorderMarkingForDJ(infoB.contact, bSide, iSB);

                const double sBeginA = iSA;
                const double sBeginB = iSB;

                int aStepDir = infoA.contact == odr::RoadLink::ContactPoint_Start ? 1 : -1;
                int bStepDir = infoB.contact == odr::RoadLink::ContactPoint_Start ? 1 : -1;

                while (0 <= iSA && iSA <= roadA->Length()
                    && 0 <= iSB && iSB <= roadB->Length())
                {
                    auto vA = roadA->generated.get_boundary_xy(aSide, iSA);
                    auto vB = roadB->generated.get_boundary_xy(bSide, iSB);
                    if (odr::euclDistance(vA, vB) > LaneWidth)
                    {
                        break;
                    }

                    auto fwdA = odr::mut(static_cast<double>(aStepDir), roadA->generated.ref_line.get_grad_xy(iSA));
                    auto fwdB = odr::mut(static_cast<double>(bStepDir), roadB->generated.ref_line.get_grad_xy(iSB));
                    auto angleA = std::abs(odr::angle(fwdA, odr::sub(vB, vA)));
                    auto angleB = std::abs(odr::angle(fwdB, odr::sub(vA, vB)));
                    auto AAhead = angleA - angleB;
                    if (AAhead > M_PI / 8)
                    {
                        // A goes too far, need B to catch up
                        iSB += bStepDir * vertexStep / 4;
                    }
                    else if (AAhead < -M_PI / 8)
                    {
                        // B goes too far, need A to catch up
                        iSA += aStepDir * vertexStep / 4;
                    }
                    else
                    {
                        iSA += aStepDir * vertexStep;
                        iSB += bStepDir * vertexStep;
                    }
                }
                iSA = std::max(0.0, std::min(roadA->Length(), iSA));
                iSB = std::max(0.0, std::min(roadB->Length(), iSB));

                odr::BoundarySegment segmentOnA{ roadA->ID(), aSide, sBeginA, iSA };
                odr::BoundarySegment segmentOnB{ roadB->ID(), bSide, sBeginB, iSB };
                rtn.push_back(segmentOnA);
                rtn.push_back(segmentOnB);
            }
            else
            {
                roadA->generated.EnableBorderMarking(infoA.contact, aSide);
                roadB->generated.EnableBorderMarking(infoB.contact, bSide);
            }
        }
        
        // Enable border marking on outmost linked roads
        {
            auto outmostA = strictlySortedLinkedRoad.front();
            auto infoA = linkedIDToInfo.at(outmostA);
            auto roadA = infoA.road.lock();
            int aSide = interfaceProvider->contact == infoA.contact ? 1 : -1;
            roadA->generated.EnableBorderMarking(infoA.contact, aSide);
        }

        {
            auto outmostB = strictlySortedLinkedRoad.back();
            auto infoB = linkedIDToInfo.at(outmostB);
            auto roadB = infoB.road.lock();
            int bSide = interfaceProvider->contact == infoB.contact ? -1 : 1;
            roadB->generated.EnableBorderMarking(infoB.contact, bSide);
        }

        // Calc overlap zone
        for (auto info : formedFrom)
        {
            const auto& linkedRoad = info.road.lock();
            const auto& lookupTable = linkedRoad->generated.boundaryHide;

            for (odr::RoadLink::ContactPoint contact :
            {odr::RoadLink::ContactPoint_Start, odr::RoadLink::ContactPoint_End})
            {
                for (int side : {-1, 1})
                {
                    auto keyLookup = std::make_pair(contact, side);
                    double overlapLength = 0;
                    if (lookupTable.find(keyLookup) != lookupTable.end())
                    {
                        overlapLength = contact == odr::RoadLink::ContactPoint_Start ? lookupTable.at(keyLookup) :
                            std::abs(linkedRoad->Length() - lookupTable.at(keyLookup));
                    }
                    if (lookupTable.find(keyLookup) != lookupTable.end() && std::abs(overlapLength) > epsilon)
                    {
                        for (auto& id_conn : generated.id_to_connection)
                        {
                            if (id_conn.second.connecting_road == linkedRoad->ID()
                                && id_conn.second.contact_point == contact)
                            {
                                std::set<int> lanesInvolved;

                                for (auto it = id_conn.second.lane_links.begin(); it != id_conn.second.lane_links.end(); ++it)
                                {
                                    lanesInvolved.emplace(it->to);
                                }
                                int overlapLaneIt = side < 0 ? *lanesInvolved.begin() : *lanesInvolved.rbegin();

                                // update lane_link xml
                                decltype(id_conn.second.lane_links) updatedLaneLink;
                                for (auto ll : id_conn.second.lane_links)
                                {
                                    if (ll.to == overlapLaneIt)
                                    {
                                        updatedLaneLink.emplace(odr::JunctionLaneLink(ll.from, ll.to, overlapLength));
                                    }
                                    else
                                    {
                                        updatedLaneLink.emplace(odr::JunctionLaneLink(ll.from, ll.to, ll.overlapZone));
                                    }
                                }
                                id_conn.second.lane_links = updatedLaneLink;
                                break;
                            }
                        }
                    }
                }
            }
        }
        return rtn;
    }

    bool DirectJunction::bordersIntersect(odr::RoadLink::ContactPoint interfaceProviderContact,
        ConnectionInfo infoA, int sideA,
        ConnectionInfo infoB, int sideB,
        double& outSA, double& outSB)
    {
        int aSide = interfaceProviderContact == infoA.contact ? -sideA : sideA;
        int bSide = interfaceProviderContact == infoB.contact ? -sideB : sideB;

        auto roadA = infoA.road.lock();
        auto roadB = infoB.road.lock();
        return LM::borderIntersect(roadA->generated, aSide, roadB->generated, bSide,
            outSA, outSB, infoA.contact, infoB.contact);
    }
};
