#include "junction.h"

#include <CGAL/intersections.h>
#include <CGAL/Cartesian.h>

#include <optional>

using namespace CGAL;
typedef Cartesian<double>  Kernel;

namespace RoadRunner
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
                hdgInto = odr::negate(incoming.forward);
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

    odr::Line2D Junction::CalcBoundary() const
    {
        odr::Line2D result;
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
        
        // Discretize all connecting road boundaries to intersect with
        // ROADRUNNERTODO: no need to include all connecting roads
        std::vector<Segment_2<Kernel>> allConnectingRoadBounds;
        for (const auto& connecting : connectingRoads)
        {
            auto borders = connecting->generated.get_road_boundary(0.5);
            
            if (!borders.first.empty())
            {
                for (int i = 0; i < borders.first.size() - 1; ++i)
                {
                    CGAL::Point_2<Kernel> p1(borders.first[i][0], borders.first[i][1]);
                    CGAL::Point_2<Kernel> p2(borders.first[i + 1][0], borders.first[i + 1][1]);
                    allConnectingRoadBounds.push_back(Segment_2<Kernel>(p1, p2));
                }
            }

            if (!borders.second.empty())
            {
                for (int i = 0; i < borders.second.size() - 1; ++i)
                {
                    CGAL::Point_2<Kernel> p1(borders.second[i][0], borders.second[i][1]);
                    CGAL::Point_2<Kernel> p2(borders.second[i + 1][0], borders.second[i + 1][1]);
                    allConnectingRoadBounds.push_back(Segment_2<Kernel>(p1, p2));
                }
            }
        }

        for (int i = 0; i != sortedConnInfo.size(); ++i)
        {
            const auto& info1 = sortedConnInfo[i].info;
            auto contactSection1 = info1.road.lock()->generated.get_road_boundary(1); // ROADRUNNERTODO: slow
            const auto& p13D = info1.contact == odr::RoadLink::ContactPoint_End ? contactSection1.second.back() :
                contactSection1.first.front();
            const odr::Vec2D p1 { p13D[0], p13D[1] };
            const auto& info2 = sortedConnInfo[(i + 1) % sortedConnInfo.size()].info;
            auto contactSection2 = info2.road.lock()->generated.get_road_boundary(1);
            const auto& p23D = info2.contact == odr::RoadLink::ContactPoint_End ?
                contactSection2.first.back() : contactSection2.second.front();
            const odr::Vec2D p2{ p23D[0], p23D[1] };
            result.push_back(p1);

            spdlog::trace("{},{} on road {} | {},{} on road {}", p1[0], p1[1], info1.road.lock()->ID(),
                p2[0], p2[1], info2.road.lock()->ID());

            const auto dist = odr::euclDistance(p1, p2);
            const auto dir = odr::mut(1 / dist, odr::sub(p2, p1));
            const auto outwardsDir = odr::Vec2D{ dir[1], -dir[0] };
            const auto tan = Direction_2< Kernel>(outwardsDir[0], outwardsDir[1]);
            int nProbs = std::ceil(dist / 1.0);
            for (int j = 0; j != nProbs; ++j)
            {
                auto ratio = static_cast<double>(j + 1) / (nProbs + 1);
                auto probPos = odr::add(odr::mut(ratio, p2), odr::mut(1 - ratio, p1));
                Point_2 <Kernel> probP(probPos[0], probPos[1]);
                Line_2< Kernel > probLine(probP, tan);

                odr::Vec2D outmostPoint;
                double farthestDist = -DBL_MAX;

                for (const auto& roadBound : allConnectingRoadBounds)
                {
                    Object result = intersection(roadBound, probLine);
                    if (const CGAL::Point_2<Kernel>* pm = object_cast<Point_2<Kernel>>(&result))
                    {
                        odr::Vec2D iPoint{ pm->x(), pm->y() };
                        auto prob2I = odr::sub(iPoint, probPos);
                        auto signedDistance = odr::dot(prob2I, outwardsDir);
                        if (signedDistance > farthestDist)
                        {
                            outmostPoint = iPoint;
                            farthestDist = signedDistance;
                        }
                    }
                }
                result.push_back(outmostPoint);
            }
            result.push_back(p2);
        }
        
        return result;
    }


    std::vector<odr::Line2D> DirectJunction::CalcCavity() const
    {
        auto interfaceProvider = InterfaceProvider();
        auto interfaceProvideRoad = interfaceProvider->road.lock();

        std::map<int, ConnectionInfo> linkedIDToInfo;
        for (auto info : formedFrom)
        {
            int id = std::stoi(info.road.lock()->ID());
            linkedIDToInfo.emplace(id, info);
        }

        std::map<int, std::pair<int, int>> linkedRoadToLaneRange;
        std::set<std::pair<int, int>> laneRanges;
        for (auto id_conn : generated.id_to_connection)
        {
            auto connectingRoad = std::stoi(id_conn.second.connecting_road);
            int maxLinkedLane = INT_MIN; // left-most on interface provider
            int minLinkedLane = INT_MAX; // right-most on interface provider
            for (auto laneLink : id_conn.second.lane_links)
            {
                maxLinkedLane = std::max(maxLinkedLane, laneLink.from);
                minLinkedLane = std::min(minLinkedLane, laneLink.from);
            }
            auto range = std::make_pair(maxLinkedLane, minLinkedLane);
            linkedRoadToLaneRange.emplace(connectingRoad, range);
            laneRanges.emplace(range);
        }

        std::vector<std::pair<int, int>> sortedLinkedRoad; // (rank, linked road ID)
        
        for (auto id_range : linkedRoadToLaneRange)
        {
            int rank = std::distance(laneRanges.begin(), laneRanges.find(id_range.second));
            sortedLinkedRoad.emplace_back(std::make_pair(rank, id_range.first));
        }
        std::sort(sortedLinkedRoad.begin(), sortedLinkedRoad.end());

        std::vector<int> numAtRank(laneRanges.size(), 0);
        for (auto rank_road : sortedLinkedRoad)
        {
            numAtRank[rank_road.first]++;
        }

        std::vector<int> strictlySortedLinkedRoad; // right to left
        for (int i = 0; i != sortedLinkedRoad.size();)
        {
            int rank = sortedLinkedRoad[i].first;
            int roadsAtRank = numAtRank[rank];
            if (roadsAtRank == 1)
            {
                // No need to discriminate
                // ROADRUNNERTODO: still need cavity polygon
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
                // ROADRUNNERTODO: convert to std::sort
                for (int a = discriminateBegin; a < discriminateEnd - 1; ++a)
                {
                    for (int b = a + 1; b < discriminateEnd; ++b)
                    {
                        int aID = sortedLinkedRoad[a].second;
                        int bID = sortedLinkedRoad[b].second;
                        auto infoA = linkedIDToInfo.at(aID);
                        auto infoB = linkedIDToInfo.at(bID);

                        // Compute min intersect S1 between A's left & B's right (up to min(lenA, lenB))
                        // Compute min intersect S2 between A's right & B's left (up to min(lenA, lenB))
                        // If S1 < S2, A on right of B
                        double sLA, sRB, sRA, sLB;
                        bool aLbR = bordersIntersect(interfaceProvider->contact, infoA, 1, infoB, -1, sLA, sRB);
                        bool aRbL = bordersIntersect(interfaceProvider->contact, infoA, -1, infoB, 1, sRA, sLB);
                        if (!aLbR && !aRbL)
                        {
                            spdlog::error("Overlap zone between roads {} & {} can't be determined", 
                                infoA.road.lock()->ID(), infoB.road.lock()->ID());
                        }

                        if ((infoA.contact == odr::RoadLink::ContactPoint_Start) == (sRA < sLA))
                        {
                            /*A is to the left to B*/
                            std::swap(strictlySortedLinkedRoad[a], strictlySortedLinkedRoad[b]);
                        }
                    }
                }
            }
            i += roadsAtRank;
        }

        for (auto road : strictlySortedLinkedRoad)
        {
            spdlog::trace("Sort result (relative to InterfaceProvider R->L): {}", road);
        }

        int interfaceSign = interfaceProvider->contact == odr::RoadLink::ContactPoint_End ? 1 : -1;
        std::vector<odr::Line2D> cavityPolygons;
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
                int aSide = interfaceProvider->contact == infoA.contact ? -1 : 1;
                int bSide = interfaceProvider->contact == infoB.contact ? 1 : -1;
                int aStepDir = infoA.contact == odr::RoadLink::ContactPoint_Start ? 1 : -1;
                int bStepDir = infoB.contact == odr::RoadLink::ContactPoint_Start ? 1 : -1;

                odr::Line2D aSideLine, bSideLine;
                while (0 <= iSA && iSA <= roadA->Length()
                    && 0 <= iSB && iSB <= roadB->Length())
                {
                    auto vA = roadA->generated.get_boundary_xy(aSide, iSA);
                    auto vB = roadB->generated.get_boundary_xy(bSide, iSB);
                    aSideLine.push_back(vA);
                    bSideLine.push_back(vB);
                    if (odr::euclDistance(vA, vB) > LaneWidth)
                    {
                        break;
                    }
                    iSA += aStepDir * vertexStep;
                    iSB += bStepDir * vertexStep;
                }

                aSideLine.insert(aSideLine.end(), bSideLine.rbegin(), bSideLine.rend());
                cavityPolygons.push_back(aSideLine);
            }
        }
        return cavityPolygons;
    }

    bool DirectJunction::bordersIntersect(odr::RoadLink::ContactPoint interfaceProviderContact,
        ConnectionInfo infoA, int sideA,
        ConnectionInfo infoB, int sideB,
        double& outSA, double& outSB)
    {
        int aSide = interfaceProviderContact == infoA.contact ? -sideA : sideA;
        int bSide = interfaceProviderContact == infoB.contact ? -sideB : sideB;

        auto roadA = infoA.road.lock();
        auto lenA = roadA->Length();
        auto commonY = roadA->RefLine().get_grad_xy(
            infoA.contact == odr::RoadLink::ContactPoint_Start ? 0 : lenA);
        if (infoA.contact == odr::RoadLink::ContactPoint_End)
        {
            commonY = odr::negate(commonY);
        }
        commonY = odr::normalize(commonY);
        odr::Vec2D commonX{ -commonY[1], commonY[0] };
        auto roadB = infoB.road.lock();
        auto lenB = roadB->Length();

        double deltaA = 0;
        double deltaB = 0;
        bool aStep = true, bStep = true;
        double projXA, projYA, projXB, projYB;
        odr::Vec2D posOnA, posOnB;

        std::optional<bool> aSmallA;
        
        while (deltaA < lenA && deltaB < lenB)
        {
            if (aStep)
            {
                outSA = infoA.contact == odr::RoadLink::ContactPoint_Start ? deltaA : lenA - deltaA;
                posOnA = roadA->generated.get_boundary_xy(aSide, outSA);
                projYA = odr::dot(posOnA, commonY);
                projXA = odr::dot(posOnA, commonX);
            }
            if (bStep)
            {
                outSB = infoB.contact == odr::RoadLink::ContactPoint_Start ? deltaB : lenB - deltaB;
                posOnB = roadB->generated.get_boundary_xy(bSide, outSB);
                projYB = odr::dot(posOnB, commonY);
                projXB = odr::dot(posOnB, commonX);
            }

            bool nowASmallX = projXA < projXB;
            if (aSmallA.has_value())
            {
                if (aSmallA.value() != nowASmallX)
                {
                    // X flipped! we are done
                    return true;
                }
            }
            else
            {
                // First iteration
                aSmallA.emplace(nowASmallX);
            }

            if (projYA < projYB)
            {
                aStep = true;
                bStep = false;
                deltaA += 0.1;
            }
            else
            {
                aStep = false;
                bStep = true;
                deltaB += 0.1;
            }
        }

        return false;
    }
};
