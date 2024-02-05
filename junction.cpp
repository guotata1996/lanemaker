#include "junction.h"
#include "curve_fitting.h"
#include <cmath> // floor, ceil
#include <map>

namespace RoadRunner
{
    odr::Junction GenerateConnections(std::vector<ConnectionInfo> connected,
        std::vector<odr::Road>& connectings)
    {
        // Collect endpoint info of each connected road
        std::vector<RoadEndpoint> incomingEndpoints, outgoingEndpoints;
        for (auto& roadAndS : connected)
        {
            odr::Road* road = roadAndS.gen;
            double meetAt = roadAndS.s;
            Road config = roadAndS.config;
            odr::LaneSection meetSection = meetAt == 0 ? 
                road->s_to_lanesection.begin()->second : road->s_to_lanesection.rbegin()->second;
            
            if (meetAt == 0)
            {
                RoadProfile rightEntrance = config.RightEntrance();
                if (rightEntrance.laneCount != 0)
                {
                    double offset = to_odr_unit(rightEntrance.offsetx2);
                    odr::Vec2D origin = road->ref_line.get_xy(meetAt, offset);
                    odr::Vec2D forward = road->ref_line.get_grad_xy(meetAt);
                    outgoingEndpoints.push_back(RoadEndpoint
                        {
                            road->id, 
                            origin,
                            forward,
                            rightEntrance.laneCount
                        });
                }
                RoadProfile leftExit = config.LeftExit();
                if (leftExit.laneCount != 0)
                {
                    double offset = to_odr_unit(leftExit.offsetx2);
                    odr::Vec2D origin = road->ref_line.get_xy(meetAt, offset);
                    odr::Vec2D forward = odr::negate(road->ref_line.get_grad_xy(meetAt));
                    incomingEndpoints.push_back(RoadEndpoint
                        {
                            road->id,
                            origin,
                            forward,
                            leftExit.laneCount,
                            roadAndS.dirSplit
                        });
                }
            }
            else
            {
                RoadProfile rightExit = config.RightExit();
                if (rightExit.laneCount != 0)
                {
                    double offset = to_odr_unit(rightExit.offsetx2);
                    odr::Vec2D origin = road->ref_line.get_xy(meetAt, offset);
                    odr::Vec2D forward = road->ref_line.get_grad_xy(meetAt);
                    incomingEndpoints.push_back(RoadEndpoint
                        {
                            road->id,
                            origin,
                            forward,
                            rightExit.laneCount,
                            roadAndS.dirSplit
                        });
                }
                RoadProfile leftEntrance = config.LeftEntrance();
                if (leftEntrance.laneCount != 0)
                {
                    double offset = to_odr_unit(leftEntrance.offsetx2);
                    odr::Vec2D origin = road->ref_line.get_xy(meetAt, offset);
                    odr::Vec2D forward = odr::negate(road->ref_line.get_grad_xy(road->length));
                    outgoingEndpoints.push_back(RoadEndpoint
                        {
                            road->id,
                            origin,
                            forward,
                            leftEntrance.laneCount
                        });
                }
            }
        }

        std::map<std::pair<std::string, std::string>, TurningGroup> turningGroups;

        // Determine incoming info for turningGroups
        for (const RoadEndpoint& incomingRoad : incomingEndpoints)
        {
            // Sort outgoing road in CW order
            std::sort(outgoingEndpoints.begin(), outgoingEndpoints.end(),
                [&incomingRoad](const RoadEndpoint& a, const RoadEndpoint& b)
            {
                odr::Vec2D va = odr::sub(a.origin, incomingRoad.origin);
                odr::Vec2D vb = odr::sub(b.origin, incomingRoad.origin);
                if (odr::norm(va) < 1e-2f)
                {
                    // A's UTurn, place in front
                    return true;
                }
                if (odr::norm(vb) < 1e-2f)
                {
                    // B's UTurn, place in front
                    return false;
                }

                va = odr::normalize(va);
                vb = odr::normalize(vb);
                return odr::crossProduct(va, incomingRoad.forward) < odr::crossProduct(vb, incomingRoad.forward);
            });

            std::vector<double> dirSplit = incomingRoad.dirSplit;
            if (dirSplit.size() != outgoingEndpoints.size() - 1)
            {
                std::vector<int8_t> outgoingLaneCounts;
                std::transform(
                    outgoingEndpoints.begin(), outgoingEndpoints.end(), outgoingLaneCounts.begin(),
                    [](const RoadEndpoint& a) {return a.nLanes; });
                dirSplit = assignIncomingLanes(incomingRoad.nLanes, outgoingLaneCounts);
                assert(dirSplit.size() == outgoingEndpoints.size() - 1);
            }
            dirSplit.insert(dirSplit.begin(), 0);
            dirSplit.push_back(incomingRoad.nLanes);

            int8_t outgoingIndex = 0;
            for (auto dirBegin = dirSplit.begin(), dirEnd = ++dirSplit.begin();
                dirEnd != dirSplit.end(); ++dirBegin, ++dirEnd, ++outgoingIndex)
            {
                const RoadEndpoint& outgoingRoad = outgoingEndpoints[outgoingIndex];
                double beginLane = *dirBegin;
                double endLane = *dirEnd;
                assert(beginLane <= endLane);
                if (beginLane == endLane)
                {
                    continue;
                }
                int8_t beginIndex = std::floor(beginLane);
                int8_t endIndex = std::ceil(endLane);
                int8_t nLane = endIndex - beginIndex;
                double turnAngle = odr::angle(incomingRoad.forward, outgoingRoad.forward);
                auto turningSemantics = TurningSemantics::No;
                if (incomingRoad.roadID == outgoingRoad.roadID)
                {
                    turningSemantics = TurningSemantics::U;
                }
                else if (turnAngle > M_PI / 4)
                {
                    turningSemantics = TurningSemantics::Left;
                }
                else if (turnAngle < -M_PI / 4)
                {
                    turningSemantics = TurningSemantics::Right;
                }
                turningGroups.emplace(
                    std::make_pair(incomingRoad.roadID, outgoingRoad.roadID),
                    TurningGroup{
                        incomingRoad.origin, incomingRoad.forward,
                        outgoingRoad.origin, outgoingRoad.forward,
                        turningSemantics,
                        beginIndex, 0,
                        nLane });
            }
        }

        // Determine outgoing lane for turningGroups
        for (const RoadEndpoint& outgoingRoad : outgoingEndpoints)
        {
            // Sort incoming road in CCW order
            std::sort(incomingEndpoints.begin(), incomingEndpoints.end(),
                [&outgoingRoad](const RoadEndpoint& a, const RoadEndpoint& b)
            {
                odr::Vec2D va = odr::sub(a.origin, outgoingRoad.origin);
                odr::Vec2D vb = odr::sub(b.origin, outgoingRoad.origin);
                if (odr::norm(va) < 1e-2f)
                {
                    // A's UTurn, place in front
                    return true;
                }
                if (odr::norm(vb) < 1e-2f)
                {
                    // B's UTurn, place in front
                    return false;
                }
                return odr::crossProduct(va, outgoingRoad.forward) < odr::crossProduct(vb, outgoingRoad.forward); // TODO: verify
            });

            std::vector<TurningGroup> existingIncomingGroups;
            for (const auto& incomingRoad : incomingEndpoints)
            {
                auto inOutKey = std::make_pair(incomingRoad.roadID, outgoingRoad.roadID);
                if (turningGroups.find(inOutKey) != turningGroups.end())
                {
                    existingIncomingGroups.push_back(turningGroups.at(inOutKey));
                }
            }
            assignOutgoingLanes(outgoingRoad.nLanes, existingIncomingGroups);

            auto assignResult = existingIncomingGroups.begin();
            for (const auto& incomingRoad : incomingEndpoints)
            {
                auto inOutKey = std::make_pair(incomingRoad.roadID, outgoingRoad.roadID);
                if (turningGroups.find(inOutKey) != turningGroups.end())
                {
                    turningGroups.at(inOutKey).toLaneIDBase = assignResult++->toLaneIDBase;
                }
            }
        }

        // Compute ref lines
        int connectingIndex = 100; // TODO: indexing
        for (const auto& turningKv : turningGroups)
        {
            std::string incomingRoad = turningKv.first.first;
            std::string outgoingRoad = turningKv.first.second;
            TurningGroup turningGroup = turningKv.second;

            odr::Vec2D incomingRight{ turningGroup.fromForward[1], -turningGroup.fromForward[0]};
            double incomingCenterS = 
                RoadRunner::LaneWidth * turningGroup.fromLaneIDBase +
                RoadRunner::LaneWidth * turningGroup.nLanes / 2;
            odr::Vec2D incomingCenter = odr::add(
                turningGroup.fromOrigin,
                odr::mut(incomingCenterS, incomingRight));

            // Assume outgoing always start from leftmost lane for now
            odr::Vec2D outgoingRight{ turningGroup.toForward[1], -turningGroup.toForward[0] };
            double outgoingCenetrS = 
                RoadRunner::LaneWidth * turningGroup.toLaneIDBase +
                RoadRunner::LaneWidth * turningGroup.nLanes / 2;
            odr::Vec2D outgoingCenter = odr::add(
                turningGroup.toOrigin,
                odr::mut(outgoingCenetrS, outgoingRight));

            auto connectLine = ConnectLines(
                incomingCenter, turningGroup.fromForward,
                outgoingCenter, turningGroup.toForward);

            if (connectLine.has_value())
            {
                RoadRunner::Road connecting(std::to_string(connectingIndex++) + "_" + 
                    std::to_string(turningGroup.direction));
                connecting.SetLength(connectLine.value().length * 100);
                connecting.AddRightSection({ RoadRunner::RoadProfile{turningGroup.nLanes, turningGroup.nLanes}, 0 });
                odr::Road gen = (odr::Road)connecting;

                gen.ref_line.s0_to_geometry[0] = connectLine.value().clone();
                connectings.push_back(gen);
            }
            else
            {
                spdlog::warn("Connecting lane has invalid shape");
            }
            
        } // For each turning direction

        return odr::Junction("1", "junction 1"); // TODO: linkage
    }

    std::vector<double> assignIncomingLanes(int8_t nLanes, std::vector<int8_t> outgoingTotals)
    {
        // TODO
        assert(false);
        return {};
    }

    void assignOutgoingLanes(int8_t nLanes, std::vector<TurningGroup>& incomingLanes)
    {
        for (TurningGroup& group : incomingLanes)
        {
            switch (group.direction)
            {
            case TurningSemantics::U:
            case TurningSemantics::Left:
                group.toLaneIDBase = 0;
                break;
            case TurningSemantics::No:
                // TODO: make those as straight as possible
                group.toLaneIDBase = 0;
                break;
            case TurningSemantics::Right:
                group.toLaneIDBase = nLanes - group.nLanes;
                break;
            }
        }
    }
}