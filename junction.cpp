#include "junction.h"
#include "curve_fitting.h"
#include <cmath> // floor, ceil

namespace RoadRunner
{
    odr::Junction GenerateConnections(std::vector<ConnectionInfo> connected,
        std::vector<odr::Road>& connectings)
    {
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

        int connectingIndex = 100; // TODO: indexing
        for (const RoadEndpoint& incomingRoad : incomingEndpoints)
        {
            auto dirSplit = incomingRoad.dirSplit;
            assert(dirSplit.size() == outgoingEndpoints.size() - 1);

            dirSplit.insert(dirSplit.begin(), 0);
            dirSplit.push_back(incomingRoad.nLanes);
            int outgoingIndex = 0;
            for (auto dirBegin = dirSplit.begin(), dirEnd = ++dirSplit.begin();
                dirEnd != dirSplit.end(); ++dirBegin, ++dirEnd, ++outgoingIndex)
            {
                double beginLane = *dirBegin;
                double endLane = *dirEnd;
                assert(beginLane <= endLane);
                if (beginLane == endLane)
                {
                    continue;
                }
                int8_t beginIncomingLane = std::floor(beginLane);
                int8_t endIncomingLane = std::ceil(endLane);
                int8_t nLane = endIncomingLane - beginIncomingLane;


                odr::Vec2D incomingRight{ incomingRoad.forward[1], -incomingRoad.forward[0] };
                double incomingCenterS = 
                    RoadRunner::LaneWidth * beginIncomingLane + 
                    RoadRunner::LaneWidth * nLane / 2;
                odr::Vec2D incomingCenter = odr::add(
                    incomingRoad.origin,
                    odr::mut(incomingCenterS, incomingRight));

                std::sort(outgoingEndpoints.begin(), outgoingEndpoints.end(),
                    [&incomingRoad, &incomingCenter](const RoadEndpoint& a, const RoadEndpoint& b)
                {
                    odr::Vec2D va = odr::sub(a.origin, incomingCenter);
                    odr::Vec2D vb = odr::sub(b.origin, incomingCenter);
                    va = odr::normalize(va);
                    vb = odr::normalize(vb);
                    return odr::crossProduct(va, incomingRoad.forward) < odr::crossProduct(vb, incomingRoad.forward);
                });

                // Assume outgoing always start from leftmost lane for now
                // TODO: make this depend on outgoingGroup.nLanes
                const RoadEndpoint& outgoingRoad = outgoingEndpoints[outgoingIndex];
                odr::Vec2D outgoingRight{ outgoingRoad.forward[1], -outgoingRoad.forward[0] };
                double outgoingWidth = RoadRunner::LaneWidth * nLane;
                odr::Vec2D outgoingCenter = odr::add(
                    outgoingRoad.origin,
                    odr::mut(outgoingWidth / 2, outgoingRight));
                spdlog::info("Fitting in: {},{} fwd: {},{} | out: {},{} fwd {},{}", 
                    incomingCenter[0], incomingCenter[1], 
                    incomingRoad.forward[0], incomingRoad.forward[1],
                    outgoingCenter[0], outgoingCenter[1],
                    outgoingRoad.forward[0], outgoingRoad.forward[1]);
                auto connectLine = ConnectLines(
                    incomingCenter, incomingRoad.forward,
                    outgoingCenter, outgoingRoad.forward);

                if (connectLine.has_value())
                {
                    RoadRunner::Road connecting(std::to_string(connectingIndex++));
                    connecting.SetLength(connectLine.value().length * 100);
                    connecting.AddRightSection({ RoadRunner::RoadProfile{nLane, nLane}, 0 });
                    odr::Road gen = (odr::Road)connecting;

                    gen.ref_line.s0_to_geometry[0] = connectLine.value().clone();
                    connectings.push_back(gen);
                }
                else
                {
                    spdlog::warn("Connecting lane has invalid shape");
                }
            } // For each turning direction
        } // For each road end

        return odr::Junction("1", "junction 1"); // TODO: linkage
    }
}