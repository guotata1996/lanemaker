#include "junction.h"
#include "curve_fitting.h"

namespace RoadRunner
{
    odr::Junction GenerateConnections(std::vector<ConnectionInfo> connected,
        std::vector<odr::Road>& connectings)
    {
        std::vector<LaneGroup> incomingGroups, outgoingGroups;
        for (auto roadAndS : connected)
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
                    outgoingGroups.push_back(LaneGroup
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
                    incomingGroups.push_back(LaneGroup
                        {
                            road->id,
                            origin,
                            forward,
                            leftExit.laneCount
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
                    incomingGroups.push_back(LaneGroup
                        {
                            road->id,
                            origin,
                            forward,
                            rightExit.laneCount
                        });
                    SPDLOG_INFO("Incoming: {},{} fwd: {},{}", origin[0], origin[1], forward[0], forward[1]);
                }
                RoadProfile leftEntrance = config.LeftEntrance();
                if (leftEntrance.laneCount != 0)
                {
                    double offset = to_odr_unit(leftEntrance.offsetx2);
                    odr::Vec2D origin = road->ref_line.get_xy(meetAt, offset);
                    odr::Vec2D forward = odr::negate(road->ref_line.get_grad_xy(road->length));
                    outgoingGroups.push_back(LaneGroup
                        {
                            road->id,
                            origin,
                            forward,
                            leftEntrance.laneCount
                        });
                    SPDLOG_INFO("Outgoing: {},{} fwd: {},{}", origin[0], origin[1], forward[0], forward[1]);
                }
            }
        }

        int connectingIndex = 100;
        for (const LaneGroup& incomingGroup : incomingGroups)
        {
            for (const LaneGroup& outgoingGroup : outgoingGroups)
            {
                if (incomingGroup.roadID == outgoingGroup.roadID)
                {
                    continue;
                }
                odr::Vec2D incomingRight{ incomingGroup.forward[1], -incomingGroup.forward[0] };
                double incomingWidth = RoadRunner::LaneWidth * incomingGroup.nLanes;
                odr::Vec2D incomingCenter = odr::add(
                    incomingGroup.origin,
                    odr::mut(incomingWidth / 2, incomingRight));
                odr::Vec2D outgoingRight{ outgoingGroup.forward[1], -outgoingGroup.forward[0] };
                double outgoingWidth = RoadRunner::LaneWidth * outgoingGroup.nLanes;
                odr::Vec2D outgoingCenter = odr::add(
                    outgoingGroup.origin,
                    odr::mut(outgoingWidth / 2, outgoingRight));
                SPDLOG_INFO("Fitting in: {},{} fwd: {},{} | to: {},{} fwd {},{}", 
                    incomingCenter[0], incomingCenter[1], 
                    incomingGroup.forward[0], incomingGroup.forward[1],
                    outgoingCenter[0], outgoingCenter[1],
                    outgoingGroup.forward[0], outgoingGroup.forward[1]);
                auto connectLine = ConnectLines(
                    incomingCenter, incomingGroup.forward,
                    outgoingCenter, outgoingGroup.forward);

                if (connectLine.has_value())
                {
                    RoadRunner::Road connecting(std::to_string(connectingIndex++));
                    connecting.SetLength(connectLine.value().length * 100);
                    connecting.AddRightSection({ RoadRunner::RoadProfile{1, 1}, 0 });
                    odr::Road gen = (odr::Road)connecting;

                    gen.ref_line.s0_to_geometry[0] = connectLine.value().clone();
                    connectings.push_back(gen);
                }
                else
                {
                    SPDLOG_INFO("Connecting lane has invalid shape");
                }
            }
        }

        return odr::Junction("1", "junction 1"); // TODO: linkage
    }
}