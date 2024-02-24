#include "junction.h"
#include "curve_fitting.h"
#include <cmath> // floor, ceil
#include <map>
#include <IDGenerator.h>

namespace RoadRunner
{
    int GenerateConnections(std::string junctionID,
        std::vector<ConnectionInfo> connected,
        std::vector<Road>& connectings)
    {
        auto errorCode = 0;
        // Collect endpoint info of each connected road
        std::vector<RoadEndpoint> incomingEndpoints, outgoingEndpoints;
        for (auto& roadAndS : connected)
        {
            odr::Road* road = &roadAndS.road->generated;
            double meetAt = roadAndS.s;
            RoadProfile config = roadAndS.road->profile;
            odr::LaneSection meetSection = meetAt == 0 ? 
                road->s_to_lanesection.begin()->second : road->s_to_lanesection.rbegin()->second;
            
            if (meetAt == 0)
            {
                road->predecessor = odr::RoadLink(junctionID, odr::RoadLink::Type_Junction, odr::RoadLink::ContactPoint_Start);
                SectionProfile rightEntrance = config.RightEntrance();
                if (rightEntrance.laneCount != 0)
                {
                    double offset = to_odr_unit(rightEntrance.offsetx2);
                    odr::Vec2D origin = road->ref_line.get_xy(meetAt, offset);
                    odr::Vec2D forward = road->ref_line.get_grad_xy(meetAt);
                    outgoingEndpoints.push_back(RoadEndpoint
                        {
                            road,
                            odr::RoadLink::ContactPoint_Start,
                            -1,
                            origin,
                            forward,
                            static_cast<uint8_t>(rightEntrance.laneCount)
                        });
                }
                SectionProfile leftExit = config.LeftExit();
                if (leftExit.laneCount != 0)
                {
                    double offset = to_odr_unit(leftExit.offsetx2);
                    odr::Vec2D origin = road->ref_line.get_xy(meetAt, offset);
                    odr::Vec2D forward = odr::negate(road->ref_line.get_grad_xy(meetAt));
                    incomingEndpoints.push_back(RoadEndpoint
                        {
                            road,
                            odr::RoadLink::ContactPoint_Start,
                            1,
                            origin,
                            forward,
                            static_cast<uint8_t>(leftExit.laneCount),
                            roadAndS.dirSplit
                        });
                }
            }
            else
            {
                road->successor = odr::RoadLink(junctionID, odr::RoadLink::Type_Junction, odr::RoadLink::ContactPoint_End);
                SectionProfile rightExit = config.RightExit();
                if (rightExit.laneCount != 0)
                {
                    double offset = to_odr_unit(rightExit.offsetx2);
                    odr::Vec2D origin = road->ref_line.get_xy(meetAt, offset);
                    odr::Vec2D forward = road->ref_line.get_grad_xy(meetAt);
                    incomingEndpoints.push_back(RoadEndpoint
                        {
                            road,
                            odr::RoadLink::ContactPoint_End,
                            -1,
                            origin,
                            forward,
                            static_cast<uint8_t>(rightExit.laneCount),
                            roadAndS.dirSplit
                        });
                }
                SectionProfile leftEntrance = config.LeftEntrance();
                if (leftEntrance.laneCount != 0)
                {
                    double offset = to_odr_unit(leftEntrance.offsetx2);
                    odr::Vec2D origin = road->ref_line.get_xy(meetAt, offset);
                    odr::Vec2D forward = odr::negate(road->ref_line.get_grad_xy(road->length));
                    outgoingEndpoints.push_back(RoadEndpoint
                        {
                            road,
                            odr::RoadLink::ContactPoint_End,
                            1,
                            origin,
                            forward,
                            static_cast<uint8_t>(leftEntrance.laneCount)
                        });
                }
            }
        }

        std::map<std::pair<RoadEndpoint, RoadEndpoint>, TurningGroup> turningGroups;

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

            std::vector<TurningGroup> allPossibleOutgoings;
            allPossibleOutgoings.reserve(outgoingEndpoints.size());
            
            for (auto outgoingRoad :outgoingEndpoints)
            {
                double turnAngle = odr::angle(incomingRoad.forward, outgoingRoad.forward);
                auto turningSemantics = RoadRunner::Turn_No;
                if (incomingRoad.road == outgoingRoad.road && incomingRoad.contact == outgoingRoad.contact)
                {
                    turningSemantics = TurningSemantics::Turn_U;
                }
                else if (turnAngle > M_PI / 4)
                {
                    turningSemantics = TurningSemantics::Turn_Left;
                }
                else if (turnAngle < -M_PI / 4)
                {
                    turningSemantics = TurningSemantics::Turn_Right;
                }
                allPossibleOutgoings.emplace_back(
                    TurningGroup{
                        incomingRoad.origin, incomingRoad.forward,
                        outgoingRoad.origin, outgoingRoad.forward,
                        incomingRoad.contact,outgoingRoad.contact,
                        incomingRoad.side,   outgoingRoad.side,
                        turningSemantics, outgoingRoad.nLanes});                
            }

            // Fetch or compute dir split
            std::vector<double> dirSplit = incomingRoad.dirSplit;
            if (dirSplit.size() != outgoingEndpoints.size() - 1)
            {
                dirSplit = assignIncomingLanes(incomingRoad.nLanes, allPossibleOutgoings, errorCode);

                if (dirSplit.size() != outgoingEndpoints.size() - 1)
                {
                    // Generate dir split failed. Skip this incoming road.
                    continue;
                }
            }

            // Filter out vanishing out directions
            int8_t outgoingIndex = 0;
            for (std::pair<uint8_t, uint8_t>& beginAndCount : splitPointToLaneCount(incomingRoad.nLanes, dirSplit))
            {
                int nLanes = beginAndCount.second;
                if (nLanes != 0)
                {
                    auto outgoingRoad = outgoingEndpoints[outgoingIndex];
                    TurningGroup outgoingGroup = allPossibleOutgoings[outgoingIndex];
                    outgoingGroup.fromLaneIDBase = beginAndCount.first;
                    outgoingGroup.nLanes = nLanes;
                    turningGroups.emplace(
                        std::make_pair(incomingRoad, outgoingRoad),
                        outgoingGroup);
                }
                outgoingIndex++;
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
                return odr::crossProduct(va, outgoingRoad.forward) < odr::crossProduct(vb, outgoingRoad.forward);
            });

            std::vector<TurningGroup> existingIncomingGroups;
            for (const auto& incomingRoad : incomingEndpoints)
            {
                auto inOutKey = std::make_pair(incomingRoad, outgoingRoad);
                if (turningGroups.find(inOutKey) != turningGroups.end())
                {
                    existingIncomingGroups.push_back(turningGroups.at(inOutKey));
                }
            }
            assignOutgoingLanes(existingIncomingGroups);

            auto assignResult = existingIncomingGroups.begin();
            for (const auto& incomingRoad : incomingEndpoints)
            {
                auto inOutKey = std::make_pair(incomingRoad, outgoingRoad);
                if (turningGroups.find(inOutKey) != turningGroups.end())
                {
                    turningGroups.at(inOutKey).toLaneIDBase = assignResult++->toLaneIDBase;
                }
            }
        }

        // Compute ref lines
        for (const auto& turningKv : turningGroups)
        {
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

            auto connectingRefLine = ConnectLines(
                incomingCenter, turningGroup.fromForward,
                outgoingCenter, turningGroup.toForward);

            if (connectingRefLine == nullptr)
            {
                errorCode |= Junction_ConnectionInvalidShape;
                continue;
            }

            RoadRunner::RoadProfile connectingProfile(0, 0, turningGroup.nLanes, turningGroup.nLanes);

            auto connecting = Road(connectingProfile, connectingRefLine);
            connecting.Generate();

            // Assign linkage
            odr::Road& connRoad = connecting.generated;
            odr::Road* incomingRoad = turningKv.first.first.road;
            odr::Road* outgoingRoad = turningKv.first.second.road;
            connRoad.junction = junctionID;
            connRoad.predecessor = odr::RoadLink(incomingRoad->id, odr::RoadLink::Type_Road, odr::RoadLink::ContactPoint_Start);
            connRoad.successor = odr::RoadLink(outgoingRoad->id, odr::RoadLink::Type_Road, odr::RoadLink::ContactPoint_End);
            odr::LaneSection& onlySection = connRoad.s_to_lanesection.begin()->second;

            auto& connectingLanes = onlySection.get_sorted_driving_lanes(-1);
            auto& incomingSection = turningGroup.fromContact == odr::RoadLink::ContactPoint_Start ?
                incomingRoad->s_to_lanesection.begin()->second : incomingRoad->s_to_lanesection.rbegin()->second;
            auto& incomingLanes = incomingSection.get_sorted_driving_lanes(turningGroup.fromSide);
            auto& outgoingSection = turningGroup.toContact == odr::RoadLink::ContactPoint_Start ?
                outgoingRoad->s_to_lanesection.begin()->second : outgoingRoad->s_to_lanesection.rbegin()->second;
            auto& outgoingLanes = outgoingSection.get_sorted_driving_lanes(turningGroup.toSide);

            for (int i = 0; i != connectingLanes.size(); ++i)
            {
                // from center to side
                int connectingID = connectingLanes[i].id;
                int incomingID = incomingLanes[std::abs(turningGroup.fromLaneIDBase) + i].id;
                onlySection.id_to_lane.at(connectingID).predecessor = incomingID;
                int outgoingID = outgoingLanes[std::abs(turningGroup.toLaneIDBase) + i].id;
                onlySection.id_to_lane.at(connectingID).successor = outgoingID;
            }

            connectings.push_back(std::move(connecting));
        } // For each turning direction
        return errorCode;
    }

    std::vector<std::pair<uint8_t, uint8_t>> splitPointToLaneCount(int8_t nLanes, std::vector<double> splitPoints)
    {
        splitPoints.insert(splitPoints.begin(), 0);
        splitPoints.push_back(nLanes);
        std::vector<std::pair<uint8_t, uint8_t>> rtn;

        for (auto dirBegin = splitPoints.begin(), dirEnd = ++splitPoints.begin();
            dirEnd != splitPoints.end(); ++dirBegin, ++dirEnd)
        {
            double beginLane = *dirBegin;
            double endLane = *dirEnd;
            int beginIndex, endIndex;
            if (std::abs(beginLane - std::round(beginLane)) < 1e-4)
            {
                beginIndex = std::round(beginLane);
            }
            else
            {
                beginIndex = std::floor(beginLane);
            }

            if (std::abs(endLane - std::round(endLane)) < 1e-4)
            {
                endIndex = std::round(endLane);
            }
            else
            {
                endIndex = std::ceil(endLane);
            }
            int laneCount = std::abs(beginLane - endLane) < 1e-4 ? 0 : endIndex - beginIndex;
            rtn.push_back(std::make_pair(beginIndex, laneCount));
        }
        return rtn;
    }

    std::vector<double> assignIncomingLanes(int8_t nLanes, const std::vector<TurningGroup>& outgoings, int& errorCode)
    {
        int totalOutgoingLanes = 0;
        std::for_each(outgoings.begin(), outgoings.end(), [&totalOutgoingLanes](const TurningGroup& group)
        {totalOutgoingLanes += group.toTotalLanes; });
        if (totalOutgoingLanes < nLanes)
        {
            spdlog::error("Not enough outgoing lanes for incomings to distribute!");
            errorCode |= Junction_TooManyIncomingLanes;
            return {};
        }

        constexpr double idealSemanticsWeight[4]{ 1, 0.01, 0.99, 0.5 }; // No, U, Left, Right

        std::vector<double> rtn;
        for (int semanticsFactor = 5; semanticsFactor >= 0; --semanticsFactor)
        {
            double factor01 = (double)semanticsFactor / 5;
            double semanticsWeight[4]{
                idealSemanticsWeight[0] * factor01 + 1.0 * (1 - factor01),
                idealSemanticsWeight[1] * factor01 + 1.0 * (1 - factor01),
                idealSemanticsWeight[2] * factor01 + 1.0 * (1 - factor01),
                idealSemanticsWeight[3] * factor01 + 1.0 * (1 - factor01),
            };
            // Raw distribution by semantic_weight * inflated_nLanes
            std::vector<double> weights(outgoings.size());
            std::transform(outgoings.begin(), outgoings.end(), weights.begin(),
                [&semanticsWeight](const TurningGroup& p) {
                return p.toTotalLanes * semanticsWeight[(int)p.direction]; });
            // Turn weight into normalized cumulative
            double init = 0;
            double sumWeight = std::accumulate(weights.begin(), weights.end(), init);
            assert(sumWeight > 1e-4);
            std::transform(weights.begin(), weights.end(), weights.begin(),
                [nLanes, sumWeight](double w) {return w / sumWeight * nLanes; });

            rtn.clear();
            for (double w : weights)
            {
                double cum = rtn.empty() ? w : rtn.back() + w;
                rtn.push_back(cum);
            }
            rtn.pop_back();

            std::stringstream sss;
            std::for_each(rtn.begin(), rtn.end(), [&sss](double p) {sss << p << " "; });
            spdlog::trace("Initial: {} lane assignment = {}", nLanes, sss.str());

            // Adjust result to obey out lane limit
            for (int adjustment = 0; adjustment != 20; ++adjustment)
            {
                std::vector<std::pair<uint8_t, uint8_t>> proposal = splitPointToLaneCount(nLanes, rtn);
                assert(proposal.size() == outgoings.size());

                bool pass = true;
                bool dead = false;
                for (int i = 0; i != proposal.size(); ++i)
                {
                    int proposedLanes = proposal[i].second;
                    int limit = outgoings[i].toTotalLanes;
                    if (proposedLanes > limit)
                    {
                        pass = false;
                        double moveLeftCost = 10, moveRightCost = 10;
                        if (i != 0)
                        {
                            // left boundary can be moved right
                            moveLeftCost = std::ceil(rtn[i - 1]) - rtn[i - 1];
                        }
                        if (i != proposal.size() - 1)
                        {
                            // right boundary can be moved left
                            moveRightCost = rtn[i] - std::floor(rtn[i]);
                        }

                        if (1e-4 <= moveLeftCost && moveLeftCost <= moveRightCost)
                        {
                            // moving left boundary is easier
                            rtn[i - 1] = std::ceil(rtn[i - 1]);
                        }
                        else if (1e-4 <= moveRightCost && moveRightCost <= moveLeftCost)
                        {
                            // moving right boundary is easier
                            rtn[i] = std::floor(rtn[i]);
                        }
                        else
                        {
                            // No further adjustment can be made
                            dead = true;
                        }
                    }
                }
                if (pass)
                {
                    // Prevent multi-direction lanes unless necessary
                    std::vector<std::pair<uint8_t, uint8_t>> resulting = splitPointToLaneCount(nLanes, rtn);
                    for (int lane = 0; lane != resulting.size(); ++lane)
                    {
                        int number = resulting[lane].second;
                        if (number == 1)
                        {
                            // section cannot shrink
                            continue;
                        }

                        // If there's an overlap with a higher-importance neighbor
                        // Try shrinking self At the cost of -1 lane to eliminate overlap
                        if (lane > 0)
                        {
                            double ceil = std::ceil(rtn[lane - 1]);
                            if (std::abs(rtn[lane - 1] - std::round(rtn[lane - 1])) > 1e-4 && std::abs(rtn[lane - 1] > ceil) > 1e-4 &&
                                idealSemanticsWeight[outgoings[lane - 1].direction] > idealSemanticsWeight[outgoings[lane].direction])
                            {
                                // left boundary can be moved right, so that my section will not share lane with left neighbor
                                rtn[lane - 1] = ceil;
                            }
                        }

                        if (lane < resulting.size() - 1)
                        {
                            double floor = std::floor(rtn[lane]);
                            if (std::abs(rtn[lane] - std::round(rtn[lane])) > 1e-4 && rtn[lane] - floor > 1e-4
                                && idealSemanticsWeight[outgoings[lane + 1].direction] > idealSemanticsWeight[outgoings[lane].direction])
                            {
                                // right boundary can be moved left, so that my section will not share lane with right neighbor
                                rtn[lane] = floor;
                            }
                        }
                    }

                    std::stringstream ss;
                    std::for_each(rtn.begin(), rtn.end(), [&ss](double p) {ss << p << " "; });
                    return rtn;
                }
                if (dead)
                {
                    break;
                }
            }
        }

        spdlog::error("Can't find a suitable incoming lane distribution. Algo failed!");
        errorCode |= Junction_AlgoFail;
        return {};
    }

    void assignOutgoingLanes(std::vector<TurningGroup>& incomingLanes)
    {
        for (TurningGroup& group : incomingLanes)
        {
            switch (group.direction)
            {
            case TurningSemantics::Turn_U:
            case TurningSemantics::Turn_Left:
                group.toLaneIDBase = 0;
                break;
            case TurningSemantics::Turn_No:
                // TODO: make those as straight as possible
                group.toLaneIDBase = 0;
                break;
            case TurningSemantics::Turn_Right:
                group.toLaneIDBase = group.toTotalLanes - group.nLanes;
                break;
            }
        }
    }

    Junction::Junction(const std::vector<ConnectionInfo>& connected, std::string id):
        generated("", id.empty() ? IDGenerator::ForJunction()->GenerateID(this) : id)
    {
        generated.name = "Junction " + generated.id;

        generationError = GenerateConnections(generated.id, connected, connectingRoads);

        int junctionConnID = 0;
        for (auto& connecting : connectingRoads)
        {
            auto incomingRoad = connecting.generated.predecessor.id;
            auto outgoingRoad = connecting.generated.successor.id;

            odr::JunctionConnection prevConn(std::to_string(junctionConnID++), 
                incomingRoad, connecting.ID(),
                odr::JunctionConnection::ContactPoint_Start,
                outgoingRoad);

            for (odr::Lane connectinglane : connecting.generated.s_to_lanesection.begin()->second.get_sorted_driving_lanes(-1))
            {
                prevConn.lane_links.insert(odr::JunctionLaneLink(connectinglane.predecessor, connectinglane.id, connectinglane.successor));
            }
            generated.id_to_connection.emplace(prevConn.id, prevConn);
        }
    }

    void clearLinkage(std::string junctionID, std::string regularRoad)
    {
        auto road = IDGenerator::ForRoad()->GetByID(regularRoad);
        if (road == nullptr)
        {
            return;
        }
        odr::Road& affectedRoad = static_cast<Road*>(road)->generated;
        if (affectedRoad.successor.type == odr::RoadLink::Type_Junction && 
            affectedRoad.successor.id == junctionID)
        {
            affectedRoad.successor.type = odr::RoadLink::Type_None;
            affectedRoad.successor.id = "";
            auto lastSection = affectedRoad.s_to_lanesection.rbegin()->second;
            // right side loses next
            for (auto& lane : lastSection.get_sorted_driving_lanes(-1))
            {
                lane.successor = 0;
                lastSection.id_to_lane.find(lane.id)->second = lane;
            }
            // left side loses prev
            for (auto& lane : lastSection.get_sorted_driving_lanes(1))
            {
                lane.predecessor = 0;
                lastSection.id_to_lane.find(lane.id)->second = lane;
            }
        }
        if (affectedRoad.predecessor.type == odr::RoadLink::Type_Junction &&
            affectedRoad.predecessor.id == junctionID)
        {
            affectedRoad.predecessor.type = odr::RoadLink::Type_None;
            affectedRoad.predecessor.id = "";
            auto firstSection = affectedRoad.s_to_lanesection.begin()->second;
            // right side loses prev
            for (auto& lane : firstSection.get_sorted_driving_lanes(-1))
            {
                lane.predecessor = 0;
                firstSection.id_to_lane.find(lane.id)->second = lane;
            }
            // left side loses next
            for (auto& lane : firstSection.get_sorted_driving_lanes(1))
            {
                lane.successor = 0;
                firstSection.id_to_lane.find(lane.id)->second = lane;
            }
        }
    }

    Junction::~Junction()
    {
        if (!ID().empty())
        {
            spdlog::trace("del junction {} w/ {} connections", ID(), connectingRoads.size());
            for (auto& connectingRoad : connectingRoads)
            {
                clearLinkage(ID(), connectingRoad.generated.successor.id);
                clearLinkage(ID(), connectingRoad.generated.predecessor.id);
            }
            connectingRoads.clear();
            IDGenerator::ForJunction()->FreeID(ID());
        }
    }

} // namespace RoadRunner