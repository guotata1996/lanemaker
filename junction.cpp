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

            std::vector<TurningGroup> allPossibleOutgoings;
            allPossibleOutgoings.reserve(outgoingEndpoints.size());
            
            for (auto outgoingRoad :outgoingEndpoints)
            {
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
                allPossibleOutgoings.emplace_back(
                    TurningGroup{
                        incomingRoad.origin, incomingRoad.forward,
                        outgoingRoad.origin, outgoingRoad.forward,
                        turningSemantics, outgoingRoad.nLanes});
            }

            // Fetch or compute dir split
            std::vector<double> dirSplit = incomingRoad.dirSplit;
            if (dirSplit.size() != outgoingEndpoints.size() - 1)
            {
                dirSplit = assignIncomingLanes(incomingRoad.nLanes, allPossibleOutgoings);
                assert(dirSplit.size() == outgoingEndpoints.size() - 1);
            }

            // Filter out true turns
            int8_t outgoingIndex = 0;
            for (std::pair<int, int>& beginAndCount : splitPointToLaneCount(incomingRoad.nLanes, dirSplit))
            {
                int nLanes = beginAndCount.second;
                if (nLanes != 0)
                {
                    auto outgoingRoad = outgoingEndpoints[outgoingIndex];
                    TurningGroup outgoingGroup = allPossibleOutgoings[outgoingIndex];
                    outgoingGroup.fromLaneIDBase = beginAndCount.first;
                    outgoingGroup.nLanes = nLanes;
                    turningGroups.emplace(
                        std::make_pair(incomingRoad.roadID, outgoingRoad.roadID),
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
            assignOutgoingLanes(existingIncomingGroups);

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
                std::string directionName;
                switch (turningGroup.direction)
                {
                case TurningSemantics::U:
                    directionName = "_u-turn";
                    break;
                case TurningSemantics::Left:
                    directionName = "_left-turn";
                    break;
                case TurningSemantics::Right:
                    directionName = "_right-turn";
                    break;
                default:
                    break;
                }
                RoadRunner::Road connecting(std::to_string(connectingIndex++) + directionName);
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

    std::vector<std::pair<int, int>> splitPointToLaneCount(int8_t nLanes, std::vector<double> splitPoints)
    {
        splitPoints.insert(splitPoints.begin(), 0);
        splitPoints.push_back(nLanes);
        std::vector<std::pair<int, int>> rtn;

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

    std::vector<double> assignIncomingLanes(int8_t nLanes, const std::vector<TurningGroup>& outgoings)
    {
        int totalOutgoingLanes = 0;
        std::for_each(outgoings.begin(), outgoings.end(), [&totalOutgoingLanes](const TurningGroup& group)
            {totalOutgoingLanes += group.toTotalLanes; });
        if (totalOutgoingLanes < nLanes)
        {
            spdlog::error("Not enough outgoing lanes for incomings to distribute!");
            return {};
        }

        constexpr double idealSemanticsWeight[4]{1, 0.01, 0.99, 0.5}; // No, U, Left, Right

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
                std::vector<std::pair<int, int>> proposal = splitPointToLaneCount(nLanes, rtn);
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
                    std::vector<std::pair<int, int>> resulting = splitPointToLaneCount(nLanes, rtn);
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
                        if (lane > 0 && std::ceil(rtn[lane - 1]) - rtn[lane - 1] > 1e-4
                            && idealSemanticsWeight[outgoings[lane - 1].direction] > idealSemanticsWeight[outgoings[lane].direction])
                        {
                            // left boundary can be moved right, so that my section will not share lane with left neighbor
                            rtn[lane - 1] = std::ceil(rtn[lane - 1]);
                        }
                        if (lane < resulting.size() - 1 && rtn[lane] - std::floor(rtn[lane])
                            && idealSemanticsWeight[outgoings[lane + 1].direction] > idealSemanticsWeight[outgoings[lane].direction])
                        {
                            // right boundary can be moved left, so that my section will not share lane with right neighbor
                            rtn[lane] = std::floor(rtn[lane]);
                        }
                    }

                    std::stringstream ss;
                    std::for_each(rtn.begin(), rtn.end(), [&ss](double p) {ss << p << " "; });
                    spdlog::debug("{} lane final assignment = {} at semantics w = {}", nLanes, ss.str(), factor01);
                    return rtn;
                }
                if (dead)
                {
                    break;
                }
            }
        }

        spdlog::error("Can't find a suitable incoming lane distribution. Algo failed!");
        return {};
    }

    void assignOutgoingLanes(std::vector<TurningGroup>& incomingLanes)
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
                group.toLaneIDBase = group.toTotalLanes - group.nLanes;
                break;
            }
        }
    }
}