#include "road.h"
#include "junction.h"
#include "constants.h"

#include <algorithm>
#include <vector>
#include <map>
#include <sstream>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"

#include "spdlog/spdlog.h"
#ifndef G_TEST
#include "road_graphics.h"
#include "multi_segment.h"
#endif

namespace RoadRunner
{
    bool Road::ClearingMap = false;

    Road::Road(const LaneProfile& p, std::unique_ptr<odr::RoadGeometry> l) :
        generated(IDGenerator::ForRoad()->GenerateID(this), 0, "-1")
    {
        generated.rr_profile = p;
        generated.ref_line.length = l->length;
        generated.ref_line.s0_to_geometry.emplace(0, std::move(l));
        Generate();
    }

    Road::Road(const LaneProfile& p, odr::RefLine& l) :
        generated(IDGenerator::ForRoad()->GenerateID(this), 0, "-1")
    {
        generated.rr_profile = p;
        generated.ref_line = std::move(l);
        Generate();
    }

    Road::Road(const odr::Road& serialized):
        generated(serialized)
    {
        IDGenerator::ForRoad()->TakeID(ID(), this);
    }

    void Road::Generate(bool notifyJunctions)
    {
        generated.length = Length();
        generated.rr_profile.Apply(Length(), &generated);

        generated.PlaceMarkings();
        generated.DeriveLaneBorders();

        IDGenerator::ForRoad()->NotifyChange(ID());

        if (notifyJunctions)
        {
            if (successorJunction != nullptr)
            {
                successorJunction->NotifyPotentialChange();
            }
            if (predecessorJunction != nullptr)
            {
                predecessorJunction->NotifyPotentialChange();
            }
        }
    }

    void Road::ReverseRefLine()
    {
        generated.ref_line.reverse();

        type_s length = from_odr_unit(Length());
        
        generated.rr_profile = generated.rr_profile.Reversed(length);

        Generate(false);

        // Handle linkage
        generated.predecessor.type = odr::RoadLink::Type_None;
        generated.successor.type = odr::RoadLink::Type_None;

        std::swap(successorJunction, predecessorJunction);
        if (successorJunction != nullptr)
        {
            successorJunction->NotifyPotentialChange(ChangeInConnecting{shared_from_this(), ChangeInConnecting::Type_Reverse});
        }
        if (predecessorJunction != nullptr)
        {
            predecessorJunction->NotifyPotentialChange(ChangeInConnecting{ shared_from_this(), ChangeInConnecting::Type_Reverse });
        }

        // Re-index visual; reverse ref line hint
#ifndef G_TEST
        decltype(s_to_section_graphics) temp_graphics(std::move(s_to_section_graphics));
        s_to_section_graphics.clear();
        const double roadLength = Length();
        for (auto it = temp_graphics.begin(); it != temp_graphics.end(); ++it)
        {
            auto segEndIt = it;
            segEndIt++;
            double sRev = (segEndIt == temp_graphics.end()) ? 0 : roadLength - segEndIt->first;
            auto graphics = std::move(it->second);
            graphics->updateIndexingInfo(ID(), -1, roadLength);
            s_to_section_graphics.emplace(sRev, std::move(graphics));
        }
#endif
    }

    Road::~Road()
    {
        if (successorJunction != nullptr)
        {
            if (!ClearingMap)
                successorJunction->NotifyPotentialChange();
            successorJunction.reset();
        }
        if (predecessorJunction != nullptr)
        {
            if (!ClearingMap)
                predecessorJunction->NotifyPotentialChange();
            predecessorJunction.reset();
        }

        if (!ID().empty())
        {
            spdlog::trace("del road {}", ID());
            IDGenerator::ForRoad()->FreeID(ID());
#ifndef G_TEST
            s_to_section_graphics.clear();
#endif
        }
    }

    bool Road::SnapToSegmentBoundary(type_s& key, type_s limit)
    {
        type_s profileLength = from_odr_unit(Length());
        return generated.rr_profile.SnapToSegmentBoundary(key, profileLength, limit);
    }

    double Road::SnapToSegmentBoundary(double key, double limit, bool* outSuccess)
    {
        type_s key_s = from_odr_unit(key);
        type_s limit_s = from_odr_unit(limit);
        type_s modifiedKey_s = key_s;

        bool snapped = SnapToSegmentBoundary(modifiedKey_s, limit_s);
        if (outSuccess != nullptr)
        {
            *outSuccess = snapped;
        }
        if (key_s == modifiedKey_s)
        {
            return key;
        }
        return to_odr_unit(modifiedKey_s);
    }

    void Road::UpdateArrowGraphics(odr::RoadLink::ContactPoint c, std::map<int, uint8_t> laneToArrow, bool stopLine)
    {
#ifndef G_TEST
        MultiSegment dueUpdateS;
        for (auto id_obj : generated.id_to_object)
        {
            if (id_obj.second.type == "roadMark" && 
                (id_obj.second.subtype == "arrow" || id_obj.second.subtype == "stopping-line"))
            {
                dueUpdateS.Insert(id_obj.second.s0 - 0.5, id_obj.second.s0 + 0.5);
            }
        }
#endif

        generated.ToggleStopLine(c, stopLine);
        generated.UpdateArrowMarkings(c, laneToArrow);

#ifndef G_TEST
        for (auto id_obj : generated.id_to_object)
        {
            if (id_obj.second.type == "roadMark" && 
                (id_obj.second.subtype == "arrow" || id_obj.second.subtype == "stopping-line"))
            {
                dueUpdateS.Insert(id_obj.second.s0 - 0.5, id_obj.second.s0 + 0.5);
            }
        }

        // Update magnetic snap area
        dueUpdateS.Insert(0, 0.5);
        dueUpdateS.Insert(Length() - 0.5, Length());

        for (auto segment : dueUpdateS.Merge())
        {
            GenerateOrUpdateSectionGraphicsBetween(segment.first, segment.second);
        }
#endif // !G_TEST
    }

#ifndef G_TEST
    void Road::ApplyBoundaryHideToGraphics()
    {
        if (generated.boundaryHide == graphicsBoundaryHide) 
            return;

        for (auto contact : { odr::RoadLink::ContactPoint_Start, odr::RoadLink::ContactPoint_End })
        {
            for (auto side : { -1, 1 })
            {
                auto boundary = std::make_pair(contact, side);
                double currBoundaryHide = odr::get_map_value_or(graphicsBoundaryHide, boundary, 0.0);
                double newBoundaryHide = odr::get_map_value_or(generated.boundaryHide, boundary, 0.0);
                if (currBoundaryHide != newBoundaryHide)
                {
                    if (contact == odr::RoadLink::ContactPoint_Start)
                    {
                        GenerateOrUpdateSectionGraphicsBetween(0, std::max(currBoundaryHide, newBoundaryHide));
                    }
                    else
                    {
                        GenerateOrUpdateSectionGraphicsBetween(std::min(currBoundaryHide, newBoundaryHide), Length());
                    }
                }
            }
        }

        graphicsBoundaryHide = generated.boundaryHide;
    }

    void Road::GenerateAllSectionGraphics()
    {
        GenerateSectionGraphicsBetween(0, Length());
    }

    void Road::GenerateSectionGraphicsBetween(double s1, double s2)
    {
        const double sBeginGlobal = generated.get_lanesection_s0(s1);
        auto sIt = generated.s_to_lanesection.find(sBeginGlobal);
        for (; sIt != generated.s_to_lanesection.end(); ++sIt)
        {
            auto nextIt = sIt;
            nextIt++;
            double endLocal = nextIt == generated.s_to_lanesection.end() ? generated.length : nextIt->first;
            double startS = std::max(s1, sIt->first);
            double endS = std::min(s2, endLocal);

            if (endS - startS < epsilon)
            {
                continue;
            }

            int nDivision = std::floor((endS - startS) / GraphicsDivision);
            nDivision = std::max(1, nDivision);
            double segmentLength = (endS - startS) / nDivision;
            for (int segmentIndex = 0; segmentIndex != nDivision; ++segmentIndex)
            {
                double segStartS = startS + segmentLength * segmentIndex;
                double segEndS = segmentIndex == nDivision - 1 ? endS : segStartS + segmentLength;
                auto sectionGraphics = std::make_unique<SectionGraphics>(
                    shared_from_this(), sIt->second, segStartS, segEndS);
                s_to_section_graphics.emplace(segStartS, std::move(sectionGraphics));
            }
        }
    }

    void Road::GenerateOrUpdateSectionGraphicsBetween(double s1, double s2)
    {
        std::set<double> dueUpdate;
        double createBegin = s1;
        double createEnd = s2;

        for (auto existingItr = s_to_section_graphics.begin(); 
            existingItr != s_to_section_graphics.end(); ++existingItr)
        {
            double existingStart = existingItr->first;
            double existingEnd = existingStart + existingItr->second->Length();
            if (existingStart < s2 && existingEnd > s1)
            {
                createBegin = std::min(createBegin, existingStart);
                createEnd = std::max(createEnd, existingEnd);
                dueUpdate.insert(existingItr->first);
            }
        }

        for (double toUpdate : dueUpdate)
        {
            s_to_section_graphics.erase(toUpdate);
        }

        GenerateSectionGraphicsBetween(createBegin, createEnd);
    }

    std::vector<Road::RoadsOverlap> Road::AllOverlaps(double sBegin, double sEnd, double zThreshold) const
    {
        std::vector<Road::RoadsOverlap> unfiltered, rtn;
        if (sBegin >= sEnd) 
            return rtn;
        
        std::set<FaceIndex_t> selfFaces;
        for (auto& s_graphics : s_to_section_graphics)
        {
            selfFaces.insert(
                s_graphics.second->allSpatialIndice.begin(),
                s_graphics.second->allSpatialIndice.end());
        }
        
        std::map<std::shared_ptr<Road>, MultiSegment> rangeOnOther;
        std::map<std::shared_ptr<Road>, MultiSegment> rangeOnSelf;
        std::map<std::shared_ptr<Road>, std::vector<std::pair<double, double>>> linkPoints;
        const double RangeExtension = 0.5; // expands from single point to range

        for (auto st: generated.sample_st(sBegin, sEnd, 1.0))
        {
            auto pt = generated.get_xyz(st.first, st.second, 0);

            for (auto overlap : SpatialIndexer::Instance()->AllOverlaps(pt, zThreshold))
            {
                if (overlap.roadID == ID())
                {
                    if (std::abs(overlap.s - st.first) < 2.0)
                    {
                        // Ignore self-overlaps adjacent to query zone
                        continue;
                    }
                    if (overlap.s < st.first)
                    {
                        // Only record pairs where s1 > s2
                        continue;
                    }
                }

                auto collidingRoad = static_cast<Road*>(IDGenerator::ForRoad()->GetByID(overlap.roadID))->shared_from_this();

                if (collidingRoad->generated.junction != "-1")
                {
                    auto junctionPtr = IDGenerator::ForJunction()->GetByID(collidingRoad->generated.junction);
                    auto junction = static_cast<RoadRunner::Junction*>(junctionPtr)->shared_from_this();
                    if (predecessorJunction == junction && st.first == 0)
                    {
                        // Already joined this junction
                        continue;
                    }
                    if (successorJunction == junction && st.first > Length() - 1e-2)
                    {
                        // Already joined this junction
                        continue;
                    }
                }
                   
                if (rangeOnOther.find(collidingRoad) == rangeOnOther.end())
                {
                    rangeOnOther.emplace(collidingRoad, MultiSegment(1));
                }
                rangeOnOther.at(collidingRoad).Insert(
                    std::max(0.0, overlap.s - RangeExtension), 
                    std::min(overlap.s + RangeExtension, collidingRoad->Length()));
                
                if (rangeOnSelf.find(collidingRoad) == rangeOnSelf.end())
                {
                    rangeOnSelf.emplace(collidingRoad, MultiSegment(1));
                }
                rangeOnSelf.at(collidingRoad).Insert(
                    std::max(0.0, st.first - RangeExtension), 
                    std::min(st.first + RangeExtension, Length()));

                linkPoints[collidingRoad].push_back(std::make_pair(st.first, overlap.s));
            }
        }
        
        for (auto other : rangeOnOther)
        {
            auto selfIntervals = rangeOnSelf.at(other.first).Merge();
            auto otherIntervals = other.second.Merge();
            std::set<std::pair<int, int>> associations; // (index of self interval, index of other interval)
            for (auto linkage : linkPoints.at(other.first))
            {
                int myIndex = -1, otherIndex = -1;
                for (int i = 0; i != selfIntervals.size(); ++i)
                {
                    if (selfIntervals[i].first <= linkage.first && linkage.first <= selfIntervals[i].second)
                    {
                        myIndex = i;
                        break;
                    }
                }
                for (int i = 0; i != otherIntervals.size(); ++i)
                {
                    if (otherIntervals[i].first <= linkage.second && linkage.second <= otherIntervals[i].second)
                    {
                        otherIndex = i;
                        break;
                    }
                }
                assert(myIndex != -1);
                assert(otherIndex != -1);
                associations.emplace(std::make_pair(myIndex, otherIndex));
            }

            for (auto association : associations)
            {
                auto selfInterval = selfIntervals[association.first];
                auto otherInterval = otherIntervals[association.second];
                std::set<DirectJunction*> othersDirectJunctions;
                if (/*otherInterval.first == 0 && */other.first->predecessorJunction != nullptr)
                {
                    auto junc = dynamic_cast<DirectJunction*>(other.first->predecessorJunction.get());
                    if (junc != nullptr) othersDirectJunctions.emplace(junc);
                }
                if (/*otherInterval.second == other.first->Length() &&*/
                    other.first->successorJunction != nullptr)
                {
                    auto junc = dynamic_cast<DirectJunction*>(other.first->successorJunction.get());
                    if (junc != nullptr) othersDirectJunctions.emplace(junc);
                }

                if (sBegin == 0 && predecessorJunction != nullptr &&
                    othersDirectJunctions.find(dynamic_cast<DirectJunction*>(predecessorJunction.get())) != othersDirectJunctions.end())
                {
                    // skip overlap across direct junctions
                    continue;
                }

                if (sEnd == Length() && successorJunction != nullptr &&
                    othersDirectJunctions.find(dynamic_cast<DirectJunction*>(successorJunction.get())) != othersDirectJunctions.end())
                {
                    // skip overlap across direct junctions
                    continue;
                }

                rtn.emplace_back(selfInterval.first, selfInterval.second,
                    other.first, otherInterval.first, otherInterval.second);
            }
        }
        // Make sure result is deterministic in case multiple overlap at the same point
        std::sort(rtn.begin(), rtn.end(), [](const Road::RoadsOverlap& a, const Road::RoadsOverlap& b)
        {
           if (a.sBegin1 != b.sBegin1)
           {
               return a.sBegin1 < b.sBegin1;
           }
           if (a.sBegin2 != b.sBegin2)
           {
               return a.sBegin2 < b.sBegin2;
           }
           return std::stoi(a.road2.lock()->ID()) < std::stoi(b.road2.lock()->ID());
        });
        
        return rtn;
    }

    std::optional<Road::RoadsOverlap> Road::FirstOverlap(double sBegin, double sEnd) const
    {
        std::optional<Road::RoadsOverlap> rtn;
        auto allOverlaps = AllOverlaps(sBegin, sEnd, BridgeClearance);
        if (allOverlaps.empty()) return rtn;
        rtn.emplace(allOverlaps.front());
        return rtn;
    }

    void Road::EnableHighlight(bool enabled, bool bringToTop)
    {
        if (highlighted.has_value() && highlighted.value() == enabled) return;

        for (auto& s_and_graphics : s_to_section_graphics)
        {
            s_and_graphics.second->EnableHighlight(enabled);
        }
        highlighted.emplace(enabled);
    }
#endif

} // namespace