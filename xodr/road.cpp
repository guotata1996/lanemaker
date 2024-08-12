#include "road.h"
#include "junction.h"

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

    Road::Road(const LaneProfile& p, const ElevationProfile& e, odr::RefLine& l) :
        generated(IDGenerator::ForRoad()->GenerateID(this), 0, "-1")
    {
        generated.rr_profile = p;
        generated.rr_eprofile = e;
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
        generated.rr_eprofile.Apply(Length(), &generated);

        PlaceOdrRoadMarkings();
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
        generated.rr_eprofile = generated.rr_eprofile.Reversed(length);
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
            graphics->sBegin = roadLength - graphics->sBegin;
            graphics->sEnd = roadLength - graphics->sEnd;
            graphics->UpdateRefLineHint();
            s_to_section_graphics.emplace(sRev, std::move(graphics));
        }
#endif
    }

    Road::~Road()
    {
        if (successorJunction != nullptr)
        {
            successorJunction->NotifyPotentialChange();
            successorJunction.reset();
        }
        if (predecessorJunction != nullptr)
        {
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

#ifndef G_TEST
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

            if (endS - startS < 0.1f)
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
            double existingEnd = existingStart + existingItr->second->Length;
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

    std::unique_ptr<Road::RoadsOverlap> Road::FirstOverlap(double sBegin, double sEnd) const
    {
        if (sBegin >= sEnd) 
            return nullptr;
        auto beginIt = s_to_section_graphics.upper_bound(sBegin - 1e-3f);
        if (beginIt != s_to_section_graphics.begin())
        {
            beginIt--;
        }
        auto endIt = s_to_section_graphics.upper_bound(sEnd + 1e-3f);

        std::map <std::shared_ptr<Road>, MultiSegment> collidings;
        std::set< LaneGraphics*> myCollidingPieces;
        for (auto it = beginIt; it != endIt; ++it)
        {
            for (auto child : it->second->childItems())
            {
                LaneGraphics* laneSegmentItem = dynamic_cast<LaneGraphics*>(child);
                if (laneSegmentItem == nullptr)
                {
                    continue;
                }
                SectionGraphics* myGraphicsSegment = dynamic_cast<SectionGraphics*>(laneSegmentItem->parentItem());

                for (auto collision : laneSegmentItem->collidingItems())
                {
                    LaneGraphics* collisionSegmentItem = dynamic_cast<LaneGraphics*>(collision);
                    if (collisionSegmentItem == nullptr)
                    {
                        continue;
                    }
                    
                    SectionGraphics* collidingGraphicsSegment = dynamic_cast<SectionGraphics*>(collisionSegmentItem->parentItem());
                    auto collidingRoad = collidingGraphicsSegment->road.lock();
                    if (collidingRoad == shared_from_this())
                    {
                        // self-intersection is ignored for now
                        continue;
                    }

                    if (collidingRoad->generated.junction != "-1")
                    {
                        auto junctionPtr = IDGenerator::ForJunction()->GetByID(collidingRoad->generated.junction);
                        auto junction = static_cast<RoadRunner::Junction*>(junctionPtr)->shared_from_this();
                        if (predecessorJunction == junction && 
                            (myGraphicsSegment->sBegin < GraphicsDivision ||
                            myGraphicsSegment->sEnd < GraphicsDivision))
                        {
                            // Already joined this junction
                            continue;
                        }
                        if (successorJunction == junction &&
                            (myGraphicsSegment->sBegin > Length() - GraphicsDivision ||
                                myGraphicsSegment->sEnd > Length() - GraphicsDivision))
                        {
                            // Already joined this junction
                            continue;
                        }
                    }
                    
                    if (collidings.find(collidingRoad) == collidings.end())
                    {
                        collidings.emplace(collidingRoad, MultiSegment(1));
                    }
                    collidings.at(collidingRoad).Insert(
                        std::min(collidingGraphicsSegment->sBegin, collidingGraphicsSegment->sEnd),
                        std::max(collidingGraphicsSegment->sBegin, collidingGraphicsSegment->sEnd));
                    myCollidingPieces.insert(laneSegmentItem);
                }
            }
        }
        
        std::map<double, std::unique_ptr<RoadsOverlap>> sortedOverlap;
        for (auto colliding : collidings)
        {
            for (auto otherCollidingArea : colliding.second.Merge())
            {
                MultiSegment myCollidingIntervals(1);

                double sBeginOnOther = otherCollidingArea.first;
                double sEndOnOther = otherCollidingArea.second;

                for (auto mine: myCollidingPieces)
                {
                    SectionGraphics* mySegment = dynamic_cast<SectionGraphics*>(mine->parentItem());

                    if (std::min(mySegment->sBegin, mySegment->sEnd) == 0
                        && predecessorJunction != nullptr
                        && dynamic_cast<DirectJunction*>(predecessorJunction.get()) != nullptr)
                    {
                        continue;
                    }
                    if (std::max(mySegment->sBegin, mySegment->sEnd) == Length()
                        && successorJunction != nullptr
                        && dynamic_cast<DirectJunction*>(successorJunction.get()) != nullptr)
                    {
                        continue;
                    }

                    for (auto otherPiece : mine->collidingItems())
                    {
                        LaneGraphics* collisionSegmentItem = dynamic_cast<LaneGraphics*>(otherPiece);
                        if (collisionSegmentItem == nullptr)
                        {
                            continue;
                        }

                        SectionGraphics* collidingGraphicsSegment = dynamic_cast<SectionGraphics*>(collisionSegmentItem->parentItem());
                        if (collidingGraphicsSegment->road.lock() == colliding.first &&
                            SegmentsIntersect(sBeginOnOther, sEndOnOther,
                            collidingGraphicsSegment->sBegin, collidingGraphicsSegment->sEnd))
                        {
                            myCollidingIntervals.Insert(mySegment->sBegin, mySegment->sEnd);
                        }
                    }
                }

                for (const auto& myCollidingInterval : myCollidingIntervals.Merge())
                {
                    auto overlap = std::make_unique<RoadsOverlap>(
                        myCollidingInterval.first, myCollidingInterval.second,
                        colliding.first, sBeginOnOther , sEndOnOther );
                    spdlog::trace("Collision detected between road {} @{}~{} vs road {} @{}~{}",
                        ID(), overlap->sBegin1, overlap->sEnd1,
                        colliding.first->ID(), overlap->sBegin2, overlap->sEnd2);

                    // If multiple candidates overlap at the same s, pick the one with minimum ID
                    if (sortedOverlap.find(overlap->sBegin1) != sortedOverlap.end()
                        && std::stoi(sortedOverlap.at(overlap->sBegin1)->road2.lock()->ID()) > 
                        std::stoi(overlap->road2.lock()->ID()))
                    {
                        sortedOverlap.erase(overlap->sBegin1);
                    }
                    sortedOverlap.emplace(overlap->sBegin1, std::move(overlap));
                }
            }
        }

        if (sortedOverlap.empty())
        {
            return nullptr;
        }
        return std::move(sortedOverlap.begin()->second);
    }

    void Road::EnableHighlight(bool enabled)
    {
        if (highlighted == enabled) return;

        for (auto& s_and_graphics : s_to_section_graphics)
        {
            s_and_graphics.second->EnableHighlight(enabled);
        }
        highlighted = enabled;
    }
#endif

    void Road::PlaceOdrRoadMarkings()
    {
        auto roadID = ID();
        const double MarkingWidth = 0.2f;
        for (auto& sAndSection : generated.s_to_lanesection)
        {
            double sectionS0 = sAndSection.first;
            auto& section = sAndSection.second;
            auto rMostLaneID = section.id_to_lane.begin()->first;
            auto lMostLaneID = section.id_to_lane.rbegin()->first;

            for (int side : {-1, 1})
            {
                int innerUID = side == -1 || rMostLaneID == 0 ? 1 : 2;
                int outerUID = side == -1 ? -rMostLaneID : lMostLaneID;

                for (int uId = innerUID; uId <= outerUID; ++uId)
                {
                    int laneID = uId * side;
                    odr::Lane& lane = section.id_to_lane.at(laneID);
                    if (uId == innerUID)
                    {
                        odr::RoadMarkGroup centerMarking(roadID, sectionS0, laneID,
                            0, 0, 0, "solid", "", "yellow", "standard", "none");
                        centerMarking.roadmark_lines.emplace(odr::RoadMarksLine(
                            roadID, sectionS0, laneID,
                            0, MarkingWidth / 2, 0, 0,
                            side * MarkingWidth / 2,      // t_offset from inner border
                            0,
                            "", ""));
                        lane.roadmark_groups.emplace(std::move(centerMarking));
                    }

                    odr::RoadMarkGroup rightMarking(roadID, sectionS0, laneID,
                        0, 0, 0, uId == outerUID ? "solid" : "broken",
                        "", "white", "standard", "none");
                    rightMarking.roadmark_lines.emplace(odr::RoadMarksLine(
                        roadID, sectionS0, laneID,
                        0, MarkingWidth, 0, 0,
                        side * (RoadRunner::LaneWidth - MarkingWidth),      // t_offset from inner border
                        0,
                        "", ""));
                    lane.roadmark_groups.emplace(std::move(rightMarking));
                }
            }
        }
    }

} // namespace