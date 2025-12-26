#include "road_drawing.h"
#include "curve_fitting.h"
#include "LaneConfigWidget.h"
#include "map_view_gl.h"
#include "junction.h"
#include "constants.h"
#include "road_overlaps.h"


RoadCreationSession::DirectionHandle::DirectionHandle(const odr::Vec3D& aCenter, double aAngle) :
	center(aCenter), angle(aAngle)
{
	UpdateGraphics();
}

bool RoadCreationSession::DirectionHandle::Update(const LM::MouseAction& act)
{
	odr::Vec2D hitPos;
	auto hit = rayHitLocal(hitPos);
	if (act.type == QEvent::Type::MouseButtonPress && hit)
	{
		dragging = true;
		deltaRotation = angle - std::atan2(hitPos[1], hitPos[0]);
		return true;
	}
	if (act.type == QEvent::Type::MouseButtonRelease && dragging)
	{
		dragging = false;
		UpdateGraphics();
		return true;
	}
	if (dragging)
	{
		angle = std::atan2(hitPos[1], hitPos[0]) + deltaRotation;
		UpdateGraphics();
		return true;
	}
	return hit;
}

double RoadCreationSession::DirectionHandle::Rotation() const
{
	return angle;
}

bool RoadCreationSession::DirectionHandle::rayHitLocal(odr::Vec2D& outPos) const
{
	outPos = odr::sub(CursorAtHeight(center[2]), odr::Vec2D{ center[0], center[1] });
	auto offsetMagnitude = odr::norm(outPos);
	return InnerRadius < offsetMagnitude && offsetMagnitude < OuterRadius;
}

void RoadCreationSession::DirectionHandle::UpdateGraphics()
{
	const int nDivision = 36;
	const double sliceAngle = 2 * M_PI / nDivision;
	auto angles = odr::xrange(angle + sliceAngle / 2, angle + 2 * M_PI - sliceAngle / 2, sliceAngle);

	auto liftedCenter = odr::add(center, odr::Vec3D{ 0, 0, 0.01 });
	odr::Line3D roundBoundary;
	// Outer circle
	for (auto angle_outer : angles)
	{
		odr::Vec3D r{ std::cos(angle_outer), std::sin(angle_outer), 0 };
		roundBoundary.push_back(odr::add(liftedCenter, odr::mut(OuterRadius, r)));
	}
	// Inner circle, reversed
	for (auto angle_inner = angles.rbegin(); angle_inner != angles.rend(); ++angle_inner)
	{
		odr::Vec3D r{ std::cos(*angle_inner), std::sin(*angle_inner), 0 };
		roundBoundary.push_back(odr::add(liftedCenter, odr::mut(InnerRadius, r)));
	}
	if (graphicsItem.has_value())
	{
		graphicsItem.reset();
	}
	graphicsItem.emplace(roundBoundary, dragging ? Qt::green : Qt::darkGreen);
}

RoadDrawingSession::SnapResult RoadCreationSession::SnapCursor(odr::Vec2D& point)
{
	double zLevel = 0;
	if (auto pointerRoad = GetPointerRoad(); pointerRoad != nullptr)
	{
		zLevel = pointerRoad->generated.ref_line.elevation_profile.get(GetAdjustedS());
	}
	point = CursorAtHeight(zLevel);

	// Snap to existing road
	if (!startPos.has_value())
	{
		extendFromStart.reset();
		auto snapFirstResult = SnapFirstPointToExisting(point);
		if (snapFirstResult != Snap_Nothing)
		{
			return snapFirstResult;
		}
	}
	else
	{
		joinAtEnd.reset();
		auto snapLastResult = SnapLastPointToExisting(point);
		if (snapLastResult != Snap_Nothing)
		{
			return snapLastResult;
		}
	}

	// Snap to end dir extension line
	if (!stagedGeometries.empty() || !extendFromStart.expired())
	{
		odr::Vec2D localStartPos, localStartDir;
		if (!stagedGeometries.empty())
		{
			const auto& geo = stagedGeometries.back().geo;
			localStartPos = geo->get_xy(geo->length);
			localStartDir = odr::normalize(geo->get_grad(geo->length));
		}
		else
		{
			localStartPos = startPos.value();
			localStartDir = ExtendFromDir();
		}
		
		auto start2Point = odr::sub(point, localStartPos);
		auto biasAngle = odr::angle(start2Point, localStartDir);

		auto projLength = odr::dot(start2Point, localStartDir);
		projLength = std::max(0.0, projLength);
		auto projected = odr::add(localStartPos, odr::mut(projLength, localStartDir));
		auto biasLength = odr::euclDistance(point, projected);
		if (std::abs(biasAngle) < 0.06 && biasLength < SnapDistFromScale())
		{
			point = projected;
		}
	}
	return Snap_Nothing;
}

odr::Vec2D RoadCreationSession::ExtendFromDir() const
{
	auto grad = odr::normalize(extendFromStart.lock()->RefLine().get_grad_xy(extendFromStartS));
	return extendFromStartS == 0 ? odr::negate(grad) : grad;
}

odr::Vec2D RoadCreationSession::JoinAtEndDir() const
{
	auto grad = odr::normalize(joinAtEnd.lock()->RefLine().get_grad_xy(joinAtEndS));
	return joinAtEndS == 0 ? grad : odr::negate(grad);
}

double RoadCreationSession::CursorElevation() const
{
	if (startPos.has_value() && !joinAtEnd.expired())
	{
		return joinAtEnd.lock()->generated.get_xyz(joinAtEndS, 0, 0)[2];
	}
	if (!startPos.has_value() && !extendFromStart.expired())
	{
		return extendFromStart.lock()->generated.get_xyz(extendFromStartS, 0, 0)[2];
	}
	return LM::ElevationStep * LM::g_createRoadElevationOption;
}

RoadDrawingSession::SnapResult RoadCreationSession::SnapFirstPointToExisting(odr::Vec2D& point)
{
	auto pointerRoad = GetPointerRoad();
	if (pointerRoad == nullptr || pointerRoad->IsConnectingRoad()) return Snap_Nothing;

	const double snapThreshold = SnapDistFromScale();
	double snapS = LM::g_PointerRoadS;
	bool onExisting = false;
	if (LM::g_PointerRoadS < snapThreshold &&
		pointerRoad->predecessorJunction == nullptr)
	{
		snapS = 0;
		extendFromStart = pointerRoad;
		extendFromStartS = 0;
	}
	else if (LM::g_PointerRoadS > pointerRoad->Length() - snapThreshold &&
		pointerRoad->successorJunction == nullptr)
	{
		snapS = pointerRoad->Length();
		extendFromStart = pointerRoad;
		extendFromStartS = pointerRoad->Length();
	}

	if (!extendFromStart.expired())
	{
		// only snap to ends
		point = pointerRoad->generated.get_xy(snapS);
		onExisting = true;
	}
	return onExisting ? Snap_Point : Snap_Nothing;
}

RoadDrawingSession::SnapResult RoadCreationSession::SnapLastPointToExisting(odr::Vec2D& point)
{
	auto pointerRoad = GetPointerRoad();
	if (pointerRoad == nullptr || pointerRoad->IsConnectingRoad()) return Snap_Nothing;

	bool onExisting = false;

	// Join to existing
	double snapS;
	const double snapThreshold = SnapDistFromScale();
	if (LM::g_PointerRoadS < snapThreshold &&
		pointerRoad->predecessorJunction.get() == nullptr)
	{
		snapS = 0;
		joinAtEnd = pointerRoad;
	}
	else if (LM::g_PointerRoadS > pointerRoad->Length() - snapThreshold &&
		pointerRoad->successorJunction.get() == nullptr)
	{
		snapS = pointerRoad->Length();
		joinAtEnd = pointerRoad;
	}
	if (!joinAtEnd.expired())
	{
		point = pointerRoad->generated.get_xy(snapS);
		onExisting = true;
		joinAtEndS = snapS;
	}
	cursorItem->EnableHighlight(onExisting);
	return onExisting ? Snap_Point : Snap_Nothing;
}

bool RoadCreationSession::Update(const LM::MouseAction& act)
{
	RoadDrawingSession::Update(act);
	SetHighlightTo(GetPointerRoad());

	bool dirHandleEvt = false;
	double prevHandleDir, currHandleDir;
	if (directionHandle != nullptr)
	{
		prevHandleDir = directionHandle->Rotation();
		dirHandleEvt = directionHandle->Update(act);
		currHandleDir = directionHandle->Rotation();
	}

	odr::Vec2D snappedPos;
	auto snapLevel = SnapCursor(snappedPos);
	cursorItem->SetTranslation({ snappedPos[0], snappedPos[1], CursorElevation() });
	cursorItem->EnableHighlight(snapLevel);

	LM::LanePlan currLeftPlan{ PreviewLeftOffsetX2(), g_laneConfig->LeftResult().laneCount };
	LM::LanePlan currRightPlan{ PreviewRightOffsetX2(), g_laneConfig->RightResult().laneCount };
	if (currLeftPlan != stagedLeftPlan || currRightPlan != stagedRightPlan)
	{
		UpdateStagedPreview();
		stagedLeftPlan = currLeftPlan;
		stagedRightPlan = currRightPlan;
	}

	if (dirHandleEvt)
	{
		// Adjust end direction by rotary
		if (prevHandleDir != currHandleDir)
		{
			auto& toRefit = stagedGeometries.back();
			auto refitStartPos = toRefit.geo->get_xy(0);
			auto startHdg = odr::normalize(toRefit.geo->get_grad(0));
			auto endPos = toRefit.geo->get_end_pos();
			auto targetHdg = currHandleDir;
			auto endHdg = odr::Vec2D{ std::cos(targetHdg), std::sin(targetHdg) };
			auto adjustedFit = LM::ConnectRays(refitStartPos, startHdg, endPos, endHdg);

			toRefit.geo = std::move(adjustedFit);

			UpdateStagedPreview();
		}
		flexBoundaryPreview.reset();
		flexRefLinePreview.reset();
	}
	else
	{
		bool pressTrigger = !LM::touchScreen &&
			(act.type == QEvent::Type::MouseButtonPress &&
				act.button == Qt::MouseButton::LeftButton);
		bool releaseTrigger = LM::touchScreen &&
			(act.type == QEvent::Type::MouseButtonRelease
				&& act.button == Qt::MouseButton::LeftButton);
		bool startFomBlank = !startPos.has_value();
		if (startFomBlank)
		{
			if (act.type == QEvent::Type::MouseButtonPress
				&& act.button == Qt::MouseButton::LeftButton)
			{
				startElevation = CursorElevation();
				startPos.emplace(snappedPos);
			}
		}
		if (!startFomBlank && (pressTrigger || releaseTrigger))
		{
			if (flexGeo != nullptr && flexGeo->length > 1.0)
			{
				auto newEnd = flexGeo->get_end_pos();
				auto newHdg = flexGeo->get_end_hdg();
				auto newEnd3 = odr::Vec3D{ newEnd[0], newEnd[1], flexEndElevation };
				directionHandle = std::make_unique<DirectionHandle>(newEnd3, newHdg);
				stagedGeometries.push_back(StagedGeometry
					{
						std::move(flexGeo), flexEndElevation
					}); // Do stage
				UpdateStagedPreview();
			}
			if (!joinAtEnd.expired())
			{
				// force complete
				return false;
			}
		}

		UpdateFlexGeometry();
	}
	return true;
}

bool RoadCreationSession::Cancel()
{
	// in touch screen mode, if all's left is startPos, quit session
	if (!LM::touchScreen && !stagedGeometries.empty() ||
		LM::touchScreen && stagedGeometries.size() > 1)
	{
		// Unstage one
		stagedGeometries.pop_back();
		if (stagedGeometries.empty())
		{
			directionHandle.reset();
		}
		else
		{
			auto newEnd = stagedGeometries.back().geo->get_end_pos();
			auto newHdg = stagedGeometries.back().geo->get_end_hdg();
			directionHandle = std::make_unique<DirectionHandle>(
				odr::Vec3D{ newEnd[0], newEnd[1], stagedGeometries.back().endEleveation}, newHdg);
		}
		if (LM::touchScreen)
		{
			flexRefLinePreview.reset();
			flexBoundaryPreview.reset();
		}
		else
		{
			UpdateFlexGeometry();
		}
		UpdateStagedPreview();
	}
	else if (!LM::touchScreen && startPos.has_value())
	{
		startPos.reset();
		extendFromStart.reset();
		UpdateFlexGeometry();
		confirmButton.reset();
        cancelButton.reset();
	}
	else
	{
		// quit session
		return false;
	}
	return true;
}

odr::RefLine RoadCreationSession::ResultRefLine() const
{
	odr::RefLine refLine("", 0);
	std::map<double, double> zControlPoints;
	zControlPoints.emplace(0, startElevation);

	for (auto& staged : stagedGeometries)
	{
		auto localGeo = staged.geo->clone();
		auto localLength = localGeo->length;
		localGeo->s0 = refLine.length;
		refLine.s0_to_geometry.emplace(refLine.length, std::move(localGeo));
		refLine.length += localLength;
		zControlPoints.emplace(refLine.length, staged.endEleveation);
	}
	if (!stagedGeometries.empty())
	{
		refLine.elevation_profile = LM::CubicSplineGenerator::FromControlPoints(zControlPoints);
	}
	return refLine;
}

LM::type_t RoadCreationSession::PreviewRightOffsetX2() const
{
	return g_laneConfig->RightResult().offsetx2;
}

LM::type_t RoadCreationSession::PreviewLeftOffsetX2() const
{
	return g_laneConfig->LeftResult().offsetx2;
}

bool RoadCreationSession::Complete()
{
	if (g_laneConfig->LeftResult().laneCount + g_laneConfig->RightResult().laneCount == 0)
	{
		spdlog::warn("Cannot create empty road!");
		return true;
	}

	if (!extendFromStart.expired() && extendFromStart.lock() == joinAtEnd.lock())
	{
		spdlog::warn("Self-loop is not supported!");
		return true;
	}

	auto refLine = ResultRefLine();

	if (refLine.length == 0)
	{
		spdlog::warn("Too few control points");
		return true;
	}
	if (refLine.length > LM::SingleDrawMaxLength)
	{
		spdlog::warn("Invalid shape or Road to create is too long");
		return true;
	}

	LM::LaneProfile config(
		g_laneConfig->LeftResult().laneCount, g_laneConfig->LeftResult().offsetx2,
		g_laneConfig->RightResult().laneCount, g_laneConfig->RightResult().offsetx2);
	
	auto newRoad = std::make_shared<LM::Road>(config, refLine);
	newRoad->GenerateAllSectionGraphics();
	
	bool standaloneRoad = true;
	// Which part of newRoad will be newly-created?
	double newPartBegin = 0, newPartEnd = newRoad->Length();
	if (!extendFromStart.expired())
	{
		auto toExtend = extendFromStart.lock();
		auto joinPointElevation = toExtend->RefLine().elevation_profile.get(extendFromStartS);
		LM::CubicSplineGenerator::OverwriteSection(
			newRoad->RefLine().elevation_profile, newRoad->Length(), 0, 0, joinPointElevation);
		newPartBegin += toExtend->Length();
		newPartEnd += toExtend->Length();
		int joinResult = LM::Road::JoinRoads(toExtend,
			extendFromStartS == 0 ? odr::RoadLink::ContactPoint_Start : odr::RoadLink::ContactPoint_End,
			newRoad, odr::RoadLink::ContactPoint_Start);
		switch (joinResult)
		{
		case LM::RoadJoin_SelfLoop:
			spdlog::warn("Self-loop is not supported!");
			return false;
		case LM::RoadJoin_DirNoOutlet:
			{
				if (newRoad->Length() > LM::JunctionTrimMax)
				{
					// Make a junction instead of extending if direction mismatch
					newRoad = LM::Road::SplitRoad(newRoad, LM::JunctionTrimMax);
					std::vector<LM::ConnectionInfo> junctionInfo =
					{
						LM::ConnectionInfo{toExtend, odr::RoadLink::ContactPoint_End},
						LM::ConnectionInfo{newRoad, odr::RoadLink::ContactPoint_Start}
					};
					auto junction = std::make_shared<LM::Junction>();
					auto errorCode = junction->CreateFrom(junctionInfo);
					if (errorCode == LM::Junction_NoError)
					{
						IDGenerator::ForType(IDType::Road)->NotifyChange(std::stoi(toExtend->ID()));
						break;
					}
				}
			}
			spdlog::warn("Roads with opposite direction cannot be joined!");
			return false;
		default:
			// Okay
			standaloneRoad = false;
			newRoad = toExtend;
			break;
		}
	}

	if (!joinAtEnd.expired())
	{
		auto toJoin = joinAtEnd.lock();
		auto copyOfToJoin = toJoin;

		auto joinPointElevation = toJoin->RefLine().elevation_profile.get(joinAtEndS);
		LM::CubicSplineGenerator::OverwriteSection(
			newRoad->RefLine().elevation_profile, newRoad->Length(),
			newRoad->Length(), newRoad->Length(), joinPointElevation);

		int joinResult = LM::Road::JoinRoads(
			newRoad, odr::RoadLink::ContactPoint_End,
			toJoin, joinAtEndS == 0 ? odr::RoadLink::ContactPoint_Start : odr::RoadLink::ContactPoint_End);
		switch (joinResult)
		{
		case LM::RoadJoin_SelfLoop:
			spdlog::warn("Self-loop is not supported!");
			return false;
		case LM::RoadJoin_DirNoOutlet:
			if (newRoad->Length() > LM::JunctionTrimMin)
			{
				// Make a junction instead of extending if direction mismatch
				LM::Road::SplitRoad(newRoad, newRoad->Length() - LM::JunctionTrimMin);
				std::vector<LM::ConnectionInfo> junctionInfo =
				{
					LM::ConnectionInfo{toJoin, odr::RoadLink::ContactPoint_Start},
					LM::ConnectionInfo{newRoad, odr::RoadLink::ContactPoint_End}
				};
				auto junction = std::make_shared<LM::Junction>();
				auto errorCode = junction->CreateFrom(junctionInfo);
				if (errorCode == LM::Junction_NoError)
				{
					IDGenerator::ForType(IDType::Road)->NotifyChange(std::stoi(toJoin->ID()));
					break;
				}
			}
			spdlog::warn("Roads with opposite direction cannot be joined!");
			return false;
		default:
			// Okay
			standaloneRoad = false;
			world->allRoads.erase(copyOfToJoin);
			world->allRoads.insert(newRoad);
			break;
		} 
	}

	if (standaloneRoad)
	{
		world->allRoads.insert(newRoad);
	}
	
	bool success = CreateJunctionAtZOverlap(std::move(newRoad), newPartBegin, newPartEnd);
	if (success)
	{
		UpdateEndMarkings();
	}
	return success;
}

void RoadCreationSession::GenerateHintLines(const odr::RefLine& refLine,
	odr::Line3D& centerPath, odr::Line3D& boundaryPathR, odr::Line3D& boundaryPathL)
{
	centerPath.clear();
	boundaryPathR.clear();
	boundaryPathL.clear();
	if (refLine.length > 1e-2)
	{
		const int Division = std::max(12, std::min(125, static_cast<int>(std::ceil(refLine.length / 4)) + 1));
		auto flexLen = refLine.length;
			
		for (int i = 0; i != Division; ++i)
		{
			auto s = flexLen / (Division - 1) * i;
			auto p = refLine.get_xyz(s);
			centerPath.push_back(odr::add(p, odr::Vec3D{0, 0, 0.03})); // not obstructed by surface
		}

		// right boundary
		LM::type_t offsetX2 = g_laneConfig->RightResult().laneCount != 0 ? PreviewRightOffsetX2() : PreviewLeftOffsetX2();
		double t = (offsetX2 - g_laneConfig->RightResult().laneCount * 2) * LM::LaneWidth / 2;
		for (int i = 0; i != Division; ++i)
		{
			auto s = flexLen / (Division - 1) * i;
			auto p = refLine.get_xyz(s, t);
			boundaryPathR.push_back(p);
		}

		// left boundary
		offsetX2 = g_laneConfig->LeftResult().laneCount != 0 ? PreviewLeftOffsetX2() : PreviewRightOffsetX2();
		t = (offsetX2 + g_laneConfig->LeftResult().laneCount * 2) * LM::LaneWidth / 2;
		for (int i = 0; i != Division; ++i)
		{
			auto s = flexLen / (Division - 1) * i;
			auto p = refLine.get_xyz(s, t);
			boundaryPathL.push_back(p);
		}
	}
}

void RoadCreationSession::UpdateFlexGeometry()
{
	odr::Vec2D snappedPos;
	SnapCursor(snappedPos);

	flexRefLinePreview.reset();
	flexBoundaryPreview.reset();

	if (startPos.has_value() || !stagedGeometries.empty())
	{
		odr::Line3D flexRefLinePath, flexBoundaryPathR, flexBoundaryPathL;

		odr::Vec2D localStartPos, localStartDir;
		double localStartZ;
		if (stagedGeometries.empty())
		{
			localStartPos = startPos.value();
			localStartDir = extendFromStart.expired() ? odr::sub(snappedPos, localStartPos) : ExtendFromDir();
			localStartZ = startElevation;
		}
		else
		{
			const auto& geo = stagedGeometries.back().geo;
			localStartPos = geo->get_xy(geo->length);
			localStartDir = geo->get_grad(geo->length);
			localStartZ = stagedGeometries.back().endEleveation;
		}
		localStartDir = odr::normalize(localStartDir);

		if (joinAtEnd.expired())
		{
			flexGeo = LM::FitArcOrLine(localStartPos, localStartDir, snappedPos);
		}
		else
		{
			flexGeo = LM::ConnectRays(localStartPos, localStartDir, snappedPos, JoinAtEndDir());
		}
		flexEndElevation = CursorElevation();

		if (flexGeo->length > 0)
		{
			auto elevationProfile = LM::CubicSplineGenerator::FromControlPoints(
				std::map<double, double>{{0, localStartZ}, { flexGeo->length, flexEndElevation }}
			);
			double avgGrade = (flexEndElevation - localStartZ) / flexGeo->length;
			double gradFrac = std::min(1.0, std::abs(avgGrade) / 0.15); // 15% is considered too steep
			const QColor flatColor = Qt::gray;
			const QColor steepColor = avgGrade > 0 ? Qt::darkMagenta : Qt::darkCyan;
			QColor gradColor(flatColor.red() * (1 - gradFrac) + steepColor.red() * gradFrac,
				flatColor.green() * (1 - gradFrac) + steepColor.green() * gradFrac,
				flatColor.blue() * (1 - gradFrac) + steepColor.blue() * gradFrac);

			std::vector<std::unique_ptr<odr::RoadGeometry>> allGeo;
			allGeo.emplace_back(flexGeo->clone());
			odr::RefLine flexRefLine("", 0);
			flexRefLine.s0_to_geometry.emplace(0, flexGeo->clone());
			flexRefLine.length = flexGeo->length;
			flexRefLine.elevation_profile = elevationProfile;
			GenerateHintLines(flexRefLine, flexRefLinePath, flexBoundaryPathR, flexBoundaryPathL);

			flexRefLinePreview.emplace(flexRefLinePath, 0.3, Qt::darkGreen);
			flexBoundaryPreview.emplace(flexBoundaryPathR, flexBoundaryPathL, gradColor);
		}
	}
}

void RoadCreationSession::UpdateStagedPreview()
{
	odr::Line3D stagedRefLinePath, stagedBoundaryPathL, stagedBoundaryPathR;

	if (!stagedGeometries.empty() && stagedGeometries.front().geo->length != 0)
	{
		auto resultRefLine = ResultRefLine();
		GenerateHintLines(resultRefLine, stagedRefLinePath, stagedBoundaryPathR, stagedBoundaryPathL);
		stagedRefLinePreview.emplace(stagedRefLinePath, 0.25, Qt::darkGreen);
		stagedBoundaryPreview.emplace(stagedBoundaryPathR, stagedBoundaryPathL, Qt::gray);

        auto end = stagedGeometries.back().geo->get_end_pos();
        odr::Vec3D buttonPos{ end[0], end[1], stagedGeometries.back().endEleveation + 10.0 };
    
		confirmButton.emplace(buttonPos, QPixmap(":/icons/confirm.png"), QRect(-40, -60, 60, 60), Qt::Key_Space);
		cancelButton.emplace(buttonPos, QPixmap(":/icons/cancel.png"), QRect(40, -60, 60, 60), Qt::Key_Escape);
	}
	else
	{
		stagedRefLinePreview.reset();
		stagedBoundaryPreview.reset();
		confirmButton.reset();
		cancelButton.reset();

		if (startPos.has_value())
		{
            odr::Vec3D buttonPos{ startPos.value()[0], startPos.value()[1], startElevation + 10.0 };
            confirmButton.emplace(buttonPos, QPixmap(":/icons/confirm.png"), QRect(-40, -60, 60, 60), Qt::Key_Space);
            cancelButton.emplace(buttonPos, QPixmap(":/icons/cancel.png"), QRect(40, -60, 60, 60), Qt::Key_Escape);
		}
	}
}
