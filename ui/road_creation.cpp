#include "road_drawing.h"
#include "curve_fitting.h"
#include "CreateRoadOptionWidget.h"
#include "map_view.h"
#include "map_view_gl.h"
#include "junction.h"
#include "constants.h"
#include "road_overlaps.h"

#include <QGraphicsSceneMouseEvent>
#include <math.h>

extern SectionProfileConfigWidget* g_createRoadOption;
extern int8_t g_createRoadElevationOption;

RoadCreationSession::DirectionHandle::DirectionHandle(odr::Vec3D _center, double _angle) :
	center(_center), angle(_angle)
{
	UpdateGraphics();
}

RoadCreationSession::DirectionHandle::~DirectionHandle()
{
	if (graphicsIndex.has_value())
	{
		RoadRunner::g_mapViewGL->RemoveItem(*graphicsIndex, true);
	}
}

bool RoadCreationSession::DirectionHandle::Update(const RoadRunner::MouseAction& act)
{
	odr::Vec2D hitPos;
	auto hit = rayHitLocal(hitPos);
	if (act.type == QEvent::Type::MouseButtonPress && hit)
	{
		dragging = true;
		deltaRotation = angle - std::atan2(hitPos[1], hitPos[0]);
	}
	else if (act.type == QEvent::Type::MouseButtonRelease && dragging)
	{
		dragging = false;
		UpdateGraphics();
	}
	else if (dragging)
	{
		angle = std::atan2(hitPos[1], hitPos[0]) + deltaRotation;
		UpdateGraphics();
	}
	return dragging || hit;
}

double RoadCreationSession::DirectionHandle::Rotation() const
{
	return angle;
}

bool RoadCreationSession::DirectionHandle::rayHitLocal(odr::Vec2D& outPos) const
{
	double cameraZ = RoadRunner::g_CameraPosition[2];
	double portion = (cameraZ - center[2]) / cameraZ;
	odr::Vec2D cameraXY{ RoadRunner::g_CameraPosition[0], RoadRunner::g_CameraPosition[1] };
	auto offsetAtGround = odr::sub(RoadRunner::g_PointerOnGround, cameraXY);
	auto offsetAtLevel = odr::mut(portion, offsetAtGround);
	odr::Vec2D centerXY{ center[0], center[1] };
	outPos = odr::sub(odr::add(cameraXY, offsetAtLevel), centerXY);

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
	if (graphicsIndex.has_value())
	{
		RoadRunner::g_mapViewGL->RemoveItem(*graphicsIndex, true);
	}

	graphicsIndex = RoadRunner::g_mapViewGL->AddPoly(roundBoundary, dragging ? Qt::green : Qt::darkGreen, true);
}

RoadCreationSession::RoadCreationSession()
{

}

RoadDrawingSession::SnapResult RoadCreationSession::SnapCursor(odr::Vec2D& point)
{
	// Snap to existing road
	if (!startPos.has_value())
	{
		extendFromStart.reset();
		auto snapFirstResult = SnapFirstPointToExisting(point);
		if (snapFirstResult != RoadDrawingSession::Snap_Nothing)
		{
			return snapFirstResult;
		}
	}
	else
	{
		joinAtEnd.reset();
		auto snapLastResult = SnapLastPointToExisting(point);
		if (snapLastResult != RoadDrawingSession::Snap_Nothing)
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
		auto projLength = odr::dot(start2Point, localStartDir);
		projLength = std::max(0.0, projLength);
		auto projected = odr::add(localStartPos, odr::mut(projLength, localStartDir));
		if (odr::euclDistance(point, projected) < SnapDistFromScale())
		{
			point = projected;
			return RoadDrawingSession::Snap_Line;
		}
	}
	return RoadDrawingSession::Snap_Nothing;
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

RoadDrawingSession::SnapResult RoadCreationSession::SnapFirstPointToExisting(odr::Vec2D& point)
{
	auto g_pointerRoad = GetPointerRoad();
	if (g_pointerRoad == nullptr) return RoadDrawingSession::Snap_Nothing;

	const double snapThreshold = SnapDistFromScale();
	double snapS = RoadRunner::g_PointerRoadS;
	bool onExisting = false;
	if (RoadRunner::g_PointerRoadS < snapThreshold &&
		dynamic_cast<RoadRunner::DirectJunction*>(g_pointerRoad->predecessorJunction.get()) == nullptr &&
		IsElevationConsistWithExtend())
	{
		snapS = 0;
		extendFromStart = g_pointerRoad;
		extendFromStartS = 0;
	}
	else if (RoadRunner::g_PointerRoadS > g_pointerRoad->Length() - snapThreshold &&
		dynamic_cast<RoadRunner::DirectJunction*>(g_pointerRoad->successorJunction.get()) == nullptr &&
		IsElevationConsistWithExtend())
	{
		snapS = g_pointerRoad->Length();
		extendFromStart = g_pointerRoad;
		extendFromStartS = g_pointerRoad->Length();
	}

	if (!extendFromStart.expired())
	{
		// only snap to ends
		point = g_pointerRoad->generated.get_xy(snapS);
		onExisting = true;
	}
	return onExisting ? RoadDrawingSession::Snap_Point : RoadDrawingSession::Snap_Nothing;
}

RoadDrawingSession::SnapResult RoadCreationSession::SnapLastPointToExisting(odr::Vec2D& point)
{
	auto g_PointerRoad = GetPointerRoad();
	if (g_PointerRoad == nullptr) return RoadDrawingSession::Snap_Nothing;

	bool onExisting = false;

	// Join to existing
	double snapS;
	const double snapThreshold = SnapDistFromScale();
	if (RoadRunner::g_PointerRoadS < snapThreshold &&
		dynamic_cast<RoadRunner::DirectJunction*>(g_PointerRoad->predecessorJunction.get()) == nullptr &&
		IsElevationConsistWithExtend())
	{
		snapS = 0;
		joinAtEnd = g_PointerRoad;
	}
	else if (RoadRunner::g_PointerRoadS > g_PointerRoad->Length() - snapThreshold &&
		dynamic_cast<RoadRunner::DirectJunction*>(g_PointerRoad->successorJunction.get()) == nullptr &&
		IsElevationConsistWithExtend())
	{
		snapS = g_PointerRoad->Length();
		joinAtEnd = g_PointerRoad;
	}
	if (!joinAtEnd.expired())
	{
		point = g_PointerRoad->generated.get_xy(snapS);
		onExisting = true;
		joinAtEndS = snapS;
	}
	cursorItem->EnableHighlight(onExisting);
	return onExisting ? RoadDrawingSession::Snap_Point : RoadDrawingSession::Snap_Nothing;
}

bool RoadCreationSession::Update(const RoadRunner::MouseAction& act)
{
	RoadDrawingSession::Update(act);
	auto g_PointerRoad = GetPointerRoad();
	SetHighlightTo(g_PointerRoad);

	bool dirHandleEvt = false;
	double prevHandleDir, currHandleDir;
	if (directionHandle != nullptr)
	{
		prevHandleDir = directionHandle->Rotation();
		dirHandleEvt = directionHandle->Update(act);
		currHandleDir = directionHandle->Rotation();
	}

	auto scenePos = RoadRunner::g_PointerOnGround;
	auto snapLevel = SnapCursor(scenePos);
	cursorItem->SetTranslation({ scenePos[0], scenePos[1], 0 });
	cursorItem->EnableHighlight(snapLevel);

	RoadRunner::LanePlan currLeftPlan{ PreviewLeftOffsetX2(), g_createRoadOption->LeftResult().laneCount };
	RoadRunner::LanePlan currRightPlan{ PreviewRightOffsetX2(), g_createRoadOption->RightResult().laneCount };
	if (currLeftPlan != stagedLeftPlan || currRightPlan != stagedRightPlan)
	{
		UpdateStagedFromGeometries(true);
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
			auto adjustedFit = RoadRunner::ConnectRays(refitStartPos, startHdg, endPos, endHdg);

			GenerateHintLines(adjustedFit, toRefit.refLinePreview, toRefit.boundaryPreviewR, toRefit.boundaryPreviewL);
			toRefit.geo = std::move(adjustedFit);

			UpdateStagedFromGeometries();
		}
		flexBoundaryPreview.reset();
		flexRefLinePreview.reset();
	}
	else
	{
		if (act.type == QEvent::Type::MouseButtonPress &&
			act.button == Qt::MouseButton::LeftButton)
		{
			if (!startPos.has_value())
			{
				startPos.emplace(scenePos);
				if (extendFromStart.expired())
				{
					overlapAtStart = g_PointerRoad;
					overlapAtStartS = RoadRunner::g_PointerRoadS;
				}
				else
				{
					overlapAtStart.reset();
				}
			}
			else if (flexGeo != nullptr)
			{
				if (joinAtEnd.expired())
				{
					overlapAtEnd = g_PointerRoad;
					overlapAtEndS = RoadRunner::g_PointerRoadS;
				}
				else
				{
					overlapAtEnd.reset();
				}

				auto newEnd = flexGeo->get_end_pos();
				auto newHdg = flexGeo->get_end_hdg();
				directionHandle = std::make_unique<DirectionHandle>(
					odr::Vec3D{ newEnd[0], newEnd[1], 0 }, newHdg);
				stagedGeometries.push_back(StagedGeometry
					{
						std::move(flexGeo), flexRefLinePath, flexBoundaryPathL, flexBoundaryPathR
					}); // Do stage
				UpdateStagedFromGeometries();
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

bool RoadCreationSession::Update(const RoadRunner::KeyPressAction& act)
{
	if (act.key == Qt::Key_Escape)
	{
		if (!stagedGeometries.empty())
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
					odr::Vec3D{ newEnd[0], newEnd[1], 0 }, newHdg);
			}
			UpdateFlexGeometry();
			UpdateStagedFromGeometries();
		}
		else if (startPos.has_value())
		{
			startPos.reset();
			extendFromStart.reset();
			UpdateFlexGeometry();
		}
		else
		{
			// quit session
			return false;
		}
	}
	return true;
}

odr::RefLine RoadCreationSession::ResultRefLine() const
{
	odr::RefLine refLine("", 0);
	for (auto& staged : stagedGeometries)
	{
		const auto& localGeo = staged.geo;
		auto localLength = localGeo->length;
		localGeo->s0 = refLine.length;
		refLine.s0_to_geometry.emplace(refLine.length, localGeo->clone());
		refLine.length += localLength;
	}
	refLine.elevation_profile = odr::CubicSpline(0);
	return refLine;
}

RoadRunner::type_t RoadCreationSession::PreviewRightOffsetX2() const
{
	return g_createRoadOption->RightResult().offsetx2;
}

RoadRunner::type_t RoadCreationSession::PreviewLeftOffsetX2() const
{
	return g_createRoadOption->LeftResult().offsetx2;
}

bool RoadCreationSession::Complete()
{
	if (g_createRoadOption->LeftResult().laneCount + g_createRoadOption->RightResult().laneCount == 0)
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
	if (refLine.length > RoadRunner::SingleDrawMaxLength)
	{
		spdlog::warn("Invalid shape or Road to create is too long");
		return true;
	}

	RoadRunner::LaneProfile config(
		g_createRoadOption->LeftResult().laneCount, g_createRoadOption->LeftResult().offsetx2,
		g_createRoadOption->RightResult().laneCount, g_createRoadOption->RightResult().offsetx2);
	
	auto newRoad = std::make_shared<RoadRunner::Road>(config, refLine);
	newRoad->GenerateAllSectionGraphics();
	
	bool standaloneRoad = true;
	// Which part of newRoad will be newly-created?
	double newPartBegin = 0, newPartEnd = newRoad->Length();
	if (!extendFromStart.expired())
	{
		auto toExtend = extendFromStart.lock();
		auto joinPointElevation = toExtend->RefLine().elevation_profile.get(extendFromStartS);
		RoadRunner::CubicSplineGenerator::OverwriteSection(
			newRoad->RefLine().elevation_profile, newRoad->Length(), 0, 0, joinPointElevation);
		newPartBegin += toExtend->Length();
		newPartEnd += toExtend->Length();
		int joinResult = RoadRunner::Road::JoinRoads(toExtend,
			extendFromStartS == 0 ? odr::RoadLink::ContactPoint_Start : odr::RoadLink::ContactPoint_End,
			newRoad, odr::RoadLink::ContactPoint_Start);
		switch (joinResult)
		{
		case RoadRunner::RoadJoin_SelfLoop:
			spdlog::warn("Self-loop is not supported!");
			return false;
		case RoadRunner::RoadJoin_DirNoOutlet:
			{
				if (newRoad->Length() > RoadRunner::JunctionTrimMax)
				{
					// Make a junction instead of extending if direction mismatch
					newRoad = RoadRunner::Road::SplitRoad(newRoad, RoadRunner::JunctionTrimMax);
					std::vector<RoadRunner::ConnectionInfo> junctionInfo =
					{
						RoadRunner::ConnectionInfo{toExtend, odr::RoadLink::ContactPoint_End},
						RoadRunner::ConnectionInfo{newRoad, odr::RoadLink::ContactPoint_Start}
					};
					auto junction = std::make_shared<RoadRunner::Junction>();
					auto errorCode = junction->CreateFrom(junctionInfo);
					if (errorCode == RoadRunner::Junction_NoError)
					{
						IDGenerator::ForRoad()->NotifyChange(toExtend->ID());
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
		RoadRunner::CubicSplineGenerator::OverwriteSection(
			newRoad->RefLine().elevation_profile, newRoad->Length(),
			newRoad->Length(), newRoad->Length(), joinPointElevation);

		int joinResult = RoadRunner::Road::JoinRoads(
			newRoad, odr::RoadLink::ContactPoint_End,
			toJoin, joinAtEndS == 0 ? odr::RoadLink::ContactPoint_Start : odr::RoadLink::ContactPoint_End);
		switch (joinResult)
		{
		case RoadRunner::RoadJoin_SelfLoop:
			spdlog::warn("Self-loop is not supported!");
			return false;
		case RoadRunner::RoadJoin_DirNoOutlet:
			if (newRoad->Length() > RoadRunner::JunctionTrimMin)
			{
				// Make a junction instead of extending if direction mismatch
				RoadRunner::Road::SplitRoad(newRoad, newRoad->Length() - RoadRunner::JunctionTrimMin);
				std::vector<RoadRunner::ConnectionInfo> junctionInfo =
				{
					RoadRunner::ConnectionInfo{toJoin, odr::RoadLink::ContactPoint_Start},
					RoadRunner::ConnectionInfo{newRoad, odr::RoadLink::ContactPoint_End}
				};
				auto junction = std::make_shared<RoadRunner::Junction>();
				auto errorCode = junction->CreateFrom(junctionInfo);
				if (errorCode == RoadRunner::Junction_NoError)
				{
					IDGenerator::ForRoad()->NotifyChange(toJoin->ID());
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
	
	bool success;
	if (g_createRoadElevationOption == 0)
	{
		success = TryCreateJunction(std::move(newRoad), newPartBegin, newPartEnd,
			overlapAtStart, overlapAtStartS, overlapAtEnd, overlapAtEndS);
	}
	else
	{
		success = TryCreateBridgeAndTunnel(std::move(newRoad), newPartBegin, newPartEnd);
	}
	if (success)
	{
		UpdateEndMarkings();
	}
	return success;
}

RoadCreationSession::~RoadCreationSession()
{
}

void RoadCreationSession::GenerateHintLines(const std::unique_ptr<odr::RoadGeometry>& geo,
	odr::Line3D& centerPath, odr::Line3D& boundaryPathR, odr::Line3D& boundaryPathL)
{
	const int Division = 30;
	centerPath.clear();
	boundaryPathR.clear();
	boundaryPathL.clear();
	if (geo != nullptr && geo->length > 1e-2)
	{
		auto flexLen = geo->length;
			
		for (int i = 0; i != Division; ++i)
		{
			auto s = flexLen / (Division - 1) * i;
			auto p = geo->get_xy(s);
			centerPath.push_back(odr::Vec3D{ p[0], p[1], 0.01 });
		}

		// right boundary
		RoadRunner::type_t offsetX2 = g_createRoadOption->RightResult().laneCount != 0 ? -PreviewRightOffsetX2() : -PreviewLeftOffsetX2();
		double t = (offsetX2 + g_createRoadOption->RightResult().laneCount * 2) * RoadRunner::LaneWidth / 2;
		for (int i = 0; i != Division; ++i)
		{
			auto s = flexLen / (Division - 1) * i;
			auto p = geo->get_xy(s);
			auto grad = odr::normalize(geo->get_grad(s));
			auto offset = odr::mut(t, odr::Vec2D{ grad[1], -grad[0] });
			p = odr::add(p, offset);
			boundaryPathR.push_back(odr::Vec3D{ p[0], p[1], 0 });
		}

		// left boundary
		offsetX2 = g_createRoadOption->LeftResult().laneCount != 0 ? PreviewLeftOffsetX2() : PreviewRightOffsetX2();
		t = -(offsetX2 + g_createRoadOption->LeftResult().laneCount * 2) * RoadRunner::LaneWidth / 2;
		for (int i = 0; i != Division; ++i)
		{
			auto s = flexLen / (Division - 1) * i;
			auto p = geo->get_xy(s);
			auto grad = odr::normalize(geo->get_grad(s));
			auto offset = odr::mut(t, odr::Vec2D{ grad[1], -grad[0] });
			p = odr::add(p, offset);
			boundaryPathL.push_back(odr::Vec3D{ p[0], p[1], 0 });
		}
	}
}

void RoadCreationSession::UpdateFlexGeometry()
{
	auto scenePos = RoadRunner::g_PointerOnGround;
	SnapCursor(scenePos);

	// Update flex geo
	if (startPos.has_value() || !stagedGeometries.empty())
	{
		odr::Vec2D localStartPos, localStartDir;
		if (stagedGeometries.empty())
		{
			localStartPos = startPos.value();
			localStartDir = extendFromStart.expired() ? odr::sub(scenePos, localStartPos) : ExtendFromDir();
		}
		else
		{
			const auto& geo = stagedGeometries.back().geo;
			localStartPos = geo->get_xy(geo->length);
			localStartDir = geo->get_grad(geo->length);
		}
		localStartDir = odr::normalize(localStartDir);

		if (joinAtEnd.expired())
		{
			flexGeo = RoadRunner::FitArcOrLine(localStartPos, localStartDir, scenePos);
		}
		else
		{
			flexGeo = RoadRunner::ConnectRays(localStartPos, localStartDir, scenePos, JoinAtEndDir());
		}

		GenerateHintLines(flexGeo, flexRefLinePath, flexBoundaryPathR, flexBoundaryPathL);
	}
	else
	{
		flexRefLinePath.clear();
		flexBoundaryPathR.clear();
		flexBoundaryPathL.clear();
	}

	if (!flexRefLinePath.empty())
	{
		flexRefLinePreview.emplace(flexRefLinePath, 0.3, Qt::green);
		flexBoundaryPreview.emplace(flexBoundaryPathR, flexBoundaryPathL, Qt::gray);
	}
	else
	{
		flexRefLinePreview.reset();
		flexBoundaryPreview.reset();
	}
}

void RoadCreationSession::UpdateStagedFromGeometries(bool lanePlanChanged)
{
	stagedRefLinePath.clear();
	stagedBoundaryPathL.clear();
	stagedBoundaryPathR.clear();
	for (auto& staged : stagedGeometries)
	{
		if (lanePlanChanged)
		{
			GenerateHintLines(staged.geo, staged.refLinePreview, staged.boundaryPreviewR, staged.boundaryPreviewL);
		}
		stagedRefLinePath.insert(stagedRefLinePath.end(), staged.refLinePreview.begin(), staged.refLinePreview.end());
		stagedBoundaryPathL.insert(stagedBoundaryPathL.end(), staged.boundaryPreviewL.begin(), staged.boundaryPreviewL.end());
		stagedBoundaryPathR.insert(stagedBoundaryPathR.end(), staged.boundaryPreviewR.begin(), staged.boundaryPreviewR.end());
	}

	stagedRefLinePreview.emplace(stagedRefLinePath, 0.25, Qt::green);
	stagedBoundaryPreview.emplace(stagedBoundaryPathR, stagedBoundaryPathL, Qt::gray);
}
