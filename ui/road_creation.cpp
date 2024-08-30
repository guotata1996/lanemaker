#include "road_drawing.h"
#include "curve_fitting.h"
#include "CreateRoadOptionWidget.h"
#include "map_view.h"
#include "junction.h"
#include "constants.h"
#include "road_overlaps.h"

#include <QGraphicsSceneMouseEvent>
#include <math.h>

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;
extern SectionProfileConfigWidget* g_createRoadOption;
extern int8_t g_createRoadElevationOption;
extern MapView* g_mapView;

RoadCreationSession::DirectionHandle::DirectionHandle()
{
	QMatrix rotTrans;
	rotTrans.rotate(90);
	auto pic = QPixmap(":/icons/dir_cursor.png").transformed(rotTrans);
	setPixmap(pic);
	setOffset(-pic.width() / 2, -pic.height() / 2);
	setZValue(128);
}

bool RoadCreationSession::DirectionHandle::Update(const RoadRunner::MouseAction& act)
{
	if (!isVisible())
		return false;
	auto localPos = QPointF(act.sceneX, act.sceneY) - pos();
	if (act.type == QEvent::Type::MouseButtonPress && contains(localPos))
	{
		dragging = true;
		deltaRotation = rotation() - std::atan2(localPos.y(), localPos.x()) * 180 / M_PI;
	}
	else if (act.type == QEvent::Type::MouseButtonRelease && dragging)
	{
		dragging = false;
	}
	else if (dragging)
	{
		double newRotation = std::atan2(localPos.y(), localPos.x()) * 180 / M_PI + deltaRotation;
		setRotation(newRotation);
	}
	return dragging;
}

bool RoadCreationSession::DirectionHandle::contains(const QPointF& point) const
{
	double dis = std::sqrt(std::pow(point.x(), 2) + std::pow(point.y(), 2));
	dis /= scale();
	return 86 < dis && dis < 132; // Pixel count from raw image
}

void RoadCreationSession::DirectionHandle::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
	this->setScale(0.4 / g_mapView->Zoom());
	QGraphicsPixmapItem::paint(painter, option, widget);
}

RoadCreationSession::RoadCreationSession(QGraphicsView* aView):
	RoadDrawingSession(aView)
{
	flexRefLinePreview = scene->addPath(flexRefLinePath);
	flexRefLinePreview->setZValue(129); // always on top of boundary preview
	stagedRefLinePreview = scene->addPath(stagedRefLinePath);
	stagedRefLinePreview->setZValue(129);

	QPen refLinePen;
	refLinePen.setColor(Qt::blue);
	stagedRefLinePreview->setPen(refLinePen);
	refLinePen.setStyle(Qt::DotLine);
	flexRefLinePreview->setPen(refLinePen);

	stagedBoundaryPreview = scene->addPath(stagedBoundaryPath);
	flexBoundaryPreview = scene->addPath(flexBoundaryPath);

	flexBoundaryPreview->setPen(QPen(Qt::gray));
	flexBoundaryPreview->setBrush(Qt::lightGray);
	stagedBoundaryPreview->setPen(QPen(Qt::gray));
	stagedBoundaryPreview->setBrush(Qt::lightGray);

	directionHandle = new DirectionHandle;
	scene->addItem(directionHandle);
	directionHandle->hide();
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
			localStartPos = extendFromStart.lock()->RefLine().get_xy(extendFromStartS);
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
	auto g_road = g_PointerRoad.lock();
	if (g_road == nullptr) return RoadDrawingSession::Snap_Nothing;

	const double snapThreshold = SnapDistFromScale();
	double snapS = g_PointerRoadS;
	bool onExisting = false;
	if (g_PointerRoadS < snapThreshold &&
		dynamic_cast<RoadRunner::DirectJunction*>(g_PointerRoad.lock()->predecessorJunction.get()) == nullptr &&
		IsElevationConsistWithExtend())
	{
		snapS = 0;
		extendFromStart = g_PointerRoad;
		extendFromStartS = 0;
	}
	else if (g_PointerRoadS > g_road->Length() - snapThreshold &&
		dynamic_cast<RoadRunner::DirectJunction*>(g_PointerRoad.lock()->successorJunction.get()) == nullptr &&
		IsElevationConsistWithExtend())
	{
		snapS = g_road->Length();
		extendFromStart = g_PointerRoad;
		extendFromStartS = g_road->Length();
	}

	if (!extendFromStart.expired())
	{
		// only snap to ends
		point = g_road->generated.ref_line.get_xy(snapS);
		onExisting = true;
	}
	return onExisting ? RoadDrawingSession::Snap_Point : RoadDrawingSession::Snap_Nothing;
}

RoadDrawingSession::SnapResult RoadCreationSession::SnapLastPointToExisting(odr::Vec2D& point)
{
	auto g_road = g_PointerRoad.lock();
	if (g_road == nullptr) return RoadDrawingSession::Snap_Nothing;

	bool onExisting = false;

	// Join to existing
	double snapS;
	const double snapThreshold = SnapDistFromScale();
	if (g_PointerRoadS < snapThreshold &&
		dynamic_cast<RoadRunner::DirectJunction*>(g_PointerRoad.lock()->predecessorJunction.get()) == nullptr &&
		IsElevationConsistWithExtend())
	{
		snapS = 0;
		joinAtEnd = g_PointerRoad;
	}
	else if (g_PointerRoadS > g_road->Length() - snapThreshold &&
		dynamic_cast<RoadRunner::DirectJunction*>(g_PointerRoad.lock()->successorJunction.get()) == nullptr &&
		IsElevationConsistWithExtend())
	{
		snapS = g_road->Length();
		joinAtEnd = g_PointerRoad;
	}
	if (!joinAtEnd.expired())
	{
		point = g_road->generated.ref_line.get_xy(snapS);
		onExisting = true;
		joinAtEndS = snapS;
	}
	cursorItem->EnableHighlight(onExisting);
	return onExisting ? RoadDrawingSession::Snap_Point : RoadDrawingSession::Snap_Nothing;
}

bool RoadCreationSession::Update(const RoadRunner::MouseAction& act)
{
	RoadDrawingSession::Update(act);

	SetHighlightTo(g_PointerRoad.lock());
	auto prevHandleDir = directionHandle->rotation();
	bool dirHandleEvt = directionHandle->Update(act);
	auto currHandleDir = directionHandle->rotation();

	odr::Vec2D  scenePos{ act.sceneX, act.sceneY };
	auto snapLevel = SnapCursor(scenePos);
	cursorItem->setPos(QPointF(scenePos[0], scenePos[1]));
	cursorItem->EnableHighlight(snapLevel);
	cursorItem->show();

	if (dirHandleEvt)
	{
		// Adjust end direction by rotary
		cursorItem->hide();
		if (prevHandleDir != currHandleDir)
		{
			auto& toRefit = stagedGeometries.back();
			auto refitStartPos = toRefit.geo->get_xy(0);
			auto startHdg = odr::normalize(toRefit.geo->get_grad(0));
			auto endPos = toRefit.geo->get_end_pos();
			auto targetHdg = currHandleDir / 180 * M_PI;
			auto endHdg = odr::Vec2D{ std::cos(targetHdg), std::sin(targetHdg) };
			auto adjustedFit = RoadRunner::ConnectRays(refitStartPos, startHdg, endPos, endHdg);

			GeneratePainterPath(adjustedFit, toRefit.refLinePreview, toRefit.boundaryPreview);
			toRefit.geo = std::move(adjustedFit);

			UpdateStagedFromGeometries();
			flexBoundaryPreview->hide();
			flexRefLinePreview->hide();
		}
	}
	else
	{
		if (act.type == QEvent::Type::MouseButtonPress)
		{
			if (act.button == Qt::MouseButton::LeftButton)
			{
				if (!startPos.has_value())
				{
					startPos.emplace(scenePos);
					if (extendFromStart.expired())
					{
						overlapAtStart = g_PointerRoad;
						overlapAtStartS = g_PointerRoadS;
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
						overlapAtEndS = g_PointerRoadS;
					}
					else
					{
						overlapAtEnd.reset();
					}

					auto newEnd = flexGeo->get_end_pos();
					auto newHdg = flexGeo->get_end_hdg();
					directionHandle->setPos(newEnd[0], newEnd[1]);
					directionHandle->setRotation(newHdg * 180 / M_PI);
					directionHandle->setScale(0);
					directionHandle->show();
					stagedGeometries.push_back(StagedGeometry
						{
							std::move(flexGeo), flexRefLinePath, flexBoundaryPath
						}); // Do stage
					stagedRefLinePath.addPath(flexRefLinePath);
					stagedRefLinePreview->setPath(stagedRefLinePath);
					stagedBoundaryPath.addPath(flexBoundaryPath);
					stagedBoundaryPreview->setPath(stagedBoundaryPath);
					stagedBoundaryPreview->setZValue(g_createRoadElevationOption >= 0 ? 128 : -128);
				}
				if (!joinAtEnd.expired())
				{
					// force complete
					return false;
				}
			}
			else if (act.button == Qt::MouseButton::RightButton)
			{
				if (!stagedGeometries.empty())
				{
					// Unstage one
					stagedGeometries.pop_back();
					if (stagedGeometries.empty())
					{
						directionHandle->hide();
					}
					else
					{
						auto newEnd = stagedGeometries.back().geo->get_end_pos();
						auto newHdg = stagedGeometries.back().geo->get_end_hdg();
						directionHandle->setPos(newEnd[0], newEnd[1]);
						directionHandle->setRotation(newHdg * 180 / M_PI);
					}
					UpdateStagedFromGeometries();
				}
				else
				{
					startPos.reset();
					extendFromStart.reset();
				}
			}
		}

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
		
			GeneratePainterPath(flexGeo, flexRefLinePath, flexBoundaryPath);
		}
		else
		{
			flexRefLinePath.clear();
			flexBoundaryPath.clear();
		}
		flexRefLinePreview->setPath(flexRefLinePath);
		flexRefLinePreview->show();
		flexBoundaryPreview->setPath(flexBoundaryPath);
		flexBoundaryPreview->setZValue(g_createRoadElevationOption >= 0 ? 128 : -128);
		flexBoundaryPreview->show();
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
				if (newRoad->Length() > RoadRunner::JunctionExtaTrim)
				{
					// Make a junction instead of extending if direction mismatch
					newRoad = RoadRunner::Road::SplitRoad(newRoad, RoadRunner::JunctionExtaTrim);
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
			newPartBegin += toExtend->Length();
			newPartEnd += toExtend->Length();
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
			if (newRoad->Length() > RoadRunner::JunctionExtaTrim)
			{
				// Make a junction instead of extending if direction mismatch
				RoadRunner::Road::SplitRoad(newRoad, newRoad->Length() - RoadRunner::JunctionExtaTrim);
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
	
	if (g_createRoadElevationOption == 0)
	{
		return TryCreateJunction(std::move(newRoad), newPartBegin, newPartEnd, 
			overlapAtStart, overlapAtStartS, overlapAtEnd, overlapAtEndS);
	}
	else
	{
		return TryCreateBridgeAndTunnel(std::move(newRoad), newPartBegin, newPartEnd);
	}
	return true;
}

RoadCreationSession::~RoadCreationSession()
{
	scene->removeItem(stagedRefLinePreview);
	scene->removeItem(stagedBoundaryPreview);
	scene->removeItem(flexRefLinePreview);
	scene->removeItem(flexBoundaryPreview);
	scene->removeItem(cursorItem);
	scene->removeItem(directionHandle);
	SetHighlightTo(nullptr);
}

void RoadCreationSession::GeneratePainterPath(const std::unique_ptr<odr::RoadGeometry>& geo,
	QPainterPath& centerPath, QPainterPath& boundaryPath)
{
	const int Division = 30;
	centerPath.clear();
	boundaryPath.clear();
	if (geo != nullptr)
	{
		auto flexLen = geo->length;
			
		for (int i = 0; i != Division; ++i)
		{
			auto s = flexLen / (Division - 1) * i;
			auto p = geo->get_xy(s);
				
			if (i == 0)
				centerPath.moveTo(p[0], p[1]);
			else
				centerPath.lineTo(p[0], p[1]);
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
			if (i == 0)
				boundaryPath.moveTo(p[0], p[1]);
			else
				boundaryPath.lineTo(p[0], p[1]);
		}

		// left boundary
		offsetX2 = g_createRoadOption->LeftResult().laneCount != 0 ? PreviewLeftOffsetX2() : PreviewRightOffsetX2();
		t = -(offsetX2 + g_createRoadOption->LeftResult().laneCount * 2) * RoadRunner::LaneWidth / 2;
		for (int i = 0; i != Division; ++i)
		{
			auto s = flexLen / (Division - 1) * (Division - 1 - i);
			auto p = geo->get_xy(s);
			auto grad = odr::normalize(geo->get_grad(s));
			auto offset = odr::mut(t, odr::Vec2D{ grad[1], -grad[0] });
			p = odr::add(p, offset);
			boundaryPath.lineTo(p[0], p[1]);
		}
		boundaryPath.closeSubpath();
	}
}

void RoadCreationSession::UpdateStagedFromGeometries()
{
	stagedRefLinePath.clear();
	stagedBoundaryPath.clear();
	for (const auto& staged : stagedGeometries)
	{
		stagedRefLinePath.addPath(staged.refLinePreview);
		stagedBoundaryPath.addPath(staged.boundaryPreview);
	}
	stagedRefLinePreview->setPath(stagedRefLinePath);
	stagedBoundaryPreview->setPath(stagedBoundaryPath);
	stagedBoundaryPreview->setZValue(g_createRoadElevationOption >= 0 ? 128 : -128);
}
