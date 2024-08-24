#include "road_drawing_new.h"
#include "curve_fitting.h"
#include "CreateRoadOptionWidget.h"
#include "map_view.h"

#include <QGraphicsSceneMouseEvent>
#include <math.h>

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;
extern SectionProfileConfigWidget* g_createRoadOption;

extern MapView* g_mapView;

DirectionHandle::DirectionHandle()
{
	QMatrix rotTrans;
	rotTrans.rotate(90);
	auto pic = QPixmap(":/icons/dir_cursor.png").transformed(rotTrans);
	setPixmap(pic);
	setOffset(-pic.width() / 2, -pic.height() / 2);
}

bool DirectionHandle::Update(const RoadRunner::MouseAction& act)
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

bool DirectionHandle::contains(const QPointF& point) const
{
	double dis = std::sqrt(std::pow(point.x(), 2) + std::pow(point.y(), 2));
	dis /= scale();
	return 86 < dis && dis < 132; // Pixel count from raw image
}

void DirectionHandle::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
	this->setScale(0.4 / g_mapView->Zoom());
	QGraphicsPixmapItem::paint(painter, option, widget);
}

RoadCreationSession_NEW::RoadCreationSession_NEW(QGraphicsView* aView):
	RoadDrawingSession(aView)
{
	stagedPreview = scene->addPath(stagedPreviewPath);
	flexPreview = scene->addPath(flexPreviewPath);
	QPen flexPen;
	flexPen.setStyle(Qt::DotLine);
	flexPreview->setPen(flexPen);
	directionHandle = new DirectionHandle;
	scene->addItem(directionHandle);
	directionHandle->hide();
}

bool RoadCreationSession_NEW::SnapCursor(odr::Vec2D& point) const
{
	if (!stagedGeometries.empty())
	{
		const auto& geo = stagedGeometries.back().geo;
		auto localStartPos = geo->get_xy(geo->length);
		auto localStartDir = odr::normalize(geo->get_grad(geo->length));
		auto start2Point = odr::sub(point, localStartPos);
		auto projLength = odr::dot(start2Point, localStartDir);
		projLength = std::max(0.0, projLength);
		auto projected = odr::add(localStartPos, odr::mut(projLength, localStartDir));
		if (odr::euclDistance(point, projected) < SnapDistFromScale())
		{
			point = projected;
			return true;
		}
	}
	return false;
}

bool RoadCreationSession_NEW::Update(const RoadRunner::MouseAction& act)
{
	SetHighlightTo(g_PointerRoad.lock());
	auto prevHandleDir = directionHandle->rotation();
	bool dirHandleEvt = directionHandle->Update(act);
	auto currHandleDir = directionHandle->rotation();

	odr::Vec2D  scenePos{ act.sceneX, act.sceneY };
	SnapCursor(scenePos);

	if (dirHandleEvt)
	{
		cursorItem->hide();
		if (prevHandleDir != currHandleDir)
		{
			auto& toRefit = stagedGeometries.back();
			auto startPos = toRefit.geo->get_xy(0);
			auto startHdg = odr::normalize(toRefit.geo->get_grad(0));
			auto endPos = toRefit.geo->get_end_pos();
			auto targetHdg = currHandleDir / 180 * M_PI;
			auto endHdg = odr::Vec2D{ std::cos(targetHdg), std::sin(targetHdg) };
			auto adjustedFit = RoadRunner::ConnectRays(startPos, startHdg, endPos, endHdg);

			GeneratePainterPath(adjustedFit, toRefit.preview);
			toRefit.geo = std::move(adjustedFit);

			stagedPreviewPath.clear();
			for (const auto& staged : stagedGeometries)
			{
				stagedPreviewPath.addPath(staged.preview);
			}
			stagedPreview->setPath(stagedPreviewPath);
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
				}
				else if (flexGeo != nullptr)
				{
					auto newEnd = flexGeo->get_end_pos();
					auto newHdg = flexGeo->get_end_hdg();
					directionHandle->setPos(newEnd[0], newEnd[1]);
					directionHandle->setRotation(newHdg * 180 / M_PI);
					directionHandle->setScale(0);
					directionHandle->show();
					stagedGeometries.push_back(StagedGeometry
						{
							std::move(flexGeo), flexPreviewPath
						}); // Do stage
					stagedPreviewPath.addPath(flexPreviewPath);
					stagedPreview->setPath(stagedPreviewPath);
				}
			}
			else if (act.button == Qt::MouseButton::RightButton)
			{
				stagedPreviewPath.clear();
				if (!stagedGeometries.empty())
				{
					// Unstage one
					stagedGeometries.pop_back();
					for (const auto& staged : stagedGeometries)
					{
						stagedPreviewPath.addPath(staged.preview);
					}
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
				}
				else
				{
					startPos.reset();
				}
				stagedPreview->setPath(stagedPreviewPath);
			}
		}

		cursorItem->setPos(QPointF(act.sceneX, act.sceneY));
		cursorItem->show();

		// Update flex geo
		if (startPos.has_value() || !stagedGeometries.empty())
		{
			odr::Vec2D localStartPos, localStartDir;
			if (stagedGeometries.empty())
			{
				localStartPos = startPos.value();
				localStartDir = startDir.value_or(odr::sub(scenePos, localStartPos));
			}
			else
			{
				const auto& geo = stagedGeometries.back().geo;
				localStartPos = geo->get_xy(geo->length);
				localStartDir = geo->get_grad(geo->length);
			}
			localStartDir = odr::normalize(localStartDir);
			flexGeo = RoadRunner::FitArcOrLine(localStartPos, localStartDir, scenePos);
		
			GeneratePainterPath(flexGeo, flexPreviewPath);
		}
		else
		{
			flexPreviewPath.clear();
		}
		flexPreview->setPath(flexPreviewPath);
	}
	return true;
}

bool RoadCreationSession_NEW::Complete()
{
	odr::RefLine refLine("", 0);
	for (auto& staged : stagedGeometries)
	{
		auto& localGeo = staged.geo;
		auto localLength = localGeo->length;
		localGeo->s0 = refLine.length;
		refLine.s0_to_geometry.emplace(refLine.length, std::move(localGeo));
		refLine.length += localLength;
	}

	if (refLine.length == 0)
	{
		spdlog::warn("Too few control points");
		return true;
	}

	RoadRunner::LaneProfile config(
		g_createRoadOption->LeftResult().laneCount, g_createRoadOption->LeftResult().offsetx2,
		g_createRoadOption->RightResult().laneCount, g_createRoadOption->RightResult().offsetx2);

	refLine.elevation_profile = odr::CubicSpline(0);
	auto newRoad = std::make_shared<RoadRunner::Road>(config, refLine);
	newRoad->GenerateAllSectionGraphics();
	world->allRoads.insert(newRoad);

	return true;
}

RoadCreationSession_NEW::~RoadCreationSession_NEW()
{
	scene->removeItem(stagedPreview);
	scene->removeItem(flexPreview);
	scene->removeItem(cursorItem);
	scene->removeItem(directionHandle);
	SetHighlightTo(nullptr);
}

void RoadCreationSession_NEW::GeneratePainterPath(const std::unique_ptr<odr::RoadGeometry>& geo,
	QPainterPath& path)
{
	path.clear();
	if (geo != nullptr)
	{
		auto flexLen = geo->length;
		for (int i = 0; i != 30; ++i)
		{
			auto s = flexLen / 29 * i;
			auto p = geo->get_xy(s);
			if (i == 0)
				path.moveTo(p[0], p[1]);
			else
				path.lineTo(p[0], p[1]);
		}
	}
}
