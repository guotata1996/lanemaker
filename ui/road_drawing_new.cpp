#include "road_drawing_new.h"
#include "curve_fitting.h"
#include "CreateRoadOptionWidget.h"

extern std::weak_ptr<RoadRunner::Road> g_PointerRoad;
extern double g_PointerRoadS;
extern SectionProfileConfigWidget* g_createRoadOption;

RoadCreationSession_NEW::RoadCreationSession_NEW(QGraphicsView* aView):
	RoadDrawingSession(aView)
{
	stagedPreview = scene->addPath(stagedPreviewPath);
	flexPreview = scene->addPath(flexPreviewPath);
	QPen flexPen;
	flexPen.setStyle(Qt::DotLine);
	flexPreview->setPen(flexPen);
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
	odr::Vec2D  scenePos{ act.sceneX, act.sceneY };
	SnapCursor(scenePos);

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
	flexPreviewPath.clear();
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
		
		if (flexGeo != nullptr)
		{
			auto flexLen = flexGeo->length;
			for (int i = 0; i != 30; ++i)
			{
				auto s = flexLen / 29 * i;
				auto p = flexGeo->get_xy(s);
				if (i == 0)
					flexPreviewPath.moveTo(p[0], p[1]);
				else
					flexPreviewPath.lineTo(p[0], p[1]);
			}
		}
	}
	flexPreview->setPath(flexPreviewPath);
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
	SetHighlightTo(nullptr);
}