#include "road_drawing.h"
#include "Geometries/RoadGeometry.h"

#include <QPainterPath>
#include <vector>
#include <optional>

namespace 
{
	class DirectionHandle : public QGraphicsPixmapItem
	{
	public:
		DirectionHandle();

	protected:
		virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, 
			QWidget* widget = nullptr) override;
	};
}

class RoadCreationSession_NEW: public RoadDrawingSession
{
public:
	RoadCreationSession_NEW(QGraphicsView* aView);

	virtual bool Update(const RoadRunner::MouseAction&);

	virtual bool Complete() override;

	virtual ~RoadCreationSession_NEW();

protected:
	struct StagedGeometry
	{
		std::unique_ptr<odr::RoadGeometry> geo;
		QPainterPath preview;
	};
	std::vector<StagedGeometry> stagedGeometries;
	
	std::optional<odr::Vec2D> startPos; // Can be on blank or extend from
	std::optional<odr::Vec2D> startDir; // Valid if extend from existing

private:
	bool SnapCursor(odr::Vec2D&) const;

	std::unique_ptr<odr::RoadGeometry> flexGeo;
	QPainterPath flexPreviewPath;
	QPainterPath stagedPreviewPath;

	QGraphicsPathItem* stagedPreview;
	QGraphicsPathItem* flexPreview;
	DirectionHandle* directionHandle;
};