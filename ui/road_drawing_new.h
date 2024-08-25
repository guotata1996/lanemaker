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

		bool Update(const RoadRunner::MouseAction& act);

	protected:
		virtual bool contains(const QPointF& point) const override;

		virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, 
			QWidget* widget = nullptr) override;

	private:
		double dragging = false;
		double deltaRotation;
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

	enum SnapResult
	{
		Snap_Nothing,
		Snap_Line,
		Snap_Point
	};
	
	std::optional<odr::Vec2D> startPos; // Can be on blank or extend from

	// Record extend / join
	virtual SnapResult SnapFirstPointToExisting(odr::Vec2D&);
	virtual SnapResult SnapLastPointToExisting(odr::Vec2D&);

	std::weak_ptr<RoadRunner::Road> extendFromStart;
	double extendFromStartS;
	std::weak_ptr<RoadRunner::Road> joinAtEnd;
	double joinAtEndS;

	// Record overlap (for junction / bridge / tunnel) // TODO:
	std::weak_ptr<RoadRunner::Road> overlapAtStart;
	double overlapAtStartS;
	std::weak_ptr<RoadRunner::Road> overlapAtEnd;
	double overlapAtEndS;


private:
	SnapResult SnapCursor(odr::Vec2D&);

	static void GeneratePainterPath(const std::unique_ptr<odr::RoadGeometry>&, 
		QPainterPath&);

	odr::Vec2D ExtendFromDir() const;
	odr::Vec2D JoinAtEndDir() const;

	std::unique_ptr<odr::RoadGeometry> flexGeo;
	QPainterPath flexPreviewPath;
	QPainterPath stagedPreviewPath;

	QGraphicsPathItem* stagedPreview;
	QGraphicsPathItem* flexPreview;
	DirectionHandle* directionHandle;
};