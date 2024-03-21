#include "world.h"
#include "qgraphicsview.h"
#include <vector>

#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
//#include "Geometries/Arc.h"
//#include "Geometries/Spiral.h"

class RoadDrawingSession
{
public:
    RoadDrawingSession(QGraphicsView * aView, QPointF initialP) : 
        view(aView), scene(aView->scene()), world(World::Instance()) 
    {
        previewItem = scene->addPath(ctrlPath);
        hintItem = scene->addPath(hintPath);
        QPen hintPen;
        hintPen.setColor(QColor(0, 200, 100, 80));
        hintPen.setStyle(Qt::DotLine);
        hintItem->setPen(hintPen);
        cursorItem = scene->addEllipse(-2, -2, 4, 4);
        ctrlPoints.push_back(initialP);
    }

    void Update(QMouseEvent* event)
    {
        QPointF scenePos = view->mapToScene(event->pos().x(), event->pos().y());

        if (event->button() == Qt::MouseButton::LeftButton)
        {
            ctrlPoints.push_back(scenePos);
        }
        else if (event->button() == Qt::MouseButton::RightButton)
        {
            if (ctrlPoints.size() > 1)
            {
                // At least keep one
                ctrlPoints.pop_back();
            }
        }
        else if (event->button() == Qt::MouseButton::NoButton)
        {
            ctrlPoints.back().setX(scenePos.x());
            ctrlPoints.back().setY(scenePos.y());
        }

        hintPath.clear();
        float maxSnapDist = ctrlPoints.size() % 2 == 1 ? 5 : 1e9;
        ProjectToPathDir(maxSnapDist);
        cursorItem->setPos(ctrlPoints.back());

        // TODO: incremental path?
        ctrlPath.clear();
        if (!ctrlPoints.empty())
        {
            ctrlPath.moveTo(ctrlPoints[0]);
        }
        int i = 1;
        for (; i + 2 < ctrlPoints.size(); i += 2)
        {
            ctrlPath.quadTo(ctrlPoints[i], ctrlPoints[i + 1]);
        }
        if (i == ctrlPoints.size() - 1)
        {
            ctrlPath.lineTo(ctrlPoints[i]);
        }
        else if (i == ctrlPoints.size() - 2)
        {
            ctrlPath.quadTo(ctrlPoints[i], ctrlPoints[i + 1]);
        }

        previewItem->setPath(ctrlPath);
        hintItem->setPath(hintPath);
    }

    void Complete()
    {
        if (ctrlPoints.size() >= 3)
        {
            CreateRoad();
        }
    }

    ~RoadDrawingSession()
    {
        scene->removeItem(previewItem);
        scene->removeItem(hintItem);
        scene->removeItem(cursorItem);
    }

    void ProjectToPathDir(float maxOffset)
    {
        if (ctrlPoints.size() < 3)
        {
            return;
        }
        
        QPointF& nextPoint = ctrlPoints.back();
        auto lastFixed = ctrlPoints[ctrlPoints.size() - 2];
        QPointF nextDirPoint = lastFixed - ctrlPoints[ctrlPoints.size() - 3];
        auto nextDir = QVector2D(nextDirPoint);
        if (nextDir.lengthSquared() > 0.01f)
        {
            nextDir.normalize();
            hintPath.moveTo(lastFixed);
            hintPath.lineTo(lastFixed + nextDirPoint * 100);
        }

        QPointF lastPoint = ctrlPoints[ctrlPoints.size() - 2];
        QVector2D last2Next(nextPoint - lastPoint);
        auto projLength = QVector2D::dotProduct(last2Next, nextDir);
        QVector2D projected = QVector2D(lastPoint) + projLength * nextDir;
        if (projected.distanceToPoint(QVector2D(nextPoint)) < maxOffset)
        {
            nextPoint.setX(projected.x());
            nextPoint.setY(projected.y());
        }
    }

    void CreateRoad()
    {
        odr::RefLine refLine("", 0);
        double cumLength = 0;
        for (int i = 0; i + 2 < ctrlPoints.size() - 1; i += 2)
        {
            std::unique_ptr<odr::RoadGeometry> localGeometry;

            QVector2D start(ctrlPoints[i]);
            QVector2D middle(ctrlPoints[i + 1]);
            QVector2D end(ctrlPoints[i + 2]);
            odr::Vec2D point1{ start.x(), start.y() };
            odr::Vec2D point2{ middle.x(), middle.y() };
            odr::Vec2D point3{ end.x(), end.y() };

            if (middle.distanceToLine(start, end - start) < 0.1f)
            {
                localGeometry = std::make_unique<odr::Line>(cumLength, point1, point3);
            }
            else
            {
                localGeometry = std::make_unique<odr::ParamPoly3>(cumLength, point1, point2, point2, point3);
            }
            double cumLengthPrev = cumLength;
            cumLength += localGeometry->length;
            refLine.s0_to_geometry.emplace(cumLengthPrev, std::move(localGeometry));
        }
        refLine.length = cumLength;

        RoadRunner::RoadProfile config(1, 0, 1, 0);
        world->allRoads.insert(std::make_shared<RoadRunner::Road>(config, refLine));
    }

private:
    QGraphicsView* view;
    QGraphicsScene* scene;

    std::vector<QPointF> ctrlPoints;
    
    QPainterPath ctrlPath;
    QGraphicsPathItem* previewItem;

    QPainterPath hintPath;
    QGraphicsPathItem* hintItem;

    QGraphicsEllipseItem* cursorItem;

    World* world;
    
};