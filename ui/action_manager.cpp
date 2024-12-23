#include "action_manager.h"
#include "road_drawing.h"
#include "main_widget.h"
#include "mainwindow.h"
#include "CreateRoadOptionWidget.h"
#include "change_tracker.h"
#include "util.h"
#include "map_view_gl.h"

#include <fstream>
#include <filesystem>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <qscrollbar.h>


extern MainWindow* g_mainWindow;
extern SectionProfileConfigWidget* g_createRoadOption;

namespace RoadRunner
{
    MouseAction::MouseAction(QMouseEvent* evt)
    {
        screenX = evt->x();
        screenY = evt->y();
        type = evt->type();
        if (type == QEvent::MouseMove)
        {
            switch (evt->buttons())
            {
            case Qt::LeftButton:
                button = Qt::LeftButton;
                break;
            case Qt::RightButton:
                button = Qt::RightButton;
                break;
            case Qt::MiddleButton:
                button = Qt::MiddleButton;
                break;
            default:
                button = Qt::NoButton;
                break;
            }
        }
        else
        {
            button = evt->button();
        }
    }

    MouseAction::MouseAction(QWheelEvent* evt)
    {
        type = evt->type();
        screenX = evt->angleDelta().x();
        screenY = evt->angleDelta().y();
    }

    KeyPressAction::KeyPressAction(QKeyEvent* e)
    {
        key = e->key();
    }

    ActionManager* ActionManager::instance = nullptr;

    ActionManager* ActionManager::Instance()
    {
        if (instance == nullptr)
        {
            instance = new ActionManager;
        }
        return instance;
    }

    ActionManager::ActionManager() : 
        startTime(QTime::currentTime()) {}

    void ActionManager::Record(std::string loadedMap)
    {
        loadedXodr = loadedMap;
    }

    void ActionManager::Record(RoadRunner::EditMode modeChange)
    {
        ChangeModeAction serialized{ modeChange };
        history.emplace_back(serialized, startTime.msecsTo(QTime::currentTime()));
        Save();
        currEditMode = modeChange;
    }

    void ActionManager::Replay(const ChangeModeAction& action)
    {
        MainWidget::Instance()->SetModeFromReplay(action.mode);
    }

    void ActionManager::Record(Transform3D t)
    {
        auto q = t.rotation().toVector4D();
        ChangeViewportAction serialized
        {
            t.translation().x(), t.translation().y(), t.translation().z(),
            t.scale().x(), t.scale().y(), t.scale().z(),
            q.x(), q.y(), q.z(), q.w(),
            0, 0
        };
        latestViewportChange.emplace(serialized);
    }

    void ActionManager::Replay(const ChangeViewportAction& action)
    {
        Transform3D t;
        t.setTranslation(action.transX, action.transY, action.transZ);
        t.setScale(action.scaleX, action.scaleY, action.scaleZ);
        t.setRotation(QQuaternion(QVector4D(action.rotX, action.rotY, action.rotZ, action.rotW)));
        RoadRunner::g_mapViewGL->SetViewFromReplay(t);
    }

    void ActionManager::Record(MouseAction serialized)
    {
        if (serialized.type == QEvent::Type::MouseMove)
        {
            bool recordNow = false;
            if (serialized.button == Qt::LeftButton && currEditMode != EditMode::Mode_None)
            {
                recordNow = true;
            }
            else if (!lastRecordedMouseMove.has_value())
            {
                recordNow = true;
            }
            else
            {
                // TODO:
                /*
                auto lastXY = g_mapView->mapFromScene(
                    lastRecordedMouseMove.value().sceneX,
                    lastRecordedMouseMove.value().sceneY);

                int lastX = lastXY.x();
                int lastY = lastXY.y();
                int distanceSqr = std::pow(lastX - evt->pos().x(), 2) +
                    std::pow(lastY - evt->pos().y(), 2);
                if (distanceSqr > MouseMoveRecordThreshold * MouseMoveRecordThreshold)
                {
                    recordNow = true;
                }*/

            }

            if (recordNow)
            {
                FlushBufferedViewportChange();
                history.emplace_back(serialized, startTime.msecsTo(QTime::currentTime()));
                lastRecordedMouseMove.emplace(serialized);
                lastUnrecordedMouseMove.reset();
            }
            else
            {
                lastUnrecordedMouseMove.emplace(serialized);
            }
        }
        else
        {
            FlushBufferedViewportChange();
            FlushUnrecordedMouseMove();
            history.emplace_back(serialized, startTime.msecsTo(QTime::currentTime()));
            Save();
        }
    }

    void ActionManager::Replay(const MouseAction& action)
    {
        RoadRunner::g_mapViewGL->UpdateRayHit(QPoint(action.screenX, action.screenY));
        MainWidget::Instance()->OnMouseAction(action);
        RoadRunner::g_mapViewGL->renderLater();
    }

    void ActionManager::Record(KeyPressAction serialized)
    {
        FlushUnrecordedMouseMove();
        history.emplace_back(serialized, startTime.msecsTo(QTime::currentTime()));
        Save();
    }

    void ActionManager::Replay(const KeyPressAction& action)
    {
        MainWidget::Instance()->OnKeyPressed(action);
        RoadRunner::g_mapViewGL->renderLater();
    }

    void ActionManager::Record(const LanePlan& left, const LanePlan& right)
    {
        ChangeProfileAction serialized{ left, right };
        history.emplace_back(serialized, startTime.msecsTo(QTime::currentTime()));
        Save();
    }

    void ActionManager::Replay(const ChangeProfileAction& action)
    {
        g_createRoadOption->SetOption(action.leftProfile, action.rightProfile);
    }

    void ActionManager::Record(int8_t plan)
    {
        ChangeElevationAction serialized{ plan };
        history.emplace_back(serialized, startTime.msecsTo(QTime::currentTime()));
        Save();
    }

    void ActionManager::Replay(const ChangeElevationAction& action)
    {
        //g_mapView->parentContainer->SetElevationFromReplay(action.plan);
    }

    void ActionManager::Record(ActionType actionNoParm)
    {
        switch (actionNoParm)
        {
        case RoadRunner::Action_Undo: case RoadRunner::Action_Redo:
            history.emplace_back(actionNoParm, startTime.msecsTo(QTime::currentTime()));
            Save();
            break;
        default:
            spdlog::error("Invalid ActionType to Record: {}", static_cast<int>(actionNoParm));
        }
    }

    void ActionManager::Record(const QSize& oldSize, const QSize& newSize)
    {
        ResizeWindowAction serialized{
            oldSize.width(), oldSize.height(),
            newSize.width(), newSize.height()};

        history.emplace_back(serialized, startTime.msecsTo(QTime::currentTime()));
        Save();
    }

    void ActionManager::Replay(const ResizeWindowAction& act)
    {
        g_mainWindow->resize(act.width, act.height);
    }

    void ActionManager::Replay(std::string mapToLoad)
    {
        RoadRunner::ChangeTracker::Instance()->LoadStr(mapToLoad);
        loadedXodr = mapToLoad;
    }

    void ActionManager::Replay(const UserAction& action)
    {
        switch (action.type)
        {
        case Action_Mouse:
            history.emplace_back(action);
            Replay(action.detail.mouse);
            break;
        case Action_KeyPress:
            history.emplace_back(action);
            Replay(action.detail.keyPress);
            break;
        case Action_ChangeMode:
            Replay(action.detail.changeMode);
            break;
        case Action_Viewport:
            history.emplace_back(action);
            Replay(action.detail.viewport);
            break;
        case Action_ChangeProfile:
            Replay(action.detail.changeProfile);
            break;
        case Action_ChangeElevation:
            Replay(action.detail.changeElevation);
            break;
        case Action_Undo:
            g_mainWindow->undo();
            break;
        case Action_Redo:
            g_mainWindow->redo();
            break;
        case Action_ResizeWindow:
            Replay(action.detail.resizeWindow);
            break;
        default:
            spdlog::error("Action type {} replay is not supported", static_cast<int>(action.type));
            break;
        }
    }

    void ActionManager::Save() const
    {
        Save(AutosavePath());
    }

    void ActionManager::Save(std::string fpath) const
    {
        std::ofstream outFile(fpath, std::ios::binary);
        cereal::BinaryOutputArchive oarchive(outFile);
        oarchive(loadedXodr, history);
    }

    std::string ActionManager::AutosavePath() const
    {
        auto rtn = RoadRunner::DefaultSaveFolder();
        rtn /= std::string("action_rec__") + RoadRunner::RunTimestamp() + std::string(".dat");
        return rtn.string();
    }

    void ActionManager::Reset()
    {
        loadedXodr.clear();
        history.clear();
        lastRecordedMouseMove.reset();
        lastUnrecordedMouseMove.reset();
        latestViewportChange.reset();
        currEditMode = EditMode::Mode_None;
    }

    void ActionManager::FlushBufferedViewportChange()
    {
        if (!latestViewportChange.has_value()) return;

        history.emplace_back(latestViewportChange.value(), startTime.msecsTo(QTime::currentTime()));
        latestViewportChange.reset();
    }

    void ActionManager::FlushUnrecordedMouseMove()
    {
        if (lastUnrecordedMouseMove.has_value())
        {
            history.emplace_back(lastUnrecordedMouseMove.value(), startTime.msecsTo(QTime::currentTime()));
            lastRecordedMouseMove.emplace(lastUnrecordedMouseMove.value());
            lastUnrecordedMouseMove.reset();
        }
    }
}
