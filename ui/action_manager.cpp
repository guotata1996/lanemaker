#include "action_manager.h"
#include "road_drawing.h"
#include "main_widget.h"
#include "mainwindow.h"
#include "CreateRoadOptionWidget.h"
#include "change_tracker.h"
#include "util.h"

#include <fstream>
#include <filesystem>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <qscrollbar.h>


extern MapView* g_mapView;
extern MainWindow* g_mainWindow;
extern SectionProfileConfigWidget* g_createRoadOption;

namespace RoadRunner
{
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

    void ActionManager::Record(MapView::EditMode modeChange)
    {
        if (!replayable) return;
        ChangeModeAction serialized{ modeChange };
        history.emplace_back(serialized, startTime.msecsTo(QTime::currentTime()));
        Save();
    }

    void ActionManager::Replay(const ChangeModeAction& action)
    {
        g_mapView->parentContainer->SetModeFromReplay(action.mode);
    }

    void ActionManager::Record(double zoomVal, double rotateVal, int hScroll, int vScroll)
    {
        if (!replayable) return;
        ChangeViewportAction serialized
        {
            zoomVal, rotateVal,
            hScroll, vScroll
        };
        latestViewportChange.emplace(serialized);
    }

    void ActionManager::Replay(const ChangeViewportAction& action)
    {
        g_mapView->SetViewFromReplay(action.zoom, action.rotate, action.hScroll, action.vScroll);
    }

    void ActionManager::Record(QMouseEvent* evt)
    {
        if (!replayable) return;

        FlushBufferedViewportChange();
        MouseAction serialized;
        serialized.x = evt->pos().x();
        serialized.y = evt->pos().y();
        serialized.type = evt->type();
        serialized.button = evt->button();
        if (evt->type() == QEvent::Type::MouseMove)
        {
            bool recordNow = false;
            if (!lastRecordedMouseMove.has_value())
            {
                recordNow = true;
            }
            else
            {
                int lastX = lastRecordedMouseMove.value().x;
                int lastY = lastRecordedMouseMove.value().y;
                int distanceSqr = (lastX - serialized.x) * (lastX - serialized.x) +
                    (lastY - serialized.y) * (lastY - serialized.y);
                if (distanceSqr > MouseMoveRecordThreshold * MouseMoveRecordThreshold)
                {
                    recordNow = true;
                }
            }

            if (recordNow)
            {
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
            FlushUnrecordedMouseMove();
            history.emplace_back(serialized, startTime.msecsTo(QTime::currentTime()));
            Save();
        }

        if (evt->type() == QEvent::Type::MouseButtonPress)
        {
            auto scenePos = g_mapView->mapToScene(evt->pos());
            spdlog::trace("Record Click: {},{} ( {},{} )-> scene {},{}", 
                evt->pos().x(), evt->pos().y(),
                g_mapView->viewportTransform().dx(), g_mapView->viewportTransform().dy(),
                scenePos.x(), scenePos.y());
        }
    }

    void ActionManager::Replay(const MouseAction& action)
    {
        auto qMouseEvent = std::make_unique<QMouseEvent>(action.type,
            QPointF(action.x, action.y), action.button,
            QFlags<Qt::MouseButton>(), QFlags<Qt::KeyboardModifier>());
        switch (action.type)
        {
        case QEvent::Type::MouseButtonPress:
        {
            auto scenePos = g_mapView->mapToScene(QPoint(action.x, action.y));
            spdlog::trace("Click: {},{} ( {},{} )-> scene {},{}",
                action.x, action.y, 
                g_mapView->viewportTransform().dx(), g_mapView->viewportTransform().dy(),
                scenePos.x(), scenePos.y());
            g_mapView->OnMousePress(qMouseEvent.get());
            break;
        }
        case QEvent::Type::MouseButtonDblClick:
            g_mapView->OnMouseDoubleClick(qMouseEvent.get());
            break;
        case QEvent::Type::MouseMove:
            g_mapView->OnMouseMove(qMouseEvent.get());
            break;
        case QEvent::Type::MouseButtonRelease:
            g_mapView->OnMouseRelease(qMouseEvent.get());
            break;
        default:
            spdlog::error("Unsupported MouseAction to replay: {}", static_cast<int>(action.type));
            break;
        }
    }

    void ActionManager::Record(QKeyEvent* evt)
    {
        if (!replayable) return;
        FlushUnrecordedMouseMove();
        KeyPressAction serialized{ evt->key() };
        history.emplace_back(serialized, startTime.msecsTo(QTime::currentTime()));
        Save();
    }

    void ActionManager::Replay(const KeyPressAction& action)
    {
        auto qKeyEvent = std::make_unique<QKeyEvent>(QEvent::Type::KeyPress,
            action.key, QFlags<Qt::KeyboardModifier>());
        g_mapView->OnKeyPress(qKeyEvent.get());
    }

    void ActionManager::Record(const SectionProfile& left, const SectionProfile& right)
    {
        if (!replayable) return;
        ChangeProfileAction serialized{ left, right };
        history.emplace_back(serialized, startTime.msecsTo(QTime::currentTime()));
        Save();
    }

    void ActionManager::Replay(const ChangeProfileAction& action)
    {
        g_createRoadOption->SetOption(action.leftProfile, action.rightProfile);
    }

    void ActionManager::Record(ActionType actionNoParm)
    {
        switch (actionNoParm)
        {
        case RoadRunner::Action_Undo: case RoadRunner::Action_Redo:
            history.emplace_back(actionNoParm, startTime.msecsTo(QTime::currentTime()));
            Save();
            break;
        case RoadRunner::Action_LoadMap:
            // To be supported
            replayable = false;
            break;
        default:
            spdlog::error("Invalid ActionType to Record: {}", static_cast<int>(actionNoParm));
        }
    }

    void ActionManager::Record(const QSize& oldSize, const QSize& newSize)
    {
        if (!replayable) return;
        ResizeWindowAction serialized{
            oldSize.width(), oldSize.height(),
            newSize.width(), newSize.height()};

        history.emplace_back(serialized, startTime.msecsTo(QTime::currentTime()));
        Save();
    }

    void ActionManager::Replay(const ResizeWindowAction& act)
    {
        g_mainWindow->resizeDontRecord(act.width, act.height);
    }

    void ActionManager::Replay(const UserAction& action)
    {
        history.emplace_back(action);
        switch (action.type)
        {
        case Action_Mouse:
            if (lastViewportReplay.has_value())
                Replay(lastViewportReplay.value()); // Weird: mapView->transform() changes silently
            Replay(action.detail.mouse);
            break;
        case Action_KeyPress:
            Replay(action.detail.keyPress);
            break;
        case Action_ChangeMode:
            Replay(action.detail.changeMode);
            break;
        case Action_Viewport:
            lastViewportReplay.emplace(action.detail.viewport);
            break;
        case Action_ChangeProfile:
            Replay(action.detail.changeProfile);
            break;
        case Action_Undo:
            if (!ChangeTracker::Instance()->Undo())
            {
                spdlog::error("Error replaying undo action");
            }
            break;
        case Action_Redo:
            if (!ChangeTracker::Instance()->Redo())
            {
                spdlog::error("Error replaying redo action");
            }
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
        oarchive(history);
    }

    std::string ActionManager::AutosavePath() const
    {
        auto rtn = RoadRunner::DefaultSaveFolder();
        rtn /= std::string("action_rec__") + RoadRunner::RunTimestamp() + std::string(".dat");
        return rtn.string();
    }

    void ActionManager::Reset()
    {
        history.clear();
        lastViewportReplay.reset();
        lastRecordedMouseMove.reset();
        lastUnrecordedMouseMove.reset();
        latestViewportChange.reset();
        replayable = true;
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
