#include "action_manager.h"
#include "road_drawing.h"
#include "CreateRoadOptionWidget.h"

#include <fstream>
#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <qscrollbar.h>


extern MapView* g_mapView;
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

    void ActionManager::Record(MapView::EditMode modeChange)
    {
        if (replayMode) return;
        ChangeModeAction serialized{ modeChange };
        history.emplace_back(serialized);
    }

    void ActionManager::Replay(const ChangeModeAction& action)
    {
        g_mapView->SetEditMode(action.mode);
    }

    void ActionManager::Record(const QTransform& newTrans, int hScroll, int vScroll)
    {
        if (replayMode) return;
        ChangeViewportAction serialized
        {
            newTrans.m11(), newTrans.m12(), newTrans.m13(),
            newTrans.m21(), newTrans.m22(), newTrans.m23(),
            newTrans.m33(),
            hScroll, vScroll
        };
        latestViewportChange.emplace(serialized);
    }

    void ActionManager::Replay(const ChangeViewportAction& action)
    {
        QTransform tr;
        tr.setMatrix(action.part1.m11, action.part1.m12, action.part2.m13,
            action.part2.m21, action.part3.m22, action.part3.m23,
            0, 0, action.part4.m33);

        g_mapView->setTransform(tr); // ROADRUNNERTODO: change sliders accordingly
        g_mapView->horizontalScrollBar()->setValue(action.part4.hScroll);
        g_mapView->verticalScrollBar()->setValue(action.part4.vScroll);
    }

    void ActionManager::Record(QMouseEvent* evt)
    {
        if (replayMode) return;

        FlushBufferedViewportChange();
        MouseAction serialized;
        serialized.x = evt->pos().x();
        serialized.y = evt->pos().y();
        serialized.type = evt->type();
        serialized.button = evt->button();
        if (evt->type() == QEvent::Type::MouseMove)
        {
            latestMouseMove.emplace(serialized);
        }
        else
        {
            FlushBufferedMouseMove();
            history.emplace_back(serialized);
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
            g_mapView->mousePressEvent(qMouseEvent.get());
            break;
        }
        case QEvent::Type::MouseButtonDblClick:
            g_mapView->mouseDoubleClickEvent(qMouseEvent.get());
            break;
        case QEvent::Type::MouseMove:
            g_mapView->mouseMoveEvent(qMouseEvent.get());
            break;
        case QEvent::Type::MouseButtonRelease:
            g_mapView->mouseReleaseEvent(qMouseEvent.get());
            break;
        default:
            throw;
        }
    }

    void ActionManager::Record(QKeyEvent* evt)
    {
        if (replayMode) return;
        FlushBufferedMouseMove();
        KeyPressAction serialized{ evt->key() };
        history.emplace_back(serialized);
    }

    void ActionManager::Replay(const KeyPressAction& action)
    {
        auto qKeyEvent = std::make_unique<QKeyEvent>(QEvent::Type::KeyPress,
            action.key, QFlags<Qt::KeyboardModifier>());
        g_mapView->keyPressEvent(qKeyEvent.get());
    }

    void ActionManager::Record(const SectionProfile& left, const SectionProfile& right)
    {
        if (replayMode) return;
        ChangeProfileAction serialized{ left, right };
        history.emplace_back(serialized);
    }

    void ActionManager::Replay(const ChangeProfileAction& action)
    {
        g_createRoadOption->SetOption(action.leftProfile, action.rightProfile);
    }

    void ActionManager::Save(std::string fpath)
    {
        std::ofstream outFile(fpath, std::ios::binary);
        cereal::BinaryOutputArchive oarchive(outFile);
        oarchive(history);
    }

    void ActionManager::ReplayImmediate(std::string fpath)
    {
        std::ifstream inFile(fpath, std::ios::binary);
        cereal::BinaryInputArchive iarchive(inFile);
        iarchive(history);
        replayMode = true;
        for (const auto& action : history)
        {
            switch (action.type)
            {
            case Action_Mouse:
                Replay(lastViewportReplay); // Weird: mapView->transform() changes silently
                Replay(action.detail.mouse);
                break;
            case Action_KeyPress:
                Replay(action.detail.keyPress);
                break;
            case Action_ChangeMode:
                Replay(action.detail.changeMode);
                break;
            case Action_Viewport_Part1:
                lastViewportReplay.part1 = action.detail.viewport_part1;
                break;
            case Action_Viewport_Part2:
                lastViewportReplay.part2 = action.detail.viewport_part2;
                break;
            case Action_Viewport_Part3:
                lastViewportReplay.part3 = action.detail.viewport_part3;
                break;
            case Action_Viewport_Part4:
                lastViewportReplay.part4 = action.detail.viewport_part4;
                break;
            case Action_ChangeProfile:
                Replay(action.detail.changeProfile);
                break;
            default:
                spdlog::error("Action type {} replay is not supported", static_cast<int>(action.type));
                break;
            }
        }
        replayMode = false;
    }

    void ActionManager::Reset()
    {
        history.clear();
    }

    void ActionManager::FlushBufferedViewportChange()
    {
        if (!latestViewportChange.has_value()) return;

        history.emplace_back(latestViewportChange.value().part1);
        history.emplace_back(latestViewportChange.value().part2);
        history.emplace_back(latestViewportChange.value().part3);
        history.emplace_back(latestViewportChange.value().part4);
        latestViewportChange.reset();
    }

    void ActionManager::FlushBufferedMouseMove()
    {
        if (!latestMouseMove.has_value()) return;

        history.emplace_back(latestMouseMove.value());
        latestMouseMove.reset();
    }
}