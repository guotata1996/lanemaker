#pragma once

#include <vector>
#include <optional>

#include "qevent.h"
#include <QTime>

#include "road_profile.h"
#include "map_view.h"

class RoadDrawingSession;
class MapView;

namespace RoadRunner
{
    enum ActionType
    {
        Action_Mouse,
        Action_KeyPress,
        Action_Viewport,
        Action_Undo,
        Action_Redo,
        Action_ChangeMode,
        Action_ChangeProfile,
        Action_LoadMap
    };

    struct MouseAction
    {
        int x, y;
        QEvent::Type type;
        Qt::MouseButton button;
    };

    struct KeyPressAction
    {
        int key;
    };
    
    struct ChangeViewportAction
    {
        double zoom, rotate;
        int hScroll, vScroll;
    };

    struct ChangeModeAction
    {
        MapView::EditMode mode;
    };

    struct ChangeProfileAction
    {
        SectionProfile leftProfile, rightProfile;
    };

    union ActionDetail
    {
        MouseAction mouse;
        KeyPressAction keyPress;
        ChangeViewportAction viewport;
        ChangeModeAction changeMode;
        ChangeProfileAction changeProfile;
    };

    static_assert(sizeof(ActionDetail) == sizeof(ChangeViewportAction), "Need to update serialize function!");

    struct UserAction
    {
        ActionType type;
        int timeMS;
        ActionDetail detail;

        UserAction() = default;

        UserAction(ActionType aType, int aTime) : 
            type(aType), timeMS(aTime) {}

        UserAction(MouseAction ma, int aTime):
            timeMS(aTime)
        {
            type = Action_Mouse;
            detail.mouse = ma;
        }

        UserAction(KeyPressAction k, int aTime):
            timeMS(aTime)
        {
            type = Action_KeyPress;
            detail.keyPress = k;
        }

        UserAction(ChangeViewportAction a, int aTime):
            timeMS(aTime)
        {
            type = Action_Viewport;
            detail.viewport = a;
        }

        UserAction(ChangeModeAction c, int aTime):
            timeMS(aTime)
        {
            type = Action_ChangeMode;
            detail.changeMode = c;
        }

        UserAction(ChangeProfileAction c, int aTime):
            timeMS(aTime)
        {
            type = Action_ChangeProfile;
            detail.changeProfile = c;
        }

        template<class Archive>
        void serialize(Archive& archive)
        {
            archive(type, timeMS,
                detail.viewport.zoom, detail.viewport.rotate,
                detail.viewport.hScroll, detail.viewport.vScroll);
        }
    };


    class ActionManager
    {
    public:
        ActionManager(ActionManager&) = delete;
        ActionManager& operator=(const ActionManager&) = delete;

        static ActionManager* Instance();

        void Replay(const UserAction&);

        void Record(MapView::EditMode);
        void Record(double zoomVal, double rotateVal, int hScroll, int vScroll);
        void Record(QMouseEvent*);
        void Record(QKeyEvent*);
        void Record(const SectionProfile&, const SectionProfile&);
        void Record(ActionType);

        void Save() const;
        void Save(std::string) const;

        std::string AutosavePath() const;

        void MarkException() { replayable = cleanAutoSave = false; }
        void Reset();

        bool Replayable() const { return replayable; }

        bool CleanAutoSave() const { return cleanAutoSave; }

        static std::vector<UserAction> Load(std::string);

    private:
        ActionManager();

        static void Replay(const ChangeModeAction&);
        static void Replay(const ChangeViewportAction&);
        static void Replay(const MouseAction&);
        static void Replay(const KeyPressAction&);
        static void Replay(const ChangeProfileAction&);

        /* Save last viewport change (if exist) before key frame
         * Called before mouse events
         */
        void FlushBufferedViewportChange();

        /* Save last mouse move (if exist) before key frame
         * Called before mouse click, release and keyboard events
         */
        void FlushUnrecordedMouseMove();

        std::vector<UserAction> history;

        static ActionManager* instance;

        bool replayable = true;

        bool cleanAutoSave = true;

        /*Buffered action during record*/
        std::optional<RoadRunner::MouseAction> lastRecordedMouseMove;
        std::optional<RoadRunner::MouseAction> lastUnrecordedMouseMove;
        std::optional<RoadRunner::ChangeViewportAction> latestViewportChange;

        /*Buffered action during replay*/
        RoadRunner::ChangeViewportAction lastViewportReplay;

        QTime startTime;

        const int MouseMoveRecordThreshold = 50;
    };
}