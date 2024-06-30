#include <vector>

#include "qevent.h"

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
        Action_ChangeProfile
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
        // TODO: make multiple structs
        double m11, m12, m13, m21, m22, m23, m31, m32, m33;
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

    struct UserAction
    {
        ActionType type;
        ActionDetail detail;

        UserAction() = default;

        UserAction(MouseAction ma)
        {
            type = Action_Mouse;
            detail.mouse = ma;
        }

        UserAction(KeyPressAction k)
        {
            type = Action_KeyPress;
            detail.keyPress = k;
        }

        UserAction(ChangeViewportAction c)
        {
            type = Action_Viewport;
            detail.viewport = c;
        }

        UserAction(ChangeModeAction c)
        {
            type = Action_ChangeMode;
            detail.changeMode = c;
        }

        UserAction(ChangeProfileAction c)
        {
            type = Action_ChangeProfile;
            detail.changeProfile = c;
        }

        template<class Archive>
        void serialize(Archive& archive)
        {
            //archive(type, detail.mouse.x, detail.mouse.y, 
            //    detail.mouse.type, detail.mouse.button);
            archive(type, 
                detail.viewport.m11, detail.viewport.m12, detail.viewport.m13,
                detail.viewport.m21, detail.viewport.m22, detail.viewport.m23,
                detail.viewport.m31, detail.viewport.m32, detail.viewport.m33,
                detail.viewport.hScroll, detail.viewport.vScroll);
        }
    };


    class ActionManager
    {
    public:
        ActionManager(ActionManager&) = delete;
        ActionManager& operator=(const ActionManager&) = delete;

        static ActionManager* Instance();

        void Record(MapView::EditMode);
        void Replay(const ChangeModeAction&);

        void Record(const QTransform&, int hScroll, int vScroll);
        void Replay(const ChangeViewportAction&);

        void Record(QMouseEvent*);
        void Replay(const MouseAction&);    // Loaded from file

        void Record(QKeyEvent*);
        void Replay(const KeyPressAction&);

        void Record(const SectionProfile&, const SectionProfile&);
        void Replay(const ChangeProfileAction&);

        void Save(std::string);

        void ReplayImmediate(std::string);

        void ReplayAnimated(std::string);

        void Reset();

    private:
        ActionManager() = default;

        std::vector<UserAction> history;

        static ActionManager* instance;

        bool replayMode = false;

        RoadRunner::ChangeViewportAction lastViewportRecord;
    };
}