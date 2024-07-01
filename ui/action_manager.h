#include <vector>
#include <optional>

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
        Action_Viewport_Part1,
        Action_Viewport_Part2,
        Action_Viewport_Part3,
        Action_Viewport_Part4,
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

    struct ChangeViewportAction_Part1
    {
        double m11, m12;
    };

    struct ChangeViewportAction_Part2
    {
        double m13, m21;
    };

    struct ChangeViewportAction_Part3
    {
        double m22, m23;
    };

    struct ChangeViewportAction_Part4
    {
        double m33;
        int hScroll, vScroll;
    };
    
    struct ChangeViewportAction
    {
        ChangeViewportAction_Part1 part1;
        ChangeViewportAction_Part2 part2;
        ChangeViewportAction_Part3 part3;
        ChangeViewportAction_Part4 part4;
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
        ChangeViewportAction_Part1 viewport_part1;
        ChangeViewportAction_Part2 viewport_part2;
        ChangeViewportAction_Part3 viewport_part3;
        ChangeViewportAction_Part4 viewport_part4;
        ChangeModeAction changeMode;
        ChangeProfileAction changeProfile;
    };

    static_assert(sizeof(ActionDetail) == sizeof(ChangeViewportAction_Part1), "Need to update serialize function!");

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

        UserAction(ChangeViewportAction_Part1 c)
        {
            type = Action_Viewport_Part1;
            detail.viewport_part1 = c;
        }

        UserAction(ChangeViewportAction_Part2 c)
        {
            type = Action_Viewport_Part2;
            detail.viewport_part2 = c;
        }

        UserAction(ChangeViewportAction_Part3 c)
        {
            type = Action_Viewport_Part3;
            detail.viewport_part3 = c;
        }

        UserAction(ChangeViewportAction_Part4 c)
        {
            type = Action_Viewport_Part4;
            detail.viewport_part4 = c;
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
            archive(type, 
                detail.viewport_part1.m11, detail.viewport_part1.m12);
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
        void Replay(const MouseAction&);

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

        /* Save last viewport change (if exist) before key frame
         * Called before mouse events
         */
        void FlushBufferedViewportChange();

        /* Save last mouse move (if exist) before key frame
         * Called before mouse click, release and keyboard events
         */
        void FlushBufferedMouseMove();

        std::vector<UserAction> history;

        static ActionManager* instance;

        bool replayMode = false;

        /*Buffered action during record*/
        std::optional<RoadRunner::MouseAction> latestMouseMove;
        std::optional<RoadRunner::ChangeViewportAction> latestViewportChange;

        /*Buffered action during replay*/
        RoadRunner::ChangeViewportAction lastViewportReplay;
    };
}