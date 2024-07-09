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
        ActionDetail detail;

        UserAction() = default;

        UserAction(ActionType aType) : type(aType) {}

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

        UserAction(ChangeViewportAction a)
        {
            type = Action_Viewport;
            detail.viewport = a;
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

        void Record(MapView::EditMode);
        void Replay(const ChangeModeAction&);

        void Record(double zoomVal, double rotateVal, int hScroll, int vScroll);
        void Replay(const ChangeViewportAction&);

        void Record(QMouseEvent*);
        void Replay(const MouseAction&);

        void Record(QKeyEvent*);
        void Replay(const KeyPressAction&);

        void Record(const SectionProfile&, const SectionProfile&);
        void Replay(const ChangeProfileAction&);

        void Record(ActionType);

        void Save() const;
        void Save(std::string) const;

        std::string AutosavePath() const;

        void ReplayImmediate();
        void ReplayImmediate(std::string);

        void MarkException() { replayable = cleanAutoSave = false; }
        void Reset();

        bool Replaying() const { return replayMode; }

        bool Replayable() const { return replayable; }

        bool CleanAutoSave() const { return cleanAutoSave; }

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

        bool replayable = true;

        bool cleanAutoSave = true;

        /*Buffered action during record*/
        std::optional<RoadRunner::MouseAction> latestMouseMove;
        std::optional<RoadRunner::ChangeViewportAction> latestViewportChange;

        /*Buffered action during replay*/
        RoadRunner::ChangeViewportAction lastViewportReplay;
    };
}