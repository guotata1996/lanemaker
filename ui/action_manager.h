#pragma once

#include <vector>
#include <optional>
#include <string>

#include <QTime>

#include "action_defs.h"

class RoadDrawingSession;

namespace RoadRunner
{
    class ActionManager
    {
    public:
        ActionManager(ActionManager&) = delete;
        ActionManager& operator=(const ActionManager&) = delete;

        static ActionManager* Instance();

        void Replay(std::string mapToLoad);
        void Replay(const UserAction&);

        void Record(std::string loadedMap);
        void Record(RoadRunner::EditMode);
        void Record(double zoomVal, double rotateVal, int hScroll, int vScroll);
        void Record(QMouseEvent*);
        void Record(QKeyEvent*);
        void Record(const LanePlan&, const LanePlan&);
        void Record(int8_t elevationPlan);
        void Record(ActionType);
        void Record(const QSize& oldSize, const QSize& newSize);

        void Save() const;
        void Save(std::string) const;

        std::string AutosavePath() const;

        void MarkException() { cleanAutoSave = false; }
        void Reset();

        bool CleanAutoSave() const { return cleanAutoSave; }

    private:
        ActionManager();

        static void Replay(const ChangeModeAction&);
        static void Replay(const ChangeViewportAction&);
        static void Replay(const MouseAction&);
        static void Replay(const KeyPressAction&);
        static void Replay(const ChangeProfileAction&);
        static void Replay(const ChangeElevationAction&);
        static void Replay(const ResizeWindowAction&);

        /* Save last viewport change (if exist) before key frame
         * Called before mouse events
         */
        void FlushBufferedViewportChange();

        /* Save last mouse move (if exist) before key frame
         * Called before mouse click, release and keyboard events
         */
        void FlushUnrecordedMouseMove();

        std::string loadedXodr;
        std::vector<UserAction> history;
        EditMode currEditMode;

        static ActionManager* instance;

        bool cleanAutoSave = true;

        /*Buffered action during record*/
        std::optional<RoadRunner::MouseAction> lastRecordedMouseMove;
        std::optional<RoadRunner::MouseAction> lastUnrecordedMouseMove;
        std::optional<RoadRunner::ChangeViewportAction> latestViewportChange;

        QTime startTime;

        const int MouseMoveRecordThreshold = 100;
    };
}