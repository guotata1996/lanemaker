#pragma once

#include "qevent.h"
#include <QString>

#include "road_profile.h"

namespace LM
{
    enum EditMode
    {
        Mode_None,
        Mode_Create,
        Mode_CreateLanes,
        Mode_Destroy,
        Mode_Modify
    };

    enum ActionType
    {
        Action_Mouse,
        Action_KeyPress,
        Action_Viewport,
        Action_Undo,
        Action_Redo,
        Action_ChangeMode,
        Action_ChangeProfile,
        Action_ChangeElevation,
        Action_ResizeWindow
    };

    struct MouseAction
    {
        int screenX, screenY;
        QEvent::Type type;
        Qt::MouseButton button;
        // No need to save modifiers, as R-drag with modifiers has no effect
        // - will not be recorded as change VP event
        // - Even if session responds incorrectly, ChangeProfileAction will still be recorded at the end
        MouseAction() = default;

        MouseAction(QMouseEvent* e);
        MouseAction(QWheelEvent* e);
    };

    struct KeyPressAction
    {
        KeyPressAction() = default;
        KeyPressAction(QKeyEvent* e);
        KeyPressAction(int keyID);

        int key;

        QString ToString() const
        {
            QString keyStr;
            switch (key)
            {
            case Qt::Key_Control:
                keyStr = "Ctrl";
                break;
            case Qt::Key_Alt:
                keyStr = "Alt";
                break;
            case Qt::Key_Shift:
                keyStr = "Sft";
                break;
            default:
                keyStr = QKeySequence(key).toString();
                if (keyStr.size() > 3)
                    keyStr.chop(3);
                break;
            }
            return keyStr;
        }
    };

    struct ChangeViewportAction
    {
        float transX, transY, transZ;
        float scaleX, scaleY, scaleZ;
        float rotX, rotY, rotZ, rotW;
        int xSize, ySize;
    };

    struct ChangeModeAction
    {
        EditMode mode;
    };

    struct ChangeProfileAction
    {
        LanePlan leftProfile, rightProfile;
    };

    struct ChangeElevationAction
    {
        int plan;
    };

    struct ResizeWindowAction
    {
        int oldWidth, oldHeight;
        int width, height;
    };

    union ActionDetail
    {
        MouseAction mouse;
        KeyPressAction keyPress;
        ChangeViewportAction viewport;
        ChangeModeAction changeMode;
        ChangeProfileAction changeProfile;
        ChangeElevationAction changeElevation;
        ResizeWindowAction resizeWindow;

        ActionDetail() = default;
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

        UserAction(MouseAction ma, int aTime) :
            timeMS(aTime)
        {
            type = Action_Mouse;
            detail.mouse = ma;
        }

        UserAction(KeyPressAction k, int aTime) :
            timeMS(aTime)
        {
            type = Action_KeyPress;
            detail.keyPress = k;
        }

        UserAction(ChangeViewportAction a, int aTime) :
            timeMS(aTime)
        {
            type = Action_Viewport;
            detail.viewport = a;
        }

        UserAction(ChangeModeAction c, int aTime) :
            timeMS(aTime)
        {
            type = Action_ChangeMode;
            detail.changeMode = c;
        }

        UserAction(ChangeProfileAction c, int aTime) :
            timeMS(aTime)
        {
            type = Action_ChangeProfile;
            detail.changeProfile = c;
        }

        UserAction(ChangeElevationAction c, int aTime)
        {
            type = Action_ChangeElevation;
            detail.changeElevation = c;
        }

        UserAction(ResizeWindowAction c, int aTime)
        {
            type = Action_ResizeWindow;
            detail.resizeWindow = c;
        }

        template<class Archive>
        void serialize(Archive& archive)
        {
            archive(type, timeMS,
                detail.viewport.transX, detail.viewport.transY, detail.viewport.transZ,
                detail.viewport.scaleX, detail.viewport.scaleY, detail.viewport.scaleZ,
                detail.viewport.rotX, detail.viewport.rotY, detail.viewport.rotZ, detail.viewport.rotW);
        }
    };

    //enum ButtonSyntax
    //{
    //    Syntax_Confirm = 1,
    //    Syntax_Cancel = 2
    //};
}
