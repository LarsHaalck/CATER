#ifndef GUI_GUI_PREFERENCES
#define GUI_GUI_PREFERENCES

namespace gui
{
struct GuiPreferences
{
    int overlayUnaries = 0;
    int overlayTrackedPos = 100;
    int overlayTrajectory = 0;

    bool enableExpertView = true; // enables more buttons in the processing group
};
} // namespace gui

#endif // GUI_GUI_PREFERENCES
