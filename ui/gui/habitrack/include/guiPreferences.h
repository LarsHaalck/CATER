#ifndef GUI_GUI_PREFERENCES
#define GUI_GUI_PREFERENCES

namespace gui
{
struct GuiPreferences
{
    int overlayUnaries = 0;
    bool overlayTrackedPos = true;
    bool overlayBearing = false;
    bool overlayTrajectory = false;
    int overlayTrajectoryWindow = 10;
    // enables more buttons in the processing group
    bool enableExpertView = false;
};
} // namespace gui

#endif // GUI_GUI_PREFERENCES
