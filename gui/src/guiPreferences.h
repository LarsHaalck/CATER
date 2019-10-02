#ifndef GUI_PREFERENCES
#define GUI_PREFERENCES

namespace gui
{
struct GuiPreferences
{
    int overlayUnaries = 50;
    int overlayTrackedPos = 50;
    int overlayTrajectory = 50;

    bool enableExpertView = true; // enables more buttons in the processing group
};
} // namespace gui

#endif // GUI_PREFERENCES
