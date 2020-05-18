#ifndef GUI_PREFERENCES_H
#define GUI_PREFERENCES_H

#include <ostream>

namespace gui
{
struct Preferences
{
    // colour correction
    bool colourCorrection = false;
    int colourRed = 33;
    int colourGreen = 33;
    int colourBlue = 33;

    // unaries
    double unarySubsample = 0.8;
    double unarySigma = 200.0;
    bool removeRedLasers = true;
    double unarySuppress = 1.0;
    double unaryMultiplier = 4.0;

    // pairwise
    int pairwiseSize = 15;
    double pairwiseSigma = 6.0;

    // smooth bearing
    bool smoothBearing = true;
    int smoothBearingWindowSize = 5;
    int smoothBearingOutlierTol = 3;

    // transformation
    bool removeCamMotion = true;
    double nnRatio = 0.8;
    double ranscacReproj = 3.0;
    int numFeatures = 8000;

};
} // namespace gui

std::ostream& operator<< (std::ostream& stream, const gui::Preferences& prefs);

#endif // GUI_PREFERENCES_H
