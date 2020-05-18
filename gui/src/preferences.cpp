#include "gui/preferences.h"

using namespace gui;
std::ostream& operator<< (std::ostream& stream, const Preferences& prefs)
{
    stream << "Preferences: \n";
    stream << "Colour Correction: \n";
    stream << "Enabled: " << prefs.colourCorrection << "\n";
    stream << "Red: " << prefs.colourRed << "\n";
    stream << "Green: " << prefs.colourGreen << "\n";
    stream << "Blue: " << prefs.colourBlue << "\n";
    stream << "--------------------\n";
    stream << "Unary Settings: \n";
    stream << "Subsample: " << prefs.unarySubsample << "\n";
    stream << "Sigma: " << prefs.unarySigma << "\n";
    stream << "Remove Red Lasers: " << prefs.removeRedLasers << "\n";
    stream << "Suppress: " << prefs.unarySuppress << "\n";
    stream << "Manual Multiplier: " << prefs.unaryMultiplier << "\n";
    stream << "--------------------\n";
    stream << "Pairwise Settings: \n";
    stream << "Size: " << prefs.pairwiseSize << "\n";
    stream << "Sigma: " << prefs.pairwiseSigma << "\n";
    stream << "--------------------\n";
    stream << "Smooth Bearing Settings: \n";
    stream << "Enable Smooth Bearing: " << prefs.smoothBearing << "\n";
    stream << "Window Size: " << prefs.smoothBearingWindowSize << "\n";
    stream << "Outlier Tol: " << prefs.smoothBearingOutlierTol << "\n";
    stream << "--------------------\n";
    stream << "Transformation Settings: \n";
    stream << "Remove Motion: " << prefs.removeCamMotion << "\n";
    stream << "NN Ratio: " << prefs.nnRatio << "\n";
    stream << "Ransac Reproj: " << prefs.ranscacReproj << "\n";
    stream << "Num Features: " << prefs.numFeatures << "\n";
    stream << "====================\n";
    return stream;
}

