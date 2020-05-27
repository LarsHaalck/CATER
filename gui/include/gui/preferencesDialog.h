#ifndef PREFERENCESDIALOG_H
#define PREFERENCESDIALOG_H

#include "gui/preferences.h"
#include "image-processing/baseFeatureContainer.h"
#include <QDialog>

namespace Ui
{
class PreferencesDialog;
}

namespace gui
{
class PreferencesDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PreferencesDialog(QWidget* parent, const Preferences& prefs = Preferences());
    ~PreferencesDialog();
    Preferences getPreferences() const;

private:
    void initPreferences(const Preferences& pefs);

    void resetGeneralTo(const Preferences& prefs);
    void resetColourTo(const Preferences& prefs);
    void resetFeaturesTo(const Preferences& prefs);
    void resetUnariesTo(const Preferences& prefs);
    void resetPairwiseTo(const Preferences& prefs);
    void resetPanoramaTo(const Preferences& prefs);
    void resetSmoothBearingTo(const Preferences& prefs);
    void resetTransformationTo(const Preferences& prefs);

    ht::FeatureType stringToFeatureType(const QString& string) const;

private slots:
    void on_resetButton_clicked();
    void on_resetAllButton_clicked();

    void on_enableColourCorrection_toggled(bool value);
    void on_enableSmoothBearing_toggled(bool value);
    void on_removeCamMotion_toggled(bool value);

private:
    Ui::PreferencesDialog* ui;
    Preferences mPrefs;
    const Preferences mPrefsDefaults;
};
} // namespace gui
#endif // PREFERENCESDIALOG_H
