#ifndef PREFERENCESDIALOG_H
#define PREFERENCESDIALOG_H

#include "model/preferences.h"
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
    explicit PreferencesDialog(QWidget* parent, const model::Preferences& prefs = model::Preferences());
    ~PreferencesDialog();
    model::Preferences getPreferences() const;

private:
    void initPreferences(const model::Preferences& pefs);

    void resetGeneralTo(const model::Preferences& prefs);
    void resetColourTo(const model::Preferences& prefs);
    void resetFeaturesTo(const model::Preferences& prefs);
    void resetUnariesTo(const model::Preferences& prefs);
    void resetPairwiseTo(const model::Preferences& prefs);
    void resetPanoramaTo(const model::Preferences& prefs);
    void resetSmoothBearingTo(const model::Preferences& prefs);
    void resetTransformationTo(const model::Preferences& prefs);

    ht::FeatureType stringToFeatureType(const QString& string) const;

private slots:
    void on_resetButton_clicked();
    void on_resetAllButton_clicked();

    void on_enableColourCorrection_toggled(bool value);
    void on_enableSmoothBearing_toggled(bool value);
    void on_removeCamMotion_toggled(bool value);

private:
    Ui::PreferencesDialog* ui;
    model::Preferences mPrefs;
    const model::Preferences mPrefsDefaults;
};
} // namespace gui
#endif // PREFERENCESDIALOG_H
