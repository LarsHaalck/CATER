#ifndef PREFERENCESDIALOG_H
#define PREFERENCESDIALOG_H

#include "habitrack/preferences.h"
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
    explicit PreferencesDialog(QWidget* parent, const ht::Preferences& prefs = ht::Preferences());
    ~PreferencesDialog();
    ht::Preferences getPreferences() const;

private:
    void initPreferences(const ht::Preferences& pefs);

    void resetGeneralTo(const ht::Preferences& prefs);
    void resetFeaturesTo(const ht::Preferences& prefs);
    void resetUnariesTo(const ht::Preferences& prefs);
    void resetPairwiseTo(const ht::Preferences& prefs);
    void resetSmoothBearingTo(const ht::Preferences& prefs);
    void resetTransformationTo(const ht::Preferences& prefs);

    ht::FeatureType stringToFeatureType(const QString& string) const;

private slots:
    void on_resetButton_clicked();
    void on_resetAllButton_clicked();

    void on_enableSmoothBearing_toggled(bool value);
    void on_removeCamMotion_toggled(bool value);

private:
    Ui::PreferencesDialog* ui;
    ht::Preferences mPrefs;
    const ht::Preferences mPrefsDefaults;
};
} // namespace gui
#endif // PREFERENCESDIALOG_H
