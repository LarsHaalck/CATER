#ifndef PREFERENCESDIALOG_H
#define PREFERENCESDIALOG_H

#include <cater/model/preferences.h>
#include <cater/image-processing/baseFeatureContainer.h>
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
    explicit PreferencesDialog(QWidget* parent, const ct::Preferences& prefs = ct::Preferences());
    ~PreferencesDialog();
    ct::Preferences getPreferences() const;

private:
    void initPreferences(const ct::Preferences& pefs);

    void resetGeneralTo(const ct::Preferences& prefs);
    void resetFeaturesTo(const ct::Preferences& prefs);
    void resetUnariesTo(const ct::Preferences& prefs);
    void resetPairwiseTo(const ct::Preferences& prefs);
    void resetSmoothBearingTo(const ct::Preferences& prefs);
    void resetTransformationTo(const ct::Preferences& prefs);

    ct::FeatureType stringToFeatureType(const QString& string) const;

private slots:
    void on_resetButton_clicked();
    void on_resetAllButton_clicked();

    void on_enableSmoothBearing_toggled(bool value);
    void on_removeCamMotion_toggled(bool value);

private:
    Ui::PreferencesDialog* ui;
    ct::Preferences mPrefs;
    const ct::Preferences mPrefsDefaults;
};
} // namespace gui
#endif // PREFERENCESDIALOG_H
