#ifndef PREFERENCESDIALOG_H
#define PREFERENCESDIALOG_H

#include <QDialog>
#include "gui/preferences.h"

namespace Ui {
class PreferencesDialog;
}

namespace gui
{
class PreferencesDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PreferencesDialog(QWidget *parent, const Preferences& prefs = Preferences());
    ~PreferencesDialog();
    Preferences getPreferences() const;

private:
    void initPreferences(const Preferences& pefs);
    void resetColourTo(const Preferences& prefs);
    void resetUnariesTo(const Preferences& prefs);
    void resetPairwiseTo(const Preferences& prefs);
    void resetPanoramaTo(const Preferences& prefs);
    void resetSmoothBearingTo(const Preferences& prefs);
    void resetTransformationTo(const Preferences& prefs);

private slots:
    void onResetClicked();
    void onResetAllClicked();

    void onEnableColourCorrection(bool value);
    void onEnableSmoothBearing(bool value);


private:
    Ui::PreferencesDialog *ui;
    Preferences mPrefs;
    const Preferences mPrefsDefaults;
};
} // namespace gui
#endif // PREFERENCESDIALOG_H
