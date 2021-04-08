#include "gui/preferencesDialog.h"
#include "ui_preferencesDialog.h"
#include <QMouseEvent>
#include <iostream>

using namespace ht;

namespace gui
{
PreferencesDialog::PreferencesDialog(QWidget* parent, const Preferences& prefs)
    : QDialog(parent)
    , ui(new Ui::PreferencesDialog)
    , mPrefsDefaults()

{
    ui->setupUi(this);
    initPreferences(prefs);
}

PreferencesDialog::~PreferencesDialog() { delete ui; }

void PreferencesDialog::resetGeneralTo(const Preferences& prefs)
{
    ui->cacheSpin->setValue(prefs.cacheSize);
    ui->chunkSpin->setValue(prefs.chunkSize);
    ui->radiusSpin->setValue(prefs.detectionRadius);
    ui->fpsSpin->setValue(prefs.fps);
    ui->pxMmSpin->setValue(prefs.pixelsPerMm);
}

void PreferencesDialog::resetFeaturesTo(const Preferences& prefs)
{
    if (!ui->featureCombo->count())
    {
        ui->featureCombo->addItem("ORB");
        ui->featureCombo->addItem("SIFT");
    }
    ui->featureCombo->setCurrentIndex(static_cast<int>(prefs.featureType));
    ui->numFeaturesSpin->setValue(prefs.numFeatures);
}

void PreferencesDialog::resetUnariesTo(const Preferences& prefs)
{
    ui->subsampleSpin->setValue(prefs.unarySubsample);
    ui->unarySigmaSpin->setValue(prefs.unarySigma);
    ui->removeRedLasersCheck->setChecked(prefs.removeRedLasers);
    ui->suppressSpin->setValue(prefs.unarySuppress);
    ui->manualMultiplierSpin->setValue(prefs.unaryMultiplier);
    ui->manualUnarySizeSpin->setValue(prefs.manualUnarySize);
}

void PreferencesDialog::resetPairwiseTo(const Preferences& prefs)
{
    ui->pairwiseSigmaSpin->setValue(prefs.pairwiseSigma);
    ui->pairwiseSizeSpin->setValue(prefs.pairwiseSize);
}

void PreferencesDialog::resetSmoothBearingTo(const Preferences& prefs)
{
    ui->enableSmoothBearing->setChecked(prefs.smoothBearing);
    ui->smoothBearingWindowSpin->setValue(prefs.smoothBearingWindowSize);
    ui->smoothBearingOutlierTolSpin->setValue(prefs.smoothBearingOutlierTol);
}

void PreferencesDialog::resetTransformationTo(const Preferences& prefs)
{
    ui->removeCamMotion->setChecked(prefs.removeCamMotion);
    ui->nnRatioSpin->setValue(prefs.nnRatio);
    ui->maxReprojSpin->setValue(prefs.ranscacReproj);
}

ht::FeatureType PreferencesDialog::stringToFeatureType(const QString& string) const
{
    if (string == "ORB")
        return ht::FeatureType::ORB;
    if (string == "SIFT")
        return ht::FeatureType::SIFT;

    // cannot happen
    return ht::FeatureType::SIFT;
}

void PreferencesDialog::initPreferences(const Preferences& prefs)
{
    resetGeneralTo(prefs);
    resetFeaturesTo(prefs);
    resetUnariesTo(prefs);
    resetPairwiseTo(prefs);
    resetSmoothBearingTo(prefs);
    resetTransformationTo(prefs);
}

void PreferencesDialog::on_resetButton_clicked()
{
    if (ui->tabWidget->currentWidget()->objectName() == "tabGeneral")
    {
        resetGeneralTo(mPrefsDefaults);
        return;
    }
    if (ui->tabWidget->currentWidget()->objectName() == "tabFeatures")
    {
        resetFeaturesTo(mPrefsDefaults);
        return;
    }
    if (ui->tabWidget->currentWidget()->objectName() == "tabUnaries")
    {
        resetUnariesTo(mPrefsDefaults);
        return;
    }
    if (ui->tabWidget->currentWidget()->objectName() == "tabPairwise")
    {
        resetPairwiseTo(mPrefsDefaults);
        return;
    }
    if (ui->tabWidget->currentWidget()->objectName() == "tabSmoothBearing")
    {
        resetSmoothBearingTo(mPrefsDefaults);
        return;
    }
    if (ui->tabWidget->currentWidget()->objectName() == "tabTransformation")
    {
        resetTransformationTo(mPrefsDefaults);
        return;
    }
}

void PreferencesDialog::on_resetAllButton_clicked() { initPreferences(mPrefsDefaults); }

void PreferencesDialog::on_enableSmoothBearing_toggled(bool value)
{
    ui->enableSmoothBearing->setChecked(value);
    ui->smoothBearingWindowSpin->setEnabled(value);
    ui->smoothBearingOutlierTolSpin->setEnabled(value);
}

void PreferencesDialog::on_removeCamMotion_toggled(bool value)
{
    ui->featureCombo->setEnabled(value);
}

Preferences PreferencesDialog::getPreferences() const
{
    Preferences p;
    // general
    p.cacheSize = ui->cacheSpin->value();
    p.chunkSize = ui->chunkSpin->value();
    p.detectionRadius = ui->radiusSpin->value();
    p.fps = ui->fpsSpin->value();
    p.pixelsPerMm = ui->pxMmSpin->value();

    // features
    p.featureType = stringToFeatureType(ui->featureCombo->currentText());
    p.numFeatures = ui->numFeaturesSpin->value();

    // unaries
    p.unarySubsample = ui->subsampleSpin->value();
    p.unarySigma = ui->unarySigmaSpin->value();
    p.removeRedLasers = ui->removeRedLasersCheck->isChecked();
    p.unarySuppress = ui->suppressSpin->value();
    p.unaryMultiplier = ui->manualMultiplierSpin->value();
    p.manualUnarySize = ui->manualUnarySizeSpin->value();

    // pairwise
    p.pairwiseSigma = ui->pairwiseSigmaSpin->value();
    p.pairwiseSize = ui->pairwiseSizeSpin->value();

    // smooth bearing
    p.smoothBearing = ui->enableSmoothBearing->isChecked();
    p.smoothBearingWindowSize = ui->smoothBearingWindowSpin->value();
    p.smoothBearingOutlierTol = ui->smoothBearingOutlierTolSpin->value();

    // trafo
    p.removeCamMotion = ui->removeCamMotion->isChecked();
    p.nnRatio = ui->nnRatioSpin->value();
    p.ranscacReproj = ui->maxReprojSpin->value();

    return p;
}

} // namespace gui
