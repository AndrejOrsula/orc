#ifndef ORC_H
#define ORC_H

#include <QDebug> //TODO: remove when done

#include <QMainWindow>
#include <QFileDialog>
#include <QFile>
#include <QStackedWidget>
#include <QRadioButton>
#include <QComboBox>
#include <QString>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>

#include "spatialrotationconversions.h"
using namespace spatialRotationConversions; //TODO: come up with a better name

namespace Ui
{
class orc;
}

class orc : public QMainWindow
{
    Q_OBJECT

public:
    explicit orc(QWidget *parent = 0);
    ~orc();

private:
    Ui::orc *ui;

    /*CONVERSIONS*/
    rotationMatrix eulerAngles2rotationMatrix(const eulerAngles &euAn, const QComboBox *comboBox);
    eulerAngles rotationMatrix2eulerAngles(const rotationMatrix &rotM, const QComboBox *comboBox);

    /*READ*/
    rotationMatrix readRotationMatrix();
    quaternions readQuaternions();
    angleAxis readAngleAxis();
    angleAxisWithMagnitude readAngleAxisWithMagnitude();
    eulerAngles readEulerAngles();

    /*UPDATE*/
    void updateRotationMatrix(const rotationMatrix &rotM);
    void updateQuaternions(const quaternions &quat);
    void updateAngleAxis(const angleAxis &anAx);
    void updateAngleAxisWithMagnitude(const angleAxisWithMagnitude &anAxWithM);
    void updateEulerAngles(const eulerAngles &euAn);

    /*EXTRA CONVERT PUSH BUTTON FUNCTIONS*/
    /*READ FROM FILE*/
    void fromLineChanger();
    void toLineChanger();
    void unitMultiplierChanger();
    void getTranslationFromLine();
    void getQuatFromLine();
    void getAngleAxisFromLine();
    void getAngleAxisWithMagnitudeFromLine();
    void getEulerAnglesFromLine();
    void getOtherVarFromLine();
    void getIntermediateFileLineRotM();
    /*WRITE TO FILE*/
    void setTranslationToLine();
    void setQuatToLine();
    void setAngleAxisToLine();
    void setAngleAxisWithMagnitudeToLine();
    void setEulerAnglesToLine();
    void setOtherVarToLine();

    /*VARIABLES*/
    rotationMatrix rotM_UI, intermediateFileLineRotM;
    quaternions quat_UI, fileLineQuat;
    angleAxis anAx_UI, fileLineAnAx;
    angleAxisWithMagnitude anAxWithM_UI, fileLineAnAxWithM;
    eulerAngles euAn_UI, fileLineEuAn;

    QString fromLine, toLine, inputFileLine;
    QByteArray outputByteArray;
    QFile inputFile, outputFile;
    double inputFileX, inputFileY, inputFileZ;
    float unitMultiplier;
    int lineCounter;

private slots:
    /****************/
    /*CONVERSION TAB*/
    /****************/
    /*CONVERT PUSH BUTTONS*/
    void on_rotationMatrixConvertPushButton_clicked();
    void on_quaternionsConvertPushButton_clicked();
    void on_angleAxisConvertPushButton_clicked();
    void on_angleAxisWithMagnitudeConvertPushButton_clicked();
    void on_eulerAnglesConvertPushButton_clicked();

    /*EULER ANGLES CONVENTIONS COMBO BOX*/
    void on_eulerAnglesConversionComboBox_currentIndexChanged(int index);

    /*RAD OR DEG RADIO BUTTONS*/
    void on_radianRadioButton_toggled(bool checked);
    void on_degreeRadioButton_toggled(bool checked);

    /*CLEAR PUSH BUTTON*/
    void on_clearPushButton_clicked();

    /*EXIT PUSH BUTTON*/
    void on_conversionTabExitPushButton_clicked();


    /**********/
    /*FILE TAB*/
    /**********/
    /*FILE BROWSE PUSH BUTTONS*/
    void on_selectInputFilePushButton_clicked();
    void on_selectOutputFilePushButton_clicked();

    /*FILE CONVERT PUSH BUTTON*/
    void on_fileConvertPushButton_clicked();

    /*REPRESENTATION COMBO BOXES*/
    void on_representationComboBoxFrom_currentIndexChanged(int index);
    void on_representationComboBoxTo_currentIndexChanged(int index);

    /*TEMPLATES PUSH BUTTONS*/
    void on_templatesPushButton_clicked();
    void on_saveTemplatePushButton_clicked();

    /*EXIT PUSH BUTTON*/
    void on_fileTabExitPushButton_clicked();
};
#endif // ORC_H
