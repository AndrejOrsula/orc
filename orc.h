#ifndef ORC_H
#define ORC_H

#include <QDebug>

#include <QMainWindow>
#include <QFileDialog>
#include <QFile>
#include <QString>
#include <math.h>
#include <QComboBox>
#include <QRadioButton>
#include <QLineEdit>
#include <QLabel>

#define NORMALISE_ORTHOGONAL_LENGTH_MARGIN 0.005

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

///////
    struct rotationMatrix
    {
        double r11,r12,r13,r21,r22,r23,r31,r32,r33;
    };

    struct quaternions
    {
        double q1,q2,q3,w;
    };

    struct angleAxis
    {
        double k1,k2,k3,theta;
    };

    struct angleAxisWithMagnitude
    {
        double rX,rY,rZ;
    };

    struct eulerAngles
    {
        double e1,e2,e3;
    };

    rotationMatrix rotM_UI;
    quaternions quat_UI;
    angleAxis anAx_UI;
    angleAxisWithMagnitude anAxWithM_UI;
    eulerAngles euAn_UI;

    rotationMatrix quaternions2rotationMatrix(quaternions quat);
    rotationMatrix angleAxis2rotationMatrix(angleAxis anAx);
    angleAxis angleAxisWithMagnitude2angleAxis(angleAxisWithMagnitude anAxWithM);
    rotationMatrix XYZiZYXe2rotationMatrix(eulerAngles euAn);
    rotationMatrix XZYiYZXe2rotationMatrix(eulerAngles euAn);
    rotationMatrix YXZiZXYe2rotationMatrix(eulerAngles euAn);
    rotationMatrix YZXiXZYe2rotationMatrix(eulerAngles euAn);
    rotationMatrix ZXYiYXZe2rotationMatrix(eulerAngles euAn);
    rotationMatrix ZYXiXYZe2rotationMatrix(eulerAngles euAn);
    rotationMatrix XYXi2rotationMatrix(eulerAngles euAn);
    rotationMatrix XZXi2rotationMatrix(eulerAngles euAn);
    rotationMatrix YXYi2rotationMatrix(eulerAngles euAn);
    rotationMatrix YZYi2rotationMatrix(eulerAngles euAn);
    rotationMatrix ZXZi2rotationMatrix(eulerAngles euAn);
    rotationMatrix ZYZi2rotationMatrix(eulerAngles euAn);
    rotationMatrix XYXe2rotationMatrix(eulerAngles euAn);
    rotationMatrix XZXe2rotationMatrix(eulerAngles euAn);
    rotationMatrix YXYe2rotationMatrix(eulerAngles euAn);
    rotationMatrix YZYe2rotationMatrix(eulerAngles euAn);
    rotationMatrix ZXZe2rotationMatrix(eulerAngles euAn);
    rotationMatrix ZYZe2rotationMatrix(eulerAngles euAn);
    rotationMatrix eulerAngles2rotationMatrix(eulerAngles euAn);

    quaternions rotationMatrix2quaternions(rotationMatrix rotM);
    angleAxis rotationMatrix2angleAxis(rotationMatrix rotM);
    angleAxisWithMagnitude rotationMatrix2angleAxisWithMagnitude(rotationMatrix rotM);
    eulerAngles rotationMatrix2XYZiZYXe(rotationMatrix rotM);
    eulerAngles rotationMatrix2XZYiYZXe(rotationMatrix rotM);
    eulerAngles rotationMatrix2YXZiZXYe(rotationMatrix rotM);
    eulerAngles rotationMatrix2YZXiXZYe(rotationMatrix rotM);
    eulerAngles rotationMatrix2ZXYiYXZe(rotationMatrix rotM);
    eulerAngles rotationMatrix2ZYXiXYZe(rotationMatrix rotM);
    eulerAngles rotationMatrix2XYXi(rotationMatrix rotM);
    eulerAngles rotationMatrix2XZXi(rotationMatrix rotM);
    eulerAngles rotationMatrix2YXYi(rotationMatrix rotM);
    eulerAngles rotationMatrix2YZYi(rotationMatrix rotM);
    eulerAngles rotationMatrix2ZXZi(rotationMatrix rotM);
    eulerAngles rotationMatrix2ZYZi(rotationMatrix rotM);
    eulerAngles rotationMatrix2XYXe(rotationMatrix rotM);
    eulerAngles rotationMatrix2XZXe(rotationMatrix rotM);
    eulerAngles rotationMatrix2YXYe(rotationMatrix rotM);
    eulerAngles rotationMatrix2YZYe(rotationMatrix rotM);
    eulerAngles rotationMatrix2ZXZe(rotationMatrix rotM);
    eulerAngles rotationMatrix2ZYZe(rotationMatrix rotM);
    eulerAngles rotationMatrix2eulerAngles(rotationMatrix rotM);

    rotationMatrix readRotationMatrix();
    quaternions readQuaternions();
    angleAxis readAngleAxis();
    angleAxisWithMagnitude readAngleAxisWithMagnitude();
    eulerAngles readEulerAngles();

    void updateRotationMatrix(orc::rotationMatrix rotM);
    void updateQuaternions(orc::quaternions quat);
    void updateAngleAxis(orc::angleAxis anAx);
    void updateAngleAxisWithMagnitude(orc::angleAxisWithMagnitude anAxWithM);
    void updateEulerAngles(orc::eulerAngles euAn);

    bool isRotationMatrixOrthogonal(orc::rotationMatrix rotM);
    bool isQuaternionsNormalised(orc::quaternions quat);
    bool isAngleAxisLengthOne(orc::angleAxis anAx);

    QString fileName;
    QFile selectedFile;
    ///////

private slots:
    void on_rotationMatrixConvertPushButton_clicked();
    void on_quaternionsConvertPushButton_clicked();
    void on_angleAxisConvertPushButton_clicked();
    void on_angleAxisWithMagnitudeConvertPushButton_clicked();
    void on_eulerAnglesConvertPushButton_clicked();

    void on_selectInputFilePushButton_clicked();
    void on_selectOutputFilePushButton_clicked();

    void on_templatesPushButton_clicked();
    void on_saveTemplatePushButton_clicked();

    void on_fileConvertPushButton_clicked();

    void on_eulerAnglesConversionComboBox_currentIndexChanged(int index);
    void on_radianRadioButton_toggled(bool checked);
    void on_degreeRadioButton_toggled(bool checked);

    void on_quaternionsFromHelpPushButton_clicked();
    void on_quaternionsToHelpPushButton_clicked();
    void on_angleAxisFromHelpPushButton_clicked();
    void on_angleAxisToHelpPushButton_clicked();
    void on_angleAxisWithMagnitudeFromHelpPushButton_clicked();
    void on_angleAxisWithMagnitudeToHelpPushButton_clicked();
    void on_eulerAnglesFromHelpPushButton_clicked();
    void on_eulerAnglesToHelpPushButton_clicked();

    void on_clearPushButton_clicked();

    void on_fileTabExitPushButton_clicked();
    void on_conversionTabExitPushButton_clicked();

private:
    Ui::orc *ui;
};

#endif // ORC_H
