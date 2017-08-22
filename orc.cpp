#include "orc.h"
#include "ui_orc.h"

orc::orc(QWidget *parent) : QMainWindow(parent), ui(new Ui::orc)
{
    ui->setupUi(this);

    ui->eulerAnglesFromComboBox->insertSeparator(6);
    ui->eulerAnglesFromComboBox->insertSeparator(13);
    ui->eulerAnglesFromComboBox->insertSeparator(14);
    ui->eulerAnglesFromComboBox->insertSeparator(21);
    ui->eulerAnglesToComboBox->insertSeparator(6);
    ui->eulerAnglesToComboBox->insertSeparator(13);
    ui->eulerAnglesToComboBox->insertSeparator(14);
    ui->eulerAnglesToComboBox->insertSeparator(21);
    ui->eulerAnglesConversionComboBox->insertSeparator(6);
    ui->eulerAnglesConversionComboBox->insertSeparator(13);
    ui->eulerAnglesConversionComboBox->insertSeparator(14);
    ui->eulerAnglesConversionComboBox->insertSeparator(21);
}

orc::~orc()
{
    delete ui;
}

/*CONVERSIONS*/
rotationMatrix orc::eulerAngles2rotationMatrix(const eulerAngles &euAn, const QComboBox *comboBox)
{
    switch (comboBox->currentIndex())
    {
    case 0:
        return XYZiZYXe2rotationMatrix(euAn);
        break;
    case 1:
        return XZYiYZXe2rotationMatrix(euAn);
        break;
    case 2:
        return YXZiZXYe2rotationMatrix(euAn);
        break;
    case 3:
        return YZXiXZYe2rotationMatrix(euAn);
        break;
    case 4:
        return ZXYiYXZe2rotationMatrix(euAn);
        break;
    case 5:
        return ZYXiXYZe2rotationMatrix(euAn);
        break;

    case 7:
        return XYXi2rotationMatrix(euAn);
        break;
    case 8:
        return XZXi2rotationMatrix(euAn);
        break;
    case 9:
        return YXYi2rotationMatrix(euAn);
        break;
    case 10:
        return YZYi2rotationMatrix(euAn);
        break;
    case 11:
        return ZXZi2rotationMatrix(euAn);
        break;
    case 12:
        return ZYZi2rotationMatrix(euAn);
        break;


    case 15:
        return ZYXiXYZe2rotationMatrix(euAn);
        break;
    case 16:
        return YZXiXZYe2rotationMatrix(euAn);
        break;
    case 17:
        return ZXYiYXZe2rotationMatrix(euAn);
        break;
    case 18:
        return XZYiYZXe2rotationMatrix(euAn);
        break;
    case 19:
        return YXZiZXYe2rotationMatrix(euAn);
        break;
    case 20:
        return XYZiZYXe2rotationMatrix(euAn);
        break;

    case 22:
        return XYXe2rotationMatrix(euAn);
        break;
    case 23:
        return XZXe2rotationMatrix(euAn);
        break;
    case 24:
        return YXYe2rotationMatrix(euAn);
        break;
    case 25:
        return YZYe2rotationMatrix(euAn);
        break;
    case 26:
        return ZXZe2rotationMatrix(euAn);
        break;
    case 27:
        return ZYZe2rotationMatrix(euAn);
        break;
    }
    QMessageBox::critical(this, "ComboBox Error", "ComboBox out of boundaries!");
    rotationMatrix rotM_error;
    rotM_error.r11=-1;
    rotM_error.r12=-1;
    rotM_error.r13=-1;
    rotM_error.r21=-1;
    rotM_error.r22=-1;
    rotM_error.r23=-1;
    rotM_error.r31=-1;
    rotM_error.r32=-1;
    rotM_error.r33=-1;
    return rotM_error;
}

eulerAngles orc::rotationMatrix2eulerAngles(const rotationMatrix &rotM, const QComboBox *comboBox)
{
    switch (comboBox->currentIndex())
    {
    case 0:
        return rotationMatrix2XYZiZYXe(rotM);
        break;
    case 1:
        return rotationMatrix2XZYiYZXe(rotM);
        break;
    case 2:
        return rotationMatrix2YXZiZXYe(rotM);
        break;
    case 3:
        return rotationMatrix2YZXiXZYe(rotM);
        break;
    case 4:
        return rotationMatrix2ZXYiYXZe(rotM);
        break;
    case 5:
        return rotationMatrix2ZYXiXYZe(rotM);
        break;

    case 7:
        return rotationMatrix2XYXi(rotM);
        break;
    case 8:
        return rotationMatrix2XZXi(rotM);
        break;
    case 9:
        return rotationMatrix2YXYi(rotM);
        break;
    case 10:
        return rotationMatrix2YZYi(rotM);
        break;
    case 11:
        return rotationMatrix2ZXZi(rotM);
        break;
    case 12:
        return rotationMatrix2ZYZi(rotM);
        break;


    case 15:
        return rotationMatrix2ZYXiXYZe(rotM);
        break;
    case 16:
        return rotationMatrix2YZXiXZYe(rotM);
        break;
    case 17:
        return rotationMatrix2ZXYiYXZe(rotM);
        break;
    case 18:
        return rotationMatrix2XZYiYZXe(rotM);
        break;
    case 19:
        return rotationMatrix2YXZiZXYe(rotM);
        break;
    case 20:
        return rotationMatrix2XYZiZYXe(rotM);
        break;

    case 22:
        return rotationMatrix2XYXe(rotM);
        break;
    case 23:
        return rotationMatrix2XZXe(rotM);
        break;
    case 24:
        return rotationMatrix2YXYe(rotM);
        break;
    case 25:
        return rotationMatrix2YZYe(rotM);
        break;
    case 26:
        return rotationMatrix2ZXZe(rotM);
        break;
    case 27:
        return rotationMatrix2ZYZe(rotM);
        break;
    }
    QMessageBox::critical(this, "ComboBox Error", "ComboBox out of boundaries!");
    eulerAngles euAn_error;
    euAn_error.e1=-1;
    euAn_error.e2=-1;
    euAn_error.e3=-1;
    return euAn_error;
}

/*READ*/
rotationMatrix orc::readRotationMatrix()
{
    rotationMatrix rotM_Read;
    rotM_Read.r11=ui->r11LineEdit->text().toDouble();
    rotM_Read.r12=ui->r12LineEdit->text().toDouble();
    rotM_Read.r13=ui->r13LineEdit->text().toDouble();
    rotM_Read.r21=ui->r21LineEdit->text().toDouble();
    rotM_Read.r22=ui->r22LineEdit->text().toDouble();
    rotM_Read.r23=ui->r23LineEdit->text().toDouble();
    rotM_Read.r31=ui->r31LineEdit->text().toDouble();
    rotM_Read.r32=ui->r32LineEdit->text().toDouble();
    rotM_Read.r33=ui->r33LineEdit->text().toDouble();
    return rotM_Read;
}

quaternions orc::readQuaternions()
{
    quaternions quat_Read;
    quat_Read.q1=ui->q1LineEdit->text().toDouble();
    quat_Read.q2=ui->q2LineEdit->text().toDouble();
    quat_Read.q3=ui->q3LineEdit->text().toDouble();
    quat_Read.w=ui->wLineEdit->text().toDouble();
    return quat_Read;
}

angleAxis orc::readAngleAxis()
{
    angleAxis anAx_Read;
    anAx_Read.k1=ui->k1LineEdit->text().toDouble();
    anAx_Read.k2=ui->k2LineEdit->text().toDouble();
    anAx_Read.k3=ui->k3LineEdit->text().toDouble();
    anAx_Read.theta=ui->thetaLineEdit->text().toDouble();
    return anAx_Read;
}

angleAxisWithMagnitude orc::readAngleAxisWithMagnitude()
{
    angleAxisWithMagnitude anAxWithM_Read;
    anAxWithM_Read.rX=ui->RxLineEdit->text().toDouble();
    anAxWithM_Read.rY=ui->RyLineEdit->text().toDouble();
    anAxWithM_Read.rZ=ui->RzLineEdit->text().toDouble();
    return anAxWithM_Read;
}

eulerAngles orc::readEulerAngles()
{
    eulerAngles euAn_Read;
    if(ui->degreeRadioButton->isChecked())
    {
        euAn_Read.e1=ui->e1LineEdit->text().toDouble()/180*M_PI;
        euAn_Read.e2=ui->e2LineEdit->text().toDouble()/180*M_PI;
        euAn_Read.e3=ui->e3LineEdit->text().toDouble()/180*M_PI;
    }
    else
    {
        euAn_Read.e1=ui->e1LineEdit->text().toDouble();
        euAn_Read.e2=ui->e2LineEdit->text().toDouble();
        euAn_Read.e3=ui->e3LineEdit->text().toDouble();
    }
    return euAn_Read;
}

/*UPDATE*/
void orc::updateRotationMatrix(const rotationMatrix &rotM)
{
    ui->r11LineEdit->setText(QString::number(rotM.r11));
    ui->r12LineEdit->setText(QString::number(rotM.r12));
    ui->r13LineEdit->setText(QString::number(rotM.r13));
    ui->r21LineEdit->setText(QString::number(rotM.r21));
    ui->r22LineEdit->setText(QString::number(rotM.r22));
    ui->r23LineEdit->setText(QString::number(rotM.r23));
    ui->r31LineEdit->setText(QString::number(rotM.r31));
    ui->r32LineEdit->setText(QString::number(rotM.r32));
    ui->r33LineEdit->setText(QString::number(rotM.r33));
}

void orc::updateQuaternions(const quaternions &quat)
{
    ui->q1LineEdit->setText(QString::number(quat.q1));
    ui->q2LineEdit->setText(QString::number(quat.q2));
    ui->q3LineEdit->setText(QString::number(quat.q3));
    ui->wLineEdit->setText(QString::number(quat.w));
}

void orc::updateAngleAxis(const angleAxis &anAx)
{
    ui->k1LineEdit->setText(QString::number(anAx.k1));
    ui->k2LineEdit->setText(QString::number(anAx.k2));
    ui->k3LineEdit->setText(QString::number(anAx.k3));
    ui->thetaLineEdit->setText(QString::number(anAx.theta));
}

void orc::updateAngleAxisWithMagnitude(const angleAxisWithMagnitude &anAxWithM)
{
    ui->RxLineEdit->setText(QString::number(anAxWithM.rX));
    ui->RyLineEdit->setText(QString::number(anAxWithM.rY));
    ui->RzLineEdit->setText(QString::number(anAxWithM.rZ));
}

void orc::updateEulerAngles(const eulerAngles &euAn)
{
    if(ui->degreeRadioButton->isChecked())
    {
        ui->e1LineEdit->setText(QString::number(euAn.e1*180/M_PI));
        ui->e2LineEdit->setText(QString::number(euAn.e2*180/M_PI));
        ui->e3LineEdit->setText(QString::number(euAn.e3*180/M_PI));
    }
    else
    {
        ui->e1LineEdit->setText(QString::number(euAn.e1));
        ui->e2LineEdit->setText(QString::number(euAn.e2));
        ui->e3LineEdit->setText(QString::number(euAn.e3));
    }
}


/****************/
/*CONVERSION TAB*/
/****************/
/*CONVERT PUSH BUTTONS*/
void orc::on_rotationMatrixConvertPushButton_clicked()
{
    rotM_UI=orc::readRotationMatrix();

    quat_UI=rotationMatrix2quaternions(rotM_UI);
    orc::updateQuaternions(quat_UI);
    anAx_UI=rotationMatrix2angleAxis(rotM_UI);
    orc::updateAngleAxis(anAx_UI);
    anAxWithM_UI=rotationMatrix2angleAxisWithMagnitude(rotM_UI);
    orc::updateAngleAxisWithMagnitude(anAxWithM_UI);
    euAn_UI=rotationMatrix2eulerAngles(rotM_UI,ui->eulerAnglesConversionComboBox);
    orc::updateEulerAngles(euAn_UI);

    if(!isRotationMatrixOrthogonal(rotM_UI))
    {
        ui->invalidConversionWarningLabel->setText("Rotation matrix is NOT orthogonal!");
    }
    else
    {
        ui->invalidConversionWarningLabel->setText("");
    }
}

void orc::on_quaternionsConvertPushButton_clicked()
{
    quat_UI=orc::readQuaternions();
    rotM_UI=quaternions2rotationMatrix(quat_UI);
    orc::updateRotationMatrix(rotM_UI);

    anAx_UI=rotationMatrix2angleAxis(rotM_UI);
    orc::updateAngleAxis(anAx_UI);
    anAxWithM_UI=rotationMatrix2angleAxisWithMagnitude(rotM_UI);
    orc::updateAngleAxisWithMagnitude(anAxWithM_UI);
    euAn_UI=rotationMatrix2eulerAngles(rotM_UI,ui->eulerAnglesConversionComboBox);
    orc::updateEulerAngles(euAn_UI);

    if(!isQuaternionsNormalised(quat_UI))
    {
        ui->invalidConversionWarningLabel->setText("Quaternions are NOT normalised!");
    }
    else
    {
        ui->invalidConversionWarningLabel->setText("");
    }
}

void orc::on_angleAxisConvertPushButton_clicked()
{
    anAx_UI=orc::readAngleAxis();
    rotM_UI=angleAxis2rotationMatrix(anAx_UI);
    orc::updateRotationMatrix(rotM_UI);

    quat_UI=rotationMatrix2quaternions(rotM_UI);
    orc::updateQuaternions(quat_UI);
    anAxWithM_UI=rotationMatrix2angleAxisWithMagnitude(rotM_UI);
    orc::updateAngleAxisWithMagnitude(anAxWithM_UI);
    euAn_UI=rotationMatrix2eulerAngles(rotM_UI,ui->eulerAnglesConversionComboBox);
    orc::updateEulerAngles(euAn_UI);

    if(!isAngleAxisLengthOne(anAx_UI))
    {
        ui->invalidConversionWarningLabel->setText("Length of the axis of rotation is NOT one!");
    }
    else
    {
        ui->invalidConversionWarningLabel->setText("");
    }
}

void orc::on_angleAxisWithMagnitudeConvertPushButton_clicked()
{
    anAxWithM_UI=readAngleAxisWithMagnitude();
    anAx_UI=angleAxisWithMagnitude2angleAxis(anAxWithM_UI);
    orc::updateAngleAxis(anAx_UI);
    rotM_UI=angleAxis2rotationMatrix(anAx_UI);
    orc::updateRotationMatrix(rotM_UI);

    quat_UI=rotationMatrix2quaternions(rotM_UI);
    orc::updateQuaternions(quat_UI);
    euAn_UI=rotationMatrix2eulerAngles(rotM_UI,ui->eulerAnglesConversionComboBox);
    orc::updateEulerAngles(euAn_UI);

    ui->invalidConversionWarningLabel->setText("");
}

void orc::on_eulerAnglesConvertPushButton_clicked()
{
    euAn_UI=readEulerAngles();
    rotM_UI=eulerAngles2rotationMatrix(euAn_UI,ui->eulerAnglesConversionComboBox);
    orc::updateRotationMatrix(rotM_UI);

    quat_UI=rotationMatrix2quaternions(rotM_UI);
    orc::updateQuaternions(quat_UI);
    anAx_UI=rotationMatrix2angleAxis(rotM_UI);
    orc::updateAngleAxis(anAx_UI);
    anAxWithM_UI=rotationMatrix2angleAxisWithMagnitude(rotM_UI);
    orc::updateAngleAxisWithMagnitude(anAxWithM_UI);

    ui->invalidConversionWarningLabel->setText("");
}

/*EULER ANGLES CONVENTIONS COMBO BOX*/
void orc::on_eulerAnglesConversionComboBox_currentIndexChanged(int index)
{
    switch (index) {
    case 0:
        ui->e1Label->setText("X");
        ui->e2Label->setText("Y'");
        ui->e3Label->setText("Z''");
        break;
    case 1:
        ui->e1Label->setText("X");
        ui->e2Label->setText("Z'");
        ui->e3Label->setText("Y''");
        break;
    case 2:
        ui->e1Label->setText("Y");
        ui->e2Label->setText("X'");
        ui->e3Label->setText("Z''");
        break;
    case 3:
        ui->e1Label->setText("Y");
        ui->e2Label->setText("Z'");
        ui->e3Label->setText("X''");
        break;
    case 4:
        ui->e1Label->setText("Z");
        ui->e2Label->setText("X'");
        ui->e3Label->setText("Y''");
        break;
    case 5:
        ui->e1Label->setText("Z");
        ui->e2Label->setText("Y'");
        ui->e3Label->setText("X''");
        break;

    case 7:
        ui->e1Label->setText("X");
        ui->e2Label->setText("Y'");
        ui->e3Label->setText("X''");
        break;
    case 8:
        ui->e1Label->setText("X");
        ui->e2Label->setText("Z'");
        ui->e3Label->setText("X''");
        break;
    case 9:
        ui->e1Label->setText("Y");
        ui->e2Label->setText("X'");
        ui->e3Label->setText("Y''");
        break;
    case 10:
        ui->e1Label->setText("Y");
        ui->e2Label->setText("Z'");
        ui->e3Label->setText("Y''");
        break;
    case 11:
        ui->e1Label->setText("Z");
        ui->e2Label->setText("X'");
        ui->e3Label->setText("Z''");
        break;
    case 12:
        ui->e1Label->setText("Z");
        ui->e2Label->setText("Y'");
        ui->e3Label->setText("Z''");
        break;


    case 15:
        ui->e1Label->setText("X");
        ui->e2Label->setText("Y");
        ui->e3Label->setText("Z");
        break;
    case 16:
        ui->e1Label->setText("X");
        ui->e2Label->setText("Z");
        ui->e3Label->setText("Y");
        break;
    case 17:
        ui->e1Label->setText("Y");
        ui->e2Label->setText("X");
        ui->e3Label->setText("Z");
        break;
    case 18:
        ui->e1Label->setText("Y");
        ui->e2Label->setText("Z");
        ui->e3Label->setText("X");
        break;
    case 19:
        ui->e1Label->setText("Z");
        ui->e2Label->setText("X");
        ui->e3Label->setText("Y");
        break;
    case 20:
        ui->e1Label->setText("Z");
        ui->e2Label->setText("Y");
        ui->e3Label->setText("X");
        break;

    case 22:
        ui->e1Label->setText("X");
        ui->e2Label->setText("Y");
        ui->e3Label->setText("X");
        break;
    case 23:
        ui->e1Label->setText("X");
        ui->e2Label->setText("Z");
        ui->e3Label->setText("X");
        break;
    case 24:
        ui->e1Label->setText("Y");
        ui->e2Label->setText("X");
        ui->e3Label->setText("Y");
        break;
    case 25:
        ui->e1Label->setText("Y");
        ui->e2Label->setText("Z");
        ui->e3Label->setText("Y");
        break;
    case 26:
        ui->e1Label->setText("Z");
        ui->e2Label->setText("X");
        ui->e3Label->setText("Z");
        break;
    case 27:
        ui->e1Label->setText("Z");
        ui->e2Label->setText("Y");
        ui->e3Label->setText("Z");
        break;

    default:
        QMessageBox::critical(this, "ComboBox Error", "ComboBox out of boundaries!");
        break;
    }
    if(ui->e1LineEdit->text().toDouble()!=0 || ui->e2LineEdit->text().toDouble()!=0 || ui->e3LineEdit->text().toDouble()!=0)
    {
        orc::on_eulerAnglesConvertPushButton_clicked();
        ui->invalidConversionWarningLabel->setText("");
    }
}

/*RAD/DEG RADIO BUTTONS*/
void orc::on_radianRadioButton_toggled(bool checked)
{
    if(checked)
    {
        ui->e1LineEdit->setText(QString::number(ui->e1LineEdit->text().toDouble()/180*M_PI));
        ui->e2LineEdit->setText(QString::number(ui->e2LineEdit->text().toDouble()/180*M_PI));
        ui->e3LineEdit->setText(QString::number(ui->e3LineEdit->text().toDouble()/180*M_PI));
    }
}

void orc::on_degreeRadioButton_toggled(bool checked)
{
    if(checked)
    {
        ui->e1LineEdit->setText(QString::number(ui->e1LineEdit->text().toDouble()*180/M_PI));
        ui->e2LineEdit->setText(QString::number(ui->e2LineEdit->text().toDouble()*180/M_PI));
        ui->e3LineEdit->setText(QString::number(ui->e3LineEdit->text().toDouble()*180/M_PI));
    }
}

/*CLEAR PUSH BUTTON*/
void orc::on_clearPushButton_clicked()
{
    ui->r11LineEdit->setText("");
    ui->r12LineEdit->setText("");
    ui->r13LineEdit->setText("");
    ui->r21LineEdit->setText("");
    ui->r22LineEdit->setText("");
    ui->r23LineEdit->setText("");
    ui->r31LineEdit->setText("");
    ui->r32LineEdit->setText("");
    ui->r33LineEdit->setText("");

    ui->q1LineEdit->setText("");
    ui->q2LineEdit->setText("");
    ui->q3LineEdit->setText("");
    ui->wLineEdit->setText("");

    ui->k1LineEdit->setText("");
    ui->k2LineEdit->setText("");
    ui->k3LineEdit->setText("");
    ui->thetaLineEdit->setText("");

    ui->RxLineEdit->setText("");
    ui->RyLineEdit->setText("");
    ui->RzLineEdit->setText("");

    ui->e1LineEdit->setText("");
    ui->e2LineEdit->setText("");
    ui->e3LineEdit->setText("");
}

/*EXIT PUSH BUTTON*/
void orc::on_conversionTabExitPushButton_clicked()
{
    QApplication::quit();
}


/**********/
/*FILE TAB*/
/**********/
/*FILE BROWSE PUSH BUTTONS*/
void orc::on_selectInputFilePushButton_clicked()
{
    ui->inputLineEdit->setText(QFileDialog::getOpenFileName());
}

void orc::on_selectOutputFilePushButton_clicked()
{
    ui->outputLineEdit->setText(QFileDialog::getSaveFileName());
}

/*FILE CONVERT PUSH BUTTON*/
void orc::on_fileConvertPushButton_clicked()
{
    lineCounter=1;
    inputFile.QFile::setFileName(ui->inputLineEdit->text());
    outputFile.QFile::setFileName(ui->outputLineEdit->text());
    if(!inputFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QMessageBox::critical(this, "InputFile Error", "InputFile could NOT be opened!");
    }
    else
    {
        fromLineChanger();
        toLineChanger();
        unitMultiplierChanger();
        QTextStream inputTextStream(&inputFile);

        while(!inputTextStream.atEnd())
        {
            inputFileLine=inputTextStream.readLine().simplified().replace(" ", "");
            while(!fromLine.isEmpty())
            {
                if(fromLine.at(0)==inputFileLine.at(0))
                {
                    fromLine.remove(0,1);
                    inputFileLine.remove(0,1);
                }
                else if(fromLine.at(0)=='\\')
                {
                    getTranslationFromLine();

                    switch (ui->stackedWidgetFrom->currentIndex())
                    {
                    case 0:
                        getQuatFromLine();
                        break;
                    case 1:
                        getAngleAxisFromLine();
                        break;
                    case 2:
                        getAngleAxisWithMagnitudeFromLine();
                        break;
                    case 3:
                        getEulerAnglesFromLine();
                        break;
                    }

                    getOtherVarFromLine();

                    if(fromLine.at(1)=='0') // "\0" - end character (ignore the rest of the line)
                    {
                        break;
                    }
                    else if(fromLine.at(1)=='i' || fromLine.at(1)=='I') // "\i" - ignore number
                    {
                        inputFileLine.remove(0,inputFileLine.indexOf(fromLine[2])+1);
                        fromLine.remove(0,3);
                    }
                }
                else
                {
                    break;
                    qDebug() << "Line" << lineCounter << "does NOT match!";
                    //TODO: line # does not matchinto the output log
                }
            }

            /*WRITE TO FILE*/
            //TODO: check margins to see whether input is valid and put into output log
            getIntermediateFileLineRotM();

            while(!toLine.isEmpty())
            {
                if(toLine[0]=='\\')
                {
                    setTranslationToLine();

                    switch (ui->representationComboBoxTo->currentIndex())
                    {
                    case 0:
                        setQuatToLine();
                        break;
                    case 1:
                        setAngleAxisToLine();
                        break;
                    case 2:
                        setAngleAxisWithMagnitudeToLine();
                        break;
                    case 3:
                        setEulerAnglesToLine();
                        break;
                    }

                    setOtherVarToLine();
                }
                else
                {
                    outputByteArray.append(toLine.at(0));
                    toLine.remove(0,1);
                }
            }
            lineCounter++;
            outputByteArray.append('\n');
            fromLineChanger();
            toLineChanger();
        }
        inputFile.close();
        if(!outputFile.open(QFile::WriteOnly | QFile::Text))
        {
            QMessageBox::critical(this, "OutputFile Error", "OutputFile could NOT be opened!");
        }
        else
        {
            outputFile.write(outputByteArray);
            outputFile.flush();
            outputFile.close();
        }
    }
}


/*ADDITIONAL CLICK BUTTOM FUNCTIONS*/
void orc::fromLineChanger()
{
    switch(ui->stackedWidgetFrom->currentIndex())
    {
    case 0:
        fromLine=ui->quaternionsFromLineEdit->text().simplified().replace(" ", "");
        break;
    case 1:
        fromLine=ui->angleAxisFromLineEdit->text().simplified().replace(" ", "");
        break;
    case 2:
        fromLine=ui->angleAxisWithMagnitudeFromLineEdit->text().simplified().replace(" ", "");
        break;
    case 3:
        fromLine=ui->eulerAnglesFromLineEdit->text().simplified().replace(" ", "");
        break;
    }
}

void orc::toLineChanger()
{
    switch(ui->stackedWidgetTo->currentIndex())
    {
    case 0:
        toLine=ui->quaternionsToLineEdit->text();
        break;
    case 1:
        toLine=ui->angleAxisToLineEdit->text();
        break;
    case 2:
        toLine=ui->angleAxisWithMagnitudeToLineEdit->text();
        break;
    case 3:
        toLine=ui->eulerAnglesToLineEdit->text();
        break;
    }
}

void orc::unitMultiplierChanger()
{
    if(ui->translationComboBoxFrom->currentIndex()==0 && ui->translationComboBoxTo->currentIndex()==1)
    {
        unitMultiplier=1000.0;
    }
    else if(ui->translationComboBoxFrom->currentIndex()==1 && ui->translationComboBoxTo->currentIndex()==0)
    {
        unitMultiplier=0.001;
    }
    else
    {
        unitMultiplier=1.0;
    }
}



void orc::getTranslationFromLine()
{
    switch(fromLine.toStdString().at(1))
    {
    case 'X':
    case 'x':
        inputFileX=inputFileLine.left(inputFileLine.indexOf(fromLine[2])).toDouble();
        inputFileLine.remove(0,inputFileLine.indexOf(fromLine[2])+1);
        fromLine.remove(0,3);
        qDebug() << "X found: " << inputFileX;
        break;
    case 'Y':
    case 'y':
        inputFileY=inputFileLine.left(inputFileLine.indexOf(fromLine[2])).toDouble();
        inputFileLine.remove(0,inputFileLine.indexOf(fromLine[2])+1);
        fromLine.remove(0,3);
        qDebug() << "Y found: " << inputFileY;
        break;
    case 'Z':
    case 'z':
        inputFileZ=inputFileLine.left(inputFileLine.indexOf(fromLine[2])).toDouble();
        inputFileLine.remove(0,inputFileLine.indexOf(fromLine[2])+1);
        fromLine.remove(0,3);
        qDebug() << "Z found: " << inputFileZ;
        break;
    }
}

void orc::getQuatFromLine()
{
    if(fromLine.toStdString().at(1)=='q' || fromLine.toStdString().at(1)=='Q')
    {
        switch (fromLine.toStdString().at(2))
        {
        case '1':
            fileLineQuat.q1=inputFileLine.left(inputFileLine.indexOf(fromLine[3])).toDouble();
            inputFileLine.remove(0,inputFileLine.indexOf(fromLine[3])+1);
            fromLine.remove(0,4);
            qDebug() << "q1 found: " << fileLineQuat.q1;
            break;
        case '2':
            fileLineQuat.q2=inputFileLine.left(inputFileLine.indexOf(fromLine[3])).toDouble();
            inputFileLine.remove(0,inputFileLine.indexOf(fromLine[3])+1);
            fromLine.remove(0,4);
            qDebug() << "q2 found: " << fileLineQuat.q2;
            break;
        case '3':
            fileLineQuat.q3=inputFileLine.left(inputFileLine.indexOf(fromLine[3])).toDouble();
            inputFileLine.remove(0,inputFileLine.indexOf(fromLine[3])+1);
            fromLine.remove(0,4);
            qDebug() << "q3 found: " << fileLineQuat.q3;
            break;
        }
    }
    else if(fromLine.toStdString().at(1)=='w' || fromLine.toStdString().at(1)=='W')
    {
        fileLineQuat.w=inputFileLine.left(inputFileLine.indexOf(fromLine[2])).toDouble();
        inputFileLine.remove(0,inputFileLine.indexOf(fromLine[2])+1);
        fromLine.remove(0,3);
        qDebug() << "w found: " << fileLineQuat.w;
    }
}

void orc::getAngleAxisFromLine()
{
    if(fromLine.toStdString().at(1)=='k' || fromLine.toStdString().at(1)=='K')
    {
        switch (fromLine.toStdString().at(2))
        {
        case '1':
            fileLineAnAx.k1=inputFileLine.left(inputFileLine.indexOf(fromLine[3])).toDouble();
            inputFileLine.remove(0,inputFileLine.indexOf(fromLine[3])+1);
            fromLine.remove(0,4);
            qDebug() << "k1 found: " << fileLineAnAx.k1;
            break;
        case '2':
            fileLineAnAx.k2=inputFileLine.left(inputFileLine.indexOf(fromLine[3])).toDouble();
            inputFileLine.remove(0,inputFileLine.indexOf(fromLine[3])+1);
            fromLine.remove(0,4);
            qDebug() << "k2 found: " << fileLineAnAx.k2;
            break;
        case '3':
            fileLineAnAx.k3=inputFileLine.left(inputFileLine.indexOf(fromLine[3])).toDouble();
            inputFileLine.remove(0,inputFileLine.indexOf(fromLine[3])+1);
            fromLine.remove(0,4);
            qDebug() << "k3 found: " << fileLineAnAx.k3;
            break;
        }
    }
    else if(QString::compare(fromLine.mid(1,5), "theta", Qt::CaseInsensitive)==0)
    {
        fileLineAnAx.theta=inputFileLine.left(inputFileLine.indexOf(fromLine[6])).toDouble();
        inputFileLine.remove(0,inputFileLine.indexOf(fromLine[6])+1);
        fromLine.remove(0,7);
        qDebug() << "theta found: " << fileLineAnAx.theta;
    }
}


void orc::getAngleAxisWithMagnitudeFromLine()
{
    if(fromLine.toStdString().at(1)=='R' || fromLine.toStdString().at(1)=='r')
    {
        switch (fromLine.toStdString().at(2))
        {
        case 'x':
        case 'X':
            fileLineAnAxWithM.rX=inputFileLine.left(inputFileLine.indexOf(fromLine[3])).toDouble();
            inputFileLine.remove(0,inputFileLine.indexOf(fromLine[3])+1);
            fromLine.remove(0,4);
            qDebug() << "rX found: " << fileLineAnAxWithM.rX;
            break;
        case 'y':
        case 'Y':
            fileLineAnAxWithM.rY=inputFileLine.left(inputFileLine.indexOf(fromLine[3])).toDouble();
            inputFileLine.remove(0,inputFileLine.indexOf(fromLine[3])+1);
            fromLine.remove(0,4);
            qDebug() << "rY found: " << fileLineAnAxWithM.rY;
            break;
        case 'z':
        case 'Z':
            fileLineAnAxWithM.rZ=inputFileLine.left(inputFileLine.indexOf(fromLine[3])).toDouble();
            inputFileLine.remove(0,inputFileLine.indexOf(fromLine[3])+1);
            fromLine.remove(0,4);
            qDebug() << "rZ found: " << fileLineAnAxWithM.rZ;
            break;
        }
    }
}

void orc::getEulerAnglesFromLine()
{
    if(fromLine.toStdString().at(1)=='e' || fromLine.toStdString().at(1)=='E')
    {
        switch (fromLine.toStdString().at(2))
        {
        case '1':
            fileLineEuAn.e1=inputFileLine.left(inputFileLine.indexOf(fromLine[3])).toDouble();
            inputFileLine.remove(0,inputFileLine.indexOf(fromLine[3])+1);
            fromLine.remove(0,4);
            qDebug() << "e1 found: " << fileLineEuAn.e1;
            break;
        case '2':
            fileLineEuAn.e2=inputFileLine.left(inputFileLine.indexOf(fromLine[3])).toDouble();
            inputFileLine.remove(0,inputFileLine.indexOf(fromLine[3])+1);
            fromLine.remove(0,4);
            qDebug() << "e2 found: " << fileLineEuAn.e2;
            break;
        case '3':
            fileLineEuAn.e3=inputFileLine.left(inputFileLine.indexOf(fromLine[3])).toDouble();
            inputFileLine.remove(0,inputFileLine.indexOf(fromLine[3])+1);
            fromLine.remove(0,4);
            qDebug() << "e3 found: " << fileLineEuAn.e3;
            break;
        }
    }
}

void orc::getOtherVarFromLine()
{
    if(QString::compare(fromLine.mid(1,3), "var", Qt::CaseInsensitive)==0)
    {
     //TODO: dynamic container containing 4th char as identifier and data in form of string/double - not urgent - useful only if commands have different parameters
    }
}

void orc::getIntermediateFileLineRotM()
{
    switch (ui->representationComboBoxFrom->currentIndex())
    {
    case 0:
        intermediateFileLineRotM=quaternions2rotationMatrix(fileLineQuat);
        break;
    case 1:
        intermediateFileLineRotM=angleAxis2rotationMatrix(fileLineAnAx);
        break;
    case 2:
        intermediateFileLineRotM=angleAxis2rotationMatrix(angleAxisWithMagnitude2angleAxis(fileLineAnAxWithM));
        break;
    case 3:
        intermediateFileLineRotM=eulerAngles2rotationMatrix(fileLineEuAn,ui->eulerAnglesFromComboBox);
        break;
    }
}

/*write to file*/
void orc::setTranslationToLine()
{
    switch(toLine.toStdString().at(1))
    {
    case 'X':
    case 'x':
        outputByteArray.append(QString::number(inputFileX*unitMultiplier));
        toLine.remove(0,2);
        break;
    case 'Y':
    case 'y':
        outputByteArray.append(QString::number(inputFileY*unitMultiplier));
        toLine.remove(0,2);
        break;
    case 'Z':
    case 'z':
        outputByteArray.append(QString::number(inputFileZ*unitMultiplier));
        toLine.remove(0,2);
        break;
    }
}

void orc::setQuatToLine()
{
    fileLineQuat=rotationMatrix2quaternions(intermediateFileLineRotM);
    if(toLine.toStdString().at(1)=='q' || toLine.toStdString().at(1)=='Q')
    {
        switch (toLine.toStdString().at(2))
        {
        case '1':
            outputByteArray.append(QString::number(fileLineQuat.q1));
            toLine.remove(0,3);
            break;
        case '2':
            outputByteArray.append(QString::number(fileLineQuat.q2));
            toLine.remove(0,3);
            break;
        case '3':
            outputByteArray.append(QString::number(fileLineQuat.q3));
            toLine.remove(0,3);
            break;
        }
    }
    else if(toLine.toStdString().at(1)=='w' || toLine.toStdString().at(1)=='W')
    {
        outputByteArray.append(QString::number(fileLineQuat.w));
        toLine.remove(0,2);
    }
}

void orc::setAngleAxisToLine()
{
    fileLineAnAx=rotationMatrix2angleAxis(intermediateFileLineRotM);
    if(toLine.toStdString().at(1)=='k' || toLine.toStdString().at(1)=='K')
    {
        switch (toLine.toStdString().at(2))
        {
        case '1':
            outputByteArray.append(QString::number(fileLineAnAx.k1));
            toLine.remove(0,3);
            break;
        case '2':
            outputByteArray.append(QString::number(fileLineAnAx.k2));
            toLine.remove(0,3);
            break;
        case '3':
            outputByteArray.append(QString::number(fileLineAnAx.k3));
            toLine.remove(0,3);
            break;
        }
    }
    else if(QString::compare(toLine.mid(1,5), "theta", Qt::CaseInsensitive)==0)
    {
        outputByteArray.append(QString::number(fileLineAnAx.theta));
        toLine.remove(0,6);
    }
}

void orc::setAngleAxisWithMagnitudeToLine()
{
    fileLineAnAxWithM=rotationMatrix2angleAxisWithMagnitude(intermediateFileLineRotM);
    if(toLine.toStdString().at(1)=='R' || toLine.toStdString().at(1)=='r')
    {
        switch (toLine.toStdString().at(2))
        {
        case 'x':
        case 'X':
            outputByteArray.append(QString::number(fileLineAnAxWithM.rX));
            toLine.remove(0,3);
            break;
        case 'y':
        case 'Y':
            outputByteArray.append(QString::number(fileLineAnAxWithM.rY));
            toLine.remove(0,3);
            break;
        case 'z':
        case 'Z':
            outputByteArray.append(QString::number(fileLineAnAxWithM.rZ));
            toLine.remove(0,3);
            break;
        }
    }
}

void orc::setEulerAnglesToLine()
{
    fileLineEuAn=rotationMatrix2eulerAngles(intermediateFileLineRotM,ui->eulerAnglesToComboBox);
    if(toLine.toStdString().at(1)=='e' || toLine.toStdString().at(1)=='E')
    {
        switch (toLine.toStdString().at(2))
        {
        case '1':
            outputByteArray.append(QString::number(fileLineEuAn.e1));
            toLine.remove(0,3);
            break;
        case '2':
            outputByteArray.append(QString::number(fileLineEuAn.e2));
            toLine.remove(0,3);
            break;
        case '3':
            outputByteArray.append(QString::number(fileLineEuAn.e3));
            toLine.remove(0,3);
            break;
        }
    }
}

void orc::setOtherVarToLine()
{
    //TODO: ALL - not as urgent - useful only if commands have different parameters
}

/*REPRESENTATION COMBO BOXES*/
void orc::on_representationComboBoxFrom_currentIndexChanged(int index)
{
    ui->stackedWidgetFrom->setCurrentIndex(index);
}

void orc::on_representationComboBoxTo_currentIndexChanged(int index)
{
    ui->stackedWidgetTo->setCurrentIndex(index);
}

/*TEMPLATES PUSH BUTTONS*/
void orc::on_templatesPushButton_clicked()
{
    //TODO:
}

void orc::on_saveTemplatePushButton_clicked()
{
    //TODO:
}

/*EXIT PUSH BUTTON*/
void orc::on_fileTabExitPushButton_clicked()
{
    QApplication::quit();
}
