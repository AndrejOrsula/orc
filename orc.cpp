#include "orc.h"
#include "ui_orc.h"

//*CONSTRUCTOR*
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
//*CONSTRUCTOR* END

//*DESTRUCTOR*
orc::~orc()
{
    delete ui;
}
//*DESTRUCTOR* END

//***MATH CONVERSIONS***
//**TO ROTATION MATRIX**
orc::rotationMatrix orc::quaternions2rotationMatrix(orc::quaternions quat)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=1-2*quat.q2*quat.q2-2*quat.q3*quat.q3;
    rotM_Result.r12=2*(quat.q1*quat.q2-quat.q3*quat.w);
    rotM_Result.r13=2*(quat.q1*quat.q3+quat.q2*quat.w);
    rotM_Result.r21=2*(quat.q1*quat.q2+quat.q3*quat.w);
    rotM_Result.r22=1-2*quat.q1*quat.q1-2*quat.q3*quat.q3;
    rotM_Result.r23=2*(quat.q2*quat.q3-quat.q1*quat.w);
    rotM_Result.r31=2*(quat.q1*quat.q3-quat.q2*quat.w);
    rotM_Result.r32=2*(quat.q2*quat.q3+quat.q1*quat.w);
    rotM_Result.r33=1-2*quat.q1*quat.q1-2*quat.q2*quat.q2;
    return rotM_Result;
}

orc::rotationMatrix orc::angleAxis2rotationMatrix(orc::angleAxis anAx)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=anAx.k1*anAx.k1*(1-cos(anAx.theta))+cos(anAx.theta);
    rotM_Result.r12=anAx.k1*anAx.k2*(1-cos(anAx.theta))-anAx.k3*sin(anAx.theta);
    rotM_Result.r13=anAx.k1*anAx.k3*(1-cos(anAx.theta))+anAx.k2*sin(anAx.theta);
    rotM_Result.r21=anAx.k1*anAx.k2*(1-cos(anAx.theta))+anAx.k3*sin(anAx.theta);
    rotM_Result.r22=anAx.k2*anAx.k2*(1-cos(anAx.theta))+cos(anAx.theta);
    rotM_Result.r23=anAx.k2*anAx.k3*(1-cos(anAx.theta))-anAx.k1*sin(anAx.theta);
    rotM_Result.r31=anAx.k1*anAx.k3*(1-cos(anAx.theta))-anAx.k2*sin(anAx.theta);
    rotM_Result.r32=anAx.k2*anAx.k3*(1-cos(anAx.theta))+anAx.k1*sin(anAx.theta);
    rotM_Result.r33=anAx.k3*anAx.k3*(1-cos(anAx.theta))+cos(anAx.theta);
    return rotM_Result;
}

orc::angleAxis orc::angleAxisWithMagnitude2angleAxis(orc::angleAxisWithMagnitude anAxWithM)
{
    orc::angleAxis anAx_Result;
    anAx_Result.theta=sqrt(anAxWithM.rX*anAxWithM.rX+anAxWithM.rY*anAxWithM.rY+anAxWithM.rZ*anAxWithM.rZ);
    anAx_Result.k1=anAxWithM.rX/anAx_Result.theta;
    anAx_Result.k2=anAxWithM.rY/anAx_Result.theta;
    anAx_Result.k3=anAxWithM.rZ/anAx_Result.theta;
    return anAx_Result;
}

//*EULER ANGLES*
orc::rotationMatrix orc::XYZiZYXe2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=cos(euAn.e2)*cos(euAn.e3);
    rotM_Result.r12=-cos(euAn.e2)*sin(euAn.e3);
    rotM_Result.r13=sin(euAn.e2);
    rotM_Result.r21=sin(euAn.e1)*sin(euAn.e2)*cos(euAn.e3)+cos(euAn.e1)*sin(euAn.e3);
    rotM_Result.r22=-sin(euAn.e1)*sin(euAn.e2)*sin(euAn.e3)+cos(euAn.e1)*cos(euAn.e3);
    rotM_Result.r23=-sin(euAn.e1)*cos(euAn.e2);
    rotM_Result.r31=-cos(euAn.e1)*sin(euAn.e2)*cos(euAn.e3)+sin(euAn.e1)*sin(euAn.e3);
    rotM_Result.r32=cos(euAn.e1)*sin(euAn.e2)*sin(euAn.e3)+sin(euAn.e1)*cos(euAn.e3);
    rotM_Result.r33=cos(euAn.e1)*cos(euAn.e2);
    return rotM_Result;
}

orc::rotationMatrix orc::XZYiYZXe2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=cos(euAn.e2)*cos(euAn.e3);
    rotM_Result.r12=-sin(euAn.e2);
    rotM_Result.r13=cos(euAn.e2)*sin(euAn.e3);
    rotM_Result.r21=cos(euAn.e1)*sin(euAn.e2)*cos(euAn.e3)+sin(euAn.e1)*sin(euAn.e3);
    rotM_Result.r22=cos(euAn.e1)*cos(euAn.e2);
    rotM_Result.r23=cos(euAn.e1)*sin(euAn.e2)*sin(euAn.e3)-sin(euAn.e1)*cos(euAn.e3);
    rotM_Result.r31=sin(euAn.e1)*sin(euAn.e2)*cos(euAn.e3)-cos(euAn.e1)*sin(euAn.e3);
    rotM_Result.r32=sin(euAn.e1)*cos(euAn.e2);
    rotM_Result.r33=sin(euAn.e1)*sin(euAn.e2)*sin(euAn.e3)+cos(euAn.e1)*cos(euAn.e3);
    return rotM_Result;
}

orc::rotationMatrix orc::YXZiZXYe2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=sin(euAn.e1)*sin(euAn.e2)*sin(euAn.e3)+cos(euAn.e1)*cos(euAn.e3);
    rotM_Result.r12=sin(euAn.e1)*sin(euAn.e2)*cos(euAn.e3)-cos(euAn.e1)*sin(euAn.e3);
    rotM_Result.r13=sin(euAn.e1)*cos(euAn.e2);
    rotM_Result.r21=cos(euAn.e2)*sin(euAn.e3);
    rotM_Result.r22=cos(euAn.e2)*cos(euAn.e3);
    rotM_Result.r23=-sin(euAn.e2);
    rotM_Result.r31=cos(euAn.e1)*sin(euAn.e2)*sin(euAn.e3)-sin(euAn.e1)*cos(euAn.e3);
    rotM_Result.r32=cos(euAn.e1)*sin(euAn.e2)*cos(euAn.e3)+sin(euAn.e1)*sin(euAn.e3);
    rotM_Result.r33=cos(euAn.e1)*cos(euAn.e2);
    return rotM_Result;
}

orc::rotationMatrix orc::YZXiXZYe2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=cos(euAn.e1)*cos(euAn.e2);
    rotM_Result.r12=-cos(euAn.e1)*sin(euAn.e2)*cos(euAn.e3)+sin(euAn.e1)*sin(euAn.e3);
    rotM_Result.r13=cos(euAn.e1)*sin(euAn.e2)*sin(euAn.e3)+sin(euAn.e1)*cos(euAn.e3);
    rotM_Result.r21=sin(euAn.e2);
    rotM_Result.r22=cos(euAn.e2)*cos(euAn.e3);
    rotM_Result.r23=-cos(euAn.e2)*sin(euAn.e3);
    rotM_Result.r31=-sin(euAn.e1)*cos(euAn.e2);
    rotM_Result.r32=sin(euAn.e1)*sin(euAn.e2)*cos(euAn.e3)+cos(euAn.e1)*sin(euAn.e3);
    rotM_Result.r33=-sin(euAn.e1)*sin(euAn.e2)*sin(euAn.e3)+cos(euAn.e1)*cos(euAn.e3);
    return rotM_Result;
}

orc::rotationMatrix orc::ZXYiYXZe2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=-sin(euAn.e1)*sin(euAn.e2)*sin(euAn.e3)+cos(euAn.e1)*cos(euAn.e3);
    rotM_Result.r12=-sin(euAn.e1)*cos(euAn.e2);
    rotM_Result.r13=sin(euAn.e1)*sin(euAn.e2)*cos(euAn.e3)+cos(euAn.e1)*sin(euAn.e3);
    rotM_Result.r21=cos(euAn.e1)*sin(euAn.e2)*sin(euAn.e3)+sin(euAn.e1)*cos(euAn.e3);
    rotM_Result.r22=cos(euAn.e1)*cos(euAn.e2);
    rotM_Result.r23=-cos(euAn.e1)*sin(euAn.e2)*cos(euAn.e3)+sin(euAn.e1)*sin(euAn.e3);
    rotM_Result.r31=-cos(euAn.e2)*sin(euAn.e3);
    rotM_Result.r32=sin(euAn.e2);
    rotM_Result.r33=cos(euAn.e2)*cos(euAn.e3);
    return rotM_Result;
}

orc::rotationMatrix orc::ZYXiXYZe2rotationMatrix(orc::eulerAngles euAn) //roll-pitch-yaw (extrinsic)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=cos(euAn.e1)*cos(euAn.e2);
    rotM_Result.r12=cos(euAn.e1)*sin(euAn.e2)*sin(euAn.e3)-sin(euAn.e1)*cos(euAn.e3);
    rotM_Result.r13=cos(euAn.e1)*sin(euAn.e2)*cos(euAn.e3)+sin(euAn.e1)*sin(euAn.e3);
    rotM_Result.r21=sin(euAn.e1)*cos(euAn.e2);
    rotM_Result.r22=sin(euAn.e1)*sin(euAn.e2)*sin(euAn.e3)+cos(euAn.e1)*cos(euAn.e3);
    rotM_Result.r23=sin(euAn.e1)*sin(euAn.e2)*cos(euAn.e3)-cos(euAn.e1)*sin(euAn.e3);
    rotM_Result.r31=-sin(euAn.e2);
    rotM_Result.r32=cos(euAn.e2)*sin(euAn.e3);
    rotM_Result.r33=cos(euAn.e2)*cos(euAn.e3);
    return rotM_Result;
}

orc::rotationMatrix orc::XYXi2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=cos(euAn.e2);
    rotM_Result.r12=sin(euAn.e2)*sin(euAn.e3);
    rotM_Result.r13=sin(euAn.e2)*cos(euAn.e3);
    rotM_Result.r21=sin(euAn.e1)*sin(euAn.e2);
    rotM_Result.r22=-sin(euAn.e1)*cos(euAn.e2)*sin(euAn.e3)+cos(euAn.e1)*cos(euAn.e3);
    rotM_Result.r23=-sin(euAn.e1)*cos(euAn.e2)*cos(euAn.e3)-cos(euAn.e1)*sin(euAn.e3);
    rotM_Result.r31=-cos(euAn.e1)*sin(euAn.e2);
    rotM_Result.r32=cos(euAn.e1)*cos(euAn.e2)*sin(euAn.e3)+sin(euAn.e1)*cos(euAn.e3);
    rotM_Result.r33=cos(euAn.e1)*cos(euAn.e2)*cos(euAn.e3)-sin(euAn.e1)*sin(euAn.e3);
    return rotM_Result;
}

orc::rotationMatrix orc::XZXi2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=cos(euAn.e2);
    rotM_Result.r12=-sin(euAn.e2)*cos(euAn.e3);
    rotM_Result.r13=sin(euAn.e2)*sin(euAn.e3);
    rotM_Result.r21=cos(euAn.e1)*sin(euAn.e2);
    rotM_Result.r22=cos(euAn.e1)*cos(euAn.e2)*cos(euAn.e3)-sin(euAn.e1)*sin(euAn.e3);
    rotM_Result.r23=-cos(euAn.e1)*cos(euAn.e2)*sin(euAn.e3)-sin(euAn.e1)*cos(euAn.e3);
    rotM_Result.r31=sin(euAn.e1)*sin(euAn.e2);
    rotM_Result.r32=sin(euAn.e1)*cos(euAn.e2)*cos(euAn.e3)+cos(euAn.e1)*sin(euAn.e3);
    rotM_Result.r33=-sin(euAn.e1)*cos(euAn.e2)*sin(euAn.e3)+cos(euAn.e1)*cos(euAn.e3);
    return rotM_Result;
}

orc::rotationMatrix orc::YXYi2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=-sin(euAn.e1)*cos(euAn.e2)*sin(euAn.e3)+cos(euAn.e1)*cos(euAn.e3);
    rotM_Result.r12=sin(euAn.e1)*sin(euAn.e2);
    rotM_Result.r13=sin(euAn.e1)*cos(euAn.e2)*cos(euAn.e3)+cos(euAn.e1)*sin(euAn.e3);
    rotM_Result.r21=sin(euAn.e2)*sin(euAn.e3);
    rotM_Result.r22=cos(euAn.e2);
    rotM_Result.r23=-sin(euAn.e2)*cos(euAn.e3);
    rotM_Result.r31=-cos(euAn.e1)*cos(euAn.e2)*sin(euAn.e3)-sin(euAn.e1)*cos(euAn.e3);
    rotM_Result.r32=cos(euAn.e1)*sin(euAn.e2);
    rotM_Result.r33=cos(euAn.e1)*cos(euAn.e2)*cos(euAn.e3)-sin(euAn.e1)*sin(euAn.e3);
    return rotM_Result;
}

orc::rotationMatrix orc::YZYi2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=cos(euAn.e1)*cos(euAn.e2)*cos(euAn.e3)-sin(euAn.e1)*sin(euAn.e3);
    rotM_Result.r12=-cos(euAn.e1)*sin(euAn.e2);
    rotM_Result.r13=cos(euAn.e1)*cos(euAn.e2)*sin(euAn.e3)+sin(euAn.e1)*cos(euAn.e3);
    rotM_Result.r21=sin(euAn.e2)*cos(euAn.e3);
    rotM_Result.r22=cos(euAn.e2);
    rotM_Result.r23=sin(euAn.e2)*sin(euAn.e3);
    rotM_Result.r31=-sin(euAn.e1)*cos(euAn.e2)*cos(euAn.e3)-cos(euAn.e1)*sin(euAn.e3);
    rotM_Result.r32=sin(euAn.e1)*sin(euAn.e2);
    rotM_Result.r33=-sin(euAn.e1)*cos(euAn.e2)*sin(euAn.e3)+cos(euAn.e1)*cos(euAn.e3);
    return rotM_Result;
}

orc::rotationMatrix orc::ZXZi2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=-sin(euAn.e1)*cos(euAn.e2)*sin(euAn.e3)+cos(euAn.e1)*cos(euAn.e3);
    rotM_Result.r12=-sin(euAn.e1)*cos(euAn.e2)*cos(euAn.e3)-cos(euAn.e1)*sin(euAn.e3);
    rotM_Result.r13=sin(euAn.e1)*sin(euAn.e2);
    rotM_Result.r21=cos(euAn.e1)*cos(euAn.e2)*sin(euAn.e3)+sin(euAn.e1)*cos(euAn.e3);
    rotM_Result.r22=cos(euAn.e1)*cos(euAn.e2)*cos(euAn.e3)-sin(euAn.e1)*sin(euAn.e3);
    rotM_Result.r23=-cos(euAn.e1)*sin(euAn.e2);
    rotM_Result.r31=sin(euAn.e2)*sin(euAn.e3);
    rotM_Result.r32=sin(euAn.e2)*cos(euAn.e3);
    rotM_Result.r33=cos(euAn.e2);
    return rotM_Result;
}

orc::rotationMatrix orc::ZYZi2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=cos(euAn.e1)*cos(euAn.e2)*cos(euAn.e3)-sin(euAn.e1)*sin(euAn.e3);
    rotM_Result.r12=-cos(euAn.e1)*cos(euAn.e2)*sin(euAn.e3)-sin(euAn.e1)*cos(euAn.e3);
    rotM_Result.r13=cos(euAn.e1)*sin(euAn.e2);
    rotM_Result.r21=sin(euAn.e1)*cos(euAn.e2)*cos(euAn.e3)+cos(euAn.e1)*sin(euAn.e3);
    rotM_Result.r22=-sin(euAn.e1)*cos(euAn.e2)*sin(euAn.e3)+cos(euAn.e1)*cos(euAn.e3);
    rotM_Result.r23=sin(euAn.e1)*sin(euAn.e2);
    rotM_Result.r31=-sin(euAn.e2)*cos(euAn.e3);
    rotM_Result.r32=sin(euAn.e2)*sin(euAn.e3);
    rotM_Result.r33=cos(euAn.e2);
    return rotM_Result;
}

orc::rotationMatrix orc::XYXe2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=cos(euAn.e2);
    rotM_Result.r12=sin(euAn.e2)*sin(euAn.e1);
    rotM_Result.r13=sin(euAn.e2)*cos(euAn.e1);
    rotM_Result.r21=sin(euAn.e3)*sin(euAn.e2);
    rotM_Result.r22=-sin(euAn.e3)*cos(euAn.e2)*sin(euAn.e1)+cos(euAn.e3)*cos(euAn.e1);
    rotM_Result.r23=-sin(euAn.e3)*cos(euAn.e2)*cos(euAn.e1)-cos(euAn.e3)*sin(euAn.e1);
    rotM_Result.r31=-cos(euAn.e3)*sin(euAn.e2);
    rotM_Result.r32=cos(euAn.e3)*cos(euAn.e2)*sin(euAn.e1)+sin(euAn.e3)*cos(euAn.e1);
    rotM_Result.r33=cos(euAn.e3)*cos(euAn.e2)*cos(euAn.e1)-sin(euAn.e3)*sin(euAn.e1);
    return rotM_Result;
}

orc::rotationMatrix orc::XZXe2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=cos(euAn.e2);
    rotM_Result.r12=-sin(euAn.e2)*cos(euAn.e1);
    rotM_Result.r13=sin(euAn.e2)*sin(euAn.e1);
    rotM_Result.r21=cos(euAn.e3)*sin(euAn.e2);
    rotM_Result.r22=cos(euAn.e3)*cos(euAn.e2)*cos(euAn.e1)-sin(euAn.e3)*sin(euAn.e1);
    rotM_Result.r23=-cos(euAn.e3)*cos(euAn.e2)*sin(euAn.e1)-sin(euAn.e3)*cos(euAn.e1);
    rotM_Result.r31=sin(euAn.e3)*sin(euAn.e2);
    rotM_Result.r32=sin(euAn.e3)*cos(euAn.e2)*cos(euAn.e1)+cos(euAn.e3)*sin(euAn.e1);
    rotM_Result.r33=-sin(euAn.e3)*cos(euAn.e2)*sin(euAn.e1)+cos(euAn.e3)*cos(euAn.e1);
    return rotM_Result;
}

orc::rotationMatrix orc::YXYe2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=-sin(euAn.e3)*cos(euAn.e2)*sin(euAn.e1)+cos(euAn.e3)*cos(euAn.e1);
    rotM_Result.r12=sin(euAn.e3)*sin(euAn.e2);
    rotM_Result.r13=sin(euAn.e3)*cos(euAn.e2)*cos(euAn.e1)+cos(euAn.e3)*sin(euAn.e1);
    rotM_Result.r21=sin(euAn.e2)*sin(euAn.e1);
    rotM_Result.r22=cos(euAn.e2);
    rotM_Result.r23=-sin(euAn.e2)*cos(euAn.e1);
    rotM_Result.r31=-cos(euAn.e3)*cos(euAn.e2)*sin(euAn.e1)-sin(euAn.e3)*cos(euAn.e1);
    rotM_Result.r32=cos(euAn.e3)*sin(euAn.e2);
    rotM_Result.r33=cos(euAn.e3)*cos(euAn.e2)*cos(euAn.e1)-sin(euAn.e3)*sin(euAn.e1);
    return rotM_Result;
}

orc::rotationMatrix orc::YZYe2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=cos(euAn.e3)*cos(euAn.e2)*cos(euAn.e1)-sin(euAn.e3)*sin(euAn.e1);
    rotM_Result.r12=-cos(euAn.e3)*sin(euAn.e2);
    rotM_Result.r13=cos(euAn.e3)*cos(euAn.e2)*sin(euAn.e1)+sin(euAn.e3)*cos(euAn.e1);
    rotM_Result.r21=sin(euAn.e2)*cos(euAn.e1);
    rotM_Result.r22=cos(euAn.e2);
    rotM_Result.r23=sin(euAn.e2)*sin(euAn.e1);
    rotM_Result.r31=-sin(euAn.e3)*cos(euAn.e2)*cos(euAn.e1)-cos(euAn.e3)*sin(euAn.e1);
    rotM_Result.r32=sin(euAn.e3)*sin(euAn.e2);
    rotM_Result.r33=-sin(euAn.e3)*cos(euAn.e2)*sin(euAn.e1)+cos(euAn.e3)*cos(euAn.e1);
    return rotM_Result;
}

orc::rotationMatrix orc::ZXZe2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=-sin(euAn.e3)*cos(euAn.e2)*sin(euAn.e1)+cos(euAn.e3)*cos(euAn.e1);
    rotM_Result.r12=-sin(euAn.e3)*cos(euAn.e2)*cos(euAn.e1)-cos(euAn.e3)*sin(euAn.e1);
    rotM_Result.r13=sin(euAn.e3)*sin(euAn.e2);
    rotM_Result.r21=cos(euAn.e3)*cos(euAn.e2)*sin(euAn.e1)+sin(euAn.e3)*cos(euAn.e1);
    rotM_Result.r22=cos(euAn.e3)*cos(euAn.e2)*cos(euAn.e1)-sin(euAn.e3)*sin(euAn.e1);
    rotM_Result.r23=-cos(euAn.e3)*sin(euAn.e2);
    rotM_Result.r31=sin(euAn.e2)*sin(euAn.e1);
    rotM_Result.r32=sin(euAn.e2)*cos(euAn.e1);
    rotM_Result.r33=cos(euAn.e2);
    return rotM_Result;
}

orc::rotationMatrix orc::ZYZe2rotationMatrix(orc::eulerAngles euAn)
{
    orc::rotationMatrix rotM_Result;
    rotM_Result.r11=cos(euAn.e3)*cos(euAn.e2)*cos(euAn.e1)-sin(euAn.e3)*sin(euAn.e1);
    rotM_Result.r12=-cos(euAn.e3)*cos(euAn.e2)*sin(euAn.e1)-sin(euAn.e3)*cos(euAn.e1);
    rotM_Result.r13=cos(euAn.e3)*sin(euAn.e2);
    rotM_Result.r21=sin(euAn.e3)*cos(euAn.e2)*cos(euAn.e1)+cos(euAn.e3)*sin(euAn.e1);
    rotM_Result.r22=-sin(euAn.e3)*cos(euAn.e2)*sin(euAn.e1)+cos(euAn.e3)*cos(euAn.e1);
    rotM_Result.r23=sin(euAn.e3)*sin(euAn.e2);
    rotM_Result.r31=-sin(euAn.e2)*cos(euAn.e1);
    rotM_Result.r32=sin(euAn.e2)*sin(euAn.e1);
    rotM_Result.r33=cos(euAn.e2);
    return rotM_Result;
}

orc::rotationMatrix orc::eulerAngles2rotationMatrix(orc::eulerAngles euAn)
{
    switch (ui->eulerAnglesConversionComboBox->currentIndex())
    {
    case 0:
        return orc::XYZiZYXe2rotationMatrix(euAn);
        break;
    case 1:
        return orc::XZYiYZXe2rotationMatrix(euAn);
        break;
    case 2:
        return orc::YXZiZXYe2rotationMatrix(euAn);
        break;
    case 3:
        return orc::YZXiXZYe2rotationMatrix(euAn);
        break;
    case 4:
        return orc::ZXYiYXZe2rotationMatrix(euAn);
        break;
    case 5:
        return orc::ZYXiXYZe2rotationMatrix(euAn);
        break;

    case 7:
        return orc::XYXi2rotationMatrix(euAn);
        break;
    case 8:
        return orc::XZXi2rotationMatrix(euAn);
        break;
    case 9:
        return orc::YXYi2rotationMatrix(euAn);
        break;
    case 10:
        return orc::YZYi2rotationMatrix(euAn);
        break;
    case 11:
        return orc::ZXZi2rotationMatrix(euAn);
        break;
    case 12:
        return orc::ZYZi2rotationMatrix(euAn);
        break;


    case 15:
        return orc::ZYXiXYZe2rotationMatrix(euAn);
        break;
    case 16:
        return orc::YZXiXZYe2rotationMatrix(euAn);
        break;
    case 17:
        return orc::ZXYiYXZe2rotationMatrix(euAn);
        break;
    case 18:
        return orc::XZYiYZXe2rotationMatrix(euAn);
        break;
    case 19:
        return orc::YXZiZXYe2rotationMatrix(euAn);
        break;
    case 20:
        return orc::XYZiZYXe2rotationMatrix(euAn);
        break;

    case 22:
        return orc::XYXe2rotationMatrix(euAn);
        break;
    case 23:
        return orc::XZXe2rotationMatrix(euAn);
        break;
    case 24:
        return orc::YXYe2rotationMatrix(euAn);
        break;
    case 25:
        return orc::YZYe2rotationMatrix(euAn);
        break;
    case 26:
        return orc::ZXZe2rotationMatrix(euAn);
        break;
    case 27:
        return orc::ZYZe2rotationMatrix(euAn);
        break;
    }
    //TODO: WARNING MESSAGE
    orc::rotationMatrix rotM_error;
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
//*EULER ANGLES* END
//**TO ROTATION MATRIX** END


//**FROM ROTATION MATRIX**
orc::quaternions orc::rotationMatrix2quaternions(orc::rotationMatrix rotM)
{
    orc::quaternions quat_Result;
    quat_Result.w=0.5*sqrt(1+rotM.r11+rotM.r22+rotM.r33);
    quat_Result.q1=(rotM.r32-rotM.r23)/(4*quat_Result.w);
    quat_Result.q2=(rotM.r13-rotM.r31)/(4*quat_Result.w);
    quat_Result.q3=(rotM.r21-rotM.r12)/(4*quat_Result.w);
    return quat_Result;
}

orc::angleAxis orc::rotationMatrix2angleAxis(orc::rotationMatrix rotM)
{
    orc::angleAxis anAx_Result;
    anAx_Result.theta=acos(0.5*(rotM.r11+rotM.r22+rotM.r33-1));
    anAx_Result.k1=(rotM.r32-rotM.r23)/(2*sin(anAx_Result.theta));
    anAx_Result.k2=(rotM.r13-rotM.r31)/(2*sin(anAx_Result.theta));
    anAx_Result.k3=(rotM.r21-rotM.r12)/(2*sin(anAx_Result.theta));
    return anAx_Result;
}

orc::angleAxisWithMagnitude orc::rotationMatrix2angleAxisWithMagnitude(orc::rotationMatrix rotM)
{
    orc::angleAxisWithMagnitude anAxWithM_Result;
    orc::angleAxis anAx_Result=orc::rotationMatrix2angleAxis(rotM);
    anAxWithM_Result.rX=anAx_Result.k1*anAx_Result.theta;
    anAxWithM_Result.rY=anAx_Result.k2*anAx_Result.theta;
    anAxWithM_Result.rZ=anAx_Result.k3*anAx_Result.theta;
    return anAxWithM_Result;
}

//*EULER ANGLES*
orc::eulerAngles orc::rotationMatrix2XYZiZYXe(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if (rotM.r13<1)
    {
        if (rotM.r13>-1)
        {
            euAn_Result.e1=atan2(-rotM.r23,rotM.r33);
            euAn_Result.e2=asin(rotM.r13);
            euAn_Result.e3=atan2(-rotM.r12,rotM.r11);
        }
        else
        {
            euAn_Result.e1=-atan2(rotM.r21,rotM.r22);
            euAn_Result.e2=-M_PI/2;
            euAn_Result.e3=0;
        }
    }
    else
    {
        euAn_Result.e1=atan2(rotM.r21,rotM.r22);
        euAn_Result.e2=M_PI/2;
        euAn_Result.e3=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2XZYiYZXe(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r12<1)
    {
        if(rotM.r12>-1)
        {
            euAn_Result.e1=atan2(rotM.r32,rotM.r22);
            euAn_Result.e2=asin(-rotM.r12);
            euAn_Result.e3=atan2(rotM.r13,rotM.r11);
        }
        else
        {
            euAn_Result.e1=-atan2(-rotM.r31,rotM.r33);
            euAn_Result.e2=M_PI/2;
            euAn_Result.e3=0;
        }
    }
    else
    {
        euAn_Result.e1=atan2(-rotM.r31,rotM.r33);
        euAn_Result.e2=-M_PI/2;
        euAn_Result.e3=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2YXZiZXYe(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r23<1)
    {
        if(rotM.r23>-1)
        {
            euAn_Result.e1=atan2(rotM.r13,rotM.r33);
            euAn_Result.e2=asin(-rotM.r23);
            euAn_Result.e3=atan2(rotM.r21,rotM.r22);
        }
        else
        {
            euAn_Result.e1=-atan2(-rotM.r12,rotM.r11);
            euAn_Result.e2=M_PI/2;
            euAn_Result.e3=0;
        }
    }
    else
    {
        euAn_Result.e1=atan2(-rotM.r12,rotM.r11);
        euAn_Result.e2=-M_PI/2;
        euAn_Result.e3=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2YZXiXZYe(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r21<1)
    {
        if(rotM.r21>-1)
        {
            euAn_Result.e1=atan2(-rotM.r31,rotM.r11);
            euAn_Result.e2=asin(rotM.r21);
            euAn_Result.e3=atan2(-rotM.r23,rotM.r22);
        }
        else
        {
            euAn_Result.e1=-atan2(rotM.r32,rotM.r33);
            euAn_Result.e2=-M_PI/2;
            euAn_Result.e3=0;
        }
    }
    else
    {
        euAn_Result.e1=atan2(rotM.r32,rotM.r33);
        euAn_Result.e2=M_PI/2;
        euAn_Result.e3=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2ZXYiYXZe(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r32<1)
    {
        if(rotM.r32>-1)
        {
            euAn_Result.e1=atan2(-rotM.r12,rotM.r22);
            euAn_Result.e2=asin(rotM.r32);
            euAn_Result.e3=atan2(-rotM.r31,rotM.r33);
        }
        else
        {
            euAn_Result.e1=-atan2(rotM.r13,rotM.r11);
            euAn_Result.e2=-M_PI/2;
            euAn_Result.e3=0;
        }
    }
    else
    {
        euAn_Result.e1=atan2(rotM.r13,rotM.r11);
        euAn_Result.e2=M_PI/2;
        euAn_Result.e3=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2ZYXiXYZe(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r31<1)
    {
        if(rotM.r31>-1)
        {
            euAn_Result.e1=atan2(rotM.r21,rotM.r11);
            euAn_Result.e2=asin(-rotM.r31);
            euAn_Result.e3=atan2(rotM.r32,rotM.r33);
        }
        else
        {
            euAn_Result.e1=-atan2(-rotM.r23,rotM.r22);
            euAn_Result.e2=M_PI/2;
            euAn_Result.e3=0;
        }
    }
    else
    {
        euAn_Result.e1=atan2(-rotM.r23,rotM.r22);
        euAn_Result.e2=-M_PI/2;
        euAn_Result.e3=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2XYXi(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r11<1)
    {
        if(rotM.r11>-1)
        {
            euAn_Result.e1=atan2(rotM.r21,-rotM.r31);
            euAn_Result.e2=acos(rotM.r11);
            euAn_Result.e3=atan2(rotM.r12,rotM.r13);
        }
        else
        {
            euAn_Result.e1=-atan2(-rotM.r23,rotM.r22);
            euAn_Result.e2=M_PI;
            euAn_Result.e3=0;
        }
    }
    else
    {
        euAn_Result.e1=atan2(-rotM.r23,rotM.r22);
        euAn_Result.e2=0;
        euAn_Result.e3=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2XZXi(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r11<1)
    {
        if(rotM.r11>-1)
        {
            euAn_Result.e1=atan2(rotM.r31,rotM.r21);
            euAn_Result.e2=acos(rotM.r11);
            euAn_Result.e3=atan2(rotM.r13,-rotM.r12);
        }
        else
        {
            euAn_Result.e1=-atan2(rotM.r32,rotM.r33);
            euAn_Result.e2=M_PI;
            euAn_Result.e3=0;
        }
    }
    else
    {
        euAn_Result.e1=atan2(rotM.r32,rotM.r33);
        euAn_Result.e2=0;
        euAn_Result.e3=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2YXYi(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r22<1)
    {
        if(rotM.r22>-1)
        {
            euAn_Result.e1=atan2(rotM.r12,rotM.r32);
            euAn_Result.e2=acos(rotM.r22);
            euAn_Result.e3=atan2(rotM.r21,-rotM.r23);
        }
        else
        {
            euAn_Result.e1=-atan2(rotM.r13,rotM.r11);
            euAn_Result.e2=M_PI;
            euAn_Result.e3=0;
        }
    }
    else
    {
        euAn_Result.e1=atan2(rotM.r13,rotM.r11);
        euAn_Result.e2=0;
        euAn_Result.e3=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2YZYi(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r22<1)
    {
        if(rotM.r22>-1)
        {
            euAn_Result.e1=atan2(rotM.r32,-rotM.r12);
            euAn_Result.e2=acos(rotM.r22);
            euAn_Result.e3=atan2(rotM.r23,rotM.r21);
        }
        else
        {
            euAn_Result.e1=-atan2(-rotM.r31,rotM.r33);
            euAn_Result.e2=M_PI;
            euAn_Result.e3=0;
        }
    }
    else
    {
        euAn_Result.e1=atan2(-rotM.r31,rotM.r33);
        euAn_Result.e2=0;
        euAn_Result.e3=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2ZXZi(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r33<1)
    {
        if(rotM.r33>-1)
        {
            euAn_Result.e1=atan2(rotM.r13,-rotM.r23);
            euAn_Result.e2=acos(rotM.r33);
            euAn_Result.e3=atan2(rotM.r31,rotM.r32);
        }
        else
        {
            euAn_Result.e1=-atan2(-rotM.r12,rotM.r11);
            euAn_Result.e2=M_PI;
            euAn_Result.e3=0;
        }
    }
    else
    {
        euAn_Result.e1=atan2(-rotM.r12,rotM.r11);
        euAn_Result.e2=0;
        euAn_Result.e3=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2ZYZi(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r33<1)
    {
        if(rotM.r33>-1)
        {
            euAn_Result.e1=atan2(rotM.r23,rotM.r13);
            euAn_Result.e2=acos(rotM.r33);
            euAn_Result.e3=atan2(rotM.r32,-rotM.r31);
        }
        else
        {
            euAn_Result.e1=-atan2(rotM.r21,rotM.r22);
            euAn_Result.e2=M_PI;
            euAn_Result.e3=0;
        }
    }
    else
    {
        euAn_Result.e1=atan2(rotM.r21,rotM.r22);
        euAn_Result.e2=0;
        euAn_Result.e3=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2XYXe(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r11<1)
    {
        if(rotM.r11>-1)
        {
            euAn_Result.e3=atan2(rotM.r21,-rotM.r31);
            euAn_Result.e2=acos(rotM.r11);
            euAn_Result.e1=atan2(rotM.r12,rotM.r13);
        }
        else
        {
            euAn_Result.e3=-atan2(-rotM.r23,rotM.r22);
            euAn_Result.e2=M_PI;
            euAn_Result.e1=0;
        }
    }
    else
    {
        euAn_Result.e3=atan2(-rotM.r23,rotM.r22);
        euAn_Result.e2=0;
        euAn_Result.e1=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2XZXe(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r11<1)
    {
        if(rotM.r11>-1)
        {
            euAn_Result.e3=atan2(rotM.r31,rotM.r21);
            euAn_Result.e2=acos(rotM.r11);
            euAn_Result.e1=atan2(rotM.r13,-rotM.r12);
        }
        else
        {
            euAn_Result.e3=-atan2(rotM.r32,rotM.r33);
            euAn_Result.e2=M_PI;
            euAn_Result.e1=0;
        }
    }
    else
    {
        euAn_Result.e3=atan2(rotM.r32,rotM.r33);
        euAn_Result.e2=0;
        euAn_Result.e1=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2YXYe(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r22<1)
    {
        if(rotM.r22>-1)
        {
            euAn_Result.e3=atan2(rotM.r12,rotM.r32);
            euAn_Result.e2=acos(rotM.r22);
            euAn_Result.e1=atan2(rotM.r21,-rotM.r23);
        }
        else
        {
            euAn_Result.e3=-atan2(rotM.r13,rotM.r11);
            euAn_Result.e2=M_PI;
            euAn_Result.e1=0;
        }
    }
    else
    {
        euAn_Result.e3=atan2(rotM.r13,rotM.r11);
        euAn_Result.e2=0;
        euAn_Result.e1=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2YZYe(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r22<1)
    {
        if(rotM.r22>-1)
        {
            euAn_Result.e3=atan2(rotM.r32,-rotM.r12);
            euAn_Result.e2=acos(rotM.r22);
            euAn_Result.e1=atan2(rotM.r23,rotM.r21);
        }
        else
        {
            euAn_Result.e3=-atan2(-rotM.r31,rotM.r33);
            euAn_Result.e2=M_PI;
            euAn_Result.e1=0;
        }
    }
    else
    {
        euAn_Result.e3=atan2(-rotM.r31,rotM.r33);
        euAn_Result.e2=0;
        euAn_Result.e1=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2ZXZe(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r33<1)
    {
        if(rotM.r33>-1)
        {
            euAn_Result.e3=atan2(rotM.r13,-rotM.r23);
            euAn_Result.e2=acos(rotM.r33);
            euAn_Result.e1=atan2(rotM.r31,rotM.r32);
        }
        else
        {
            euAn_Result.e3=-atan2(-rotM.r12,rotM.r11);
            euAn_Result.e2=M_PI;
            euAn_Result.e1=0;
        }
    }
    else
    {
        euAn_Result.e3=atan2(-rotM.r12,rotM.r11);
        euAn_Result.e2=0;
        euAn_Result.e1=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2ZYZe(orc::rotationMatrix rotM)
{
    orc::eulerAngles euAn_Result;
    if(rotM.r33<1)
    {
        if(rotM.r33>-1)
        {
            euAn_Result.e3=atan2(rotM.r23,rotM.r13);
            euAn_Result.e2=acos(rotM.r33);
            euAn_Result.e1=atan2(rotM.r32,-rotM.r31);
        }
        else
        {
            euAn_Result.e3=-atan2(rotM.r21,rotM.r22);
            euAn_Result.e2=M_PI;
            euAn_Result.e1=0;
        }
    }
    else
    {
        euAn_Result.e3=atan2(rotM.r21,rotM.r22);
        euAn_Result.e2=0;
        euAn_Result.e1=0;
    }
    return euAn_Result;
}

orc::eulerAngles orc::rotationMatrix2eulerAngles(orc::rotationMatrix rotM)
{
    switch (ui->eulerAnglesConversionComboBox->currentIndex())
    {
    case 0:
        return orc::rotationMatrix2XYZiZYXe(rotM);
        break;
    case 1:
        return orc::rotationMatrix2XZYiYZXe(rotM);
        break;
    case 2:
        return orc::rotationMatrix2YXZiZXYe(rotM);
        break;
    case 3:
        return orc::rotationMatrix2YZXiXZYe(rotM);
        break;
    case 4:
        return orc::rotationMatrix2ZXYiYXZe(rotM);
        break;
    case 5:
        return orc::rotationMatrix2ZYXiXYZe(rotM);
        break;

    case 7:
        return orc::rotationMatrix2XYXi(rotM);
        break;
    case 8:
        return orc::rotationMatrix2XZXi(rotM);
        break;
    case 9:
        return orc::rotationMatrix2YXYi(rotM);
        break;
    case 10:
        return orc::rotationMatrix2YZYi(rotM);
        break;
    case 11:
        return orc::rotationMatrix2ZXZi(rotM);
        break;
    case 12:
        return orc::rotationMatrix2ZYZi(rotM);
        break;


    case 15:
        return orc::rotationMatrix2ZYXiXYZe(rotM);
        break;
    case 16:
        return orc::rotationMatrix2YZXiXZYe(rotM);
        break;
    case 17:
        return orc::rotationMatrix2ZXYiYXZe(rotM);
        break;
    case 18:
        return orc::rotationMatrix2XZYiYZXe(rotM);
        break;
    case 19:
        return orc::rotationMatrix2YXZiZXYe(rotM);
        break;
    case 20:
        return orc::rotationMatrix2XYZiZYXe(rotM);
        break;

    case 22:
        return orc::rotationMatrix2XYXe(rotM);
        break;
    case 23:
        return orc::rotationMatrix2XZXe(rotM);
        break;
    case 24:
        return orc::rotationMatrix2YXYe(rotM);
        break;
    case 25:
        return orc::rotationMatrix2YZYe(rotM);
        break;
    case 26:
        return orc::rotationMatrix2ZXZe(rotM);
        break;
    case 27:
        return orc::rotationMatrix2ZYZe(rotM);
        break;
    }
    //TODO: WARNING MESSAGE
    orc::eulerAngles euAn_error;
    euAn_error.e1=-1;
    euAn_error.e2=-1;
    euAn_error.e3=-1;
    return euAn_error;
}
//*EULER ANGLES* END

//**FROM ROTATION MATRIX** END
//***MATH CONVERSIONS*** END









//***INTERFACE***
//*Read*
orc::rotationMatrix orc::readRotationMatrix()
{
    orc::rotationMatrix rotM_Read;
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
orc::quaternions orc::readQuaternions()
{
    orc::quaternions quat_Read;
    quat_Read.q1=ui->q1LineEdit->text().toDouble();
    quat_Read.q2=ui->q2LineEdit->text().toDouble();
    quat_Read.q3=ui->q3LineEdit->text().toDouble();
    quat_Read.w=ui->wLineEdit->text().toDouble();
    return quat_Read;
}
orc::angleAxis orc::readAngleAxis()
{
    orc::angleAxis anAx_Read;
    anAx_Read.k1=ui->k1LineEdit->text().toDouble();
    anAx_Read.k2=ui->k2LineEdit->text().toDouble();
    anAx_Read.k3=ui->k3LineEdit->text().toDouble();
    anAx_Read.theta=ui->thetaLineEdit->text().toDouble();
    return anAx_Read;
}
orc::angleAxisWithMagnitude orc::readAngleAxisWithMagnitude()
{
    orc::angleAxisWithMagnitude anAxWithM_Read;
    anAxWithM_Read.rX=ui->RxLineEdit->text().toDouble();
    anAxWithM_Read.rY=ui->RyLineEdit->text().toDouble();
    anAxWithM_Read.rZ=ui->RzLineEdit->text().toDouble();
    return anAxWithM_Read;
}
orc::eulerAngles orc::readEulerAngles()
{
    orc::eulerAngles euAn_Read;
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
//*Read* END

//*Update*
void orc::updateRotationMatrix(orc::rotationMatrix rotM)
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
void orc::updateQuaternions(orc::quaternions quat)
{
    ui->q1LineEdit->setText(QString::number(quat.q1));
    ui->q2LineEdit->setText(QString::number(quat.q2));
    ui->q3LineEdit->setText(QString::number(quat.q3));
    ui->wLineEdit->setText(QString::number(quat.w));
}
void orc::updateAngleAxis(orc::angleAxis anAx)
{
    ui->k1LineEdit->setText(QString::number(anAx.k1));
    ui->k2LineEdit->setText(QString::number(anAx.k2));
    ui->k3LineEdit->setText(QString::number(anAx.k3));
    ui->thetaLineEdit->setText(QString::number(anAx.theta));
}
void orc::updateAngleAxisWithMagnitude(orc::angleAxisWithMagnitude anAxWithM)
{
    ui->RxLineEdit->setText(QString::number(anAxWithM.rX));
    ui->RyLineEdit->setText(QString::number(anAxWithM.rY));
    ui->RzLineEdit->setText(QString::number(anAxWithM.rZ));
}
void orc::updateEulerAngles(orc::eulerAngles euAn)
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
//*Update* END

//Euler Angles END
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
        //TODO: WARNING
        break;
    }
    if(ui->e1LineEdit->text().toDouble()!=0 || ui->e2LineEdit->text().toDouble()!=0 || ui->e3LineEdit->text().toDouble()!=0)
    {
        orc::on_eulerAnglesConvertPushButton_clicked();
        ui->invalidConversionWarningLabel->setText("");
    }
}
//*Euler Angles* END

//*MARGINS*
bool orc::isRotationMatrixOrthogonal(orc::rotationMatrix rotM)
{
    if( abs((rotM.r11*rotM.r11+rotM.r12*rotM.r12+rotM.r13*rotM.r13)-1)<NORMALISE_ORTHOGONAL_LENGTH_MARGIN &&
        abs((rotM.r21*rotM.r21+rotM.r22*rotM.r22+rotM.r23*rotM.r23)-1)<NORMALISE_ORTHOGONAL_LENGTH_MARGIN &&
        abs((rotM.r31*rotM.r31+rotM.r32*rotM.r32+rotM.r33*rotM.r33)-1)<NORMALISE_ORTHOGONAL_LENGTH_MARGIN &&
        abs((rotM.r11*rotM.r11+rotM.r21*rotM.r21+rotM.r31*rotM.r31)-1)<NORMALISE_ORTHOGONAL_LENGTH_MARGIN &&
        abs((rotM.r12*rotM.r12+rotM.r22*rotM.r22+rotM.r32*rotM.r32)-1)<NORMALISE_ORTHOGONAL_LENGTH_MARGIN &&
        abs((rotM.r31*rotM.r31+rotM.r32*rotM.r32+rotM.r33*rotM.r33)-1)<NORMALISE_ORTHOGONAL_LENGTH_MARGIN)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool orc::isQuaternionsNormalised(orc::quaternions quat)
{
    if(abs((quat.q1*quat.q1+quat.q2*quat.q2+quat.q3*quat.q3+quat.w*quat.w)-1)<NORMALISE_ORTHOGONAL_LENGTH_MARGIN)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool orc::isAngleAxisLengthOne(orc::angleAxis anAx)
{
    if(abs((anAx.k1*anAx.k1+anAx.k2*anAx.k2+anAx.k3*anAx.k3)-1)<NORMALISE_ORTHOGONAL_LENGTH_MARGIN)
    {
        return true;
    }
    else
    {
        return false;
    }
}
//*MARGINS* END

//*RAD/DEG*
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
//*RAD/DEG* END

//**CONVERT PUSH BUTTONS**
void orc::on_rotationMatrixConvertPushButton_clicked()
{
    rotM_UI=orc::readRotationMatrix();

    quat_UI=orc::rotationMatrix2quaternions(rotM_UI);
    orc::updateQuaternions(quat_UI);
    anAx_UI=orc::rotationMatrix2angleAxis(rotM_UI);
    orc::updateAngleAxis(anAx_UI);
    anAxWithM_UI=orc::rotationMatrix2angleAxisWithMagnitude(rotM_UI);
    orc::updateAngleAxisWithMagnitude(anAxWithM_UI);
    euAn_UI=orc::rotationMatrix2eulerAngles(rotM_UI);
    orc::updateEulerAngles(euAn_UI);

    if(!orc::isRotationMatrixOrthogonal(rotM_UI))
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
    rotM_UI=orc::quaternions2rotationMatrix(quat_UI);
    orc::updateRotationMatrix(rotM_UI);

    anAx_UI=orc::rotationMatrix2angleAxis(rotM_UI);
    orc::updateAngleAxis(anAx_UI);
    anAxWithM_UI=orc::rotationMatrix2angleAxisWithMagnitude(rotM_UI);
    orc::updateAngleAxisWithMagnitude(anAxWithM_UI);
    euAn_UI=orc::rotationMatrix2eulerAngles(rotM_UI);
    orc::updateEulerAngles(euAn_UI);

    if(!orc::isQuaternionsNormalised(quat_UI))
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
    rotM_UI=orc::angleAxis2rotationMatrix(anAx_UI);
    orc::updateRotationMatrix(rotM_UI);

    quat_UI=orc::rotationMatrix2quaternions(rotM_UI);
    orc::updateQuaternions(quat_UI);
    anAxWithM_UI=orc::rotationMatrix2angleAxisWithMagnitude(rotM_UI);
    orc::updateAngleAxisWithMagnitude(anAxWithM_UI);
    euAn_UI=orc::rotationMatrix2eulerAngles(rotM_UI);
    orc::updateEulerAngles(euAn_UI);

    if(!orc::isAngleAxisLengthOne(anAx_UI))
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

    quat_UI=orc::rotationMatrix2quaternions(rotM_UI);
    orc::updateQuaternions(quat_UI);
    euAn_UI=orc::rotationMatrix2eulerAngles(rotM_UI);
    orc::updateEulerAngles(euAn_UI);

    ui->invalidConversionWarningLabel->setText("");
}

void orc::on_eulerAnglesConvertPushButton_clicked()
{
    euAn_UI=readEulerAngles();
    rotM_UI=orc::eulerAngles2rotationMatrix(euAn_UI);
    orc::updateRotationMatrix(rotM_UI);

    quat_UI=orc::rotationMatrix2quaternions(rotM_UI);
    orc::updateQuaternions(quat_UI);
    anAx_UI=orc::rotationMatrix2angleAxis(rotM_UI);
    orc::updateAngleAxis(anAx_UI);
    anAxWithM_UI=orc::rotationMatrix2angleAxisWithMagnitude(rotM_UI);
    orc::updateAngleAxisWithMagnitude(anAxWithM_UI);

    ui->invalidConversionWarningLabel->setText("");
}
//**CONVERT PUSH BUTTONS** END



//**FILE BROWSE PUSH BUTTONS**
void orc::on_selectInputFilePushButton_clicked()
{
    fileName = QFileDialog::getOpenFileName();

    qDebug() << fileName;

    selectedFile.QFile::setFileName(fileName);

    qDebug() << selectedFile.QFile::fileName();

//    TODO: go through the file
}

void orc::on_selectOutputFilePushButton_clicked()
{

}
//**FILE BROWSE PUSH BUTTONS** END

//**TEMPLATE PUSH BUTTONS**
void orc::on_templatesPushButton_clicked()
{

}

void orc::on_saveTemplatePushButton_clicked()
{

}
//**TEPLATE PUSH BUTTONS** END

//**FILE CONVERT PUSH BUTTON**
void orc::on_fileConvertPushButton_clicked()
{

}
//**FILE CONVERT PUSH BUTTON**







//*CLEAR PUSH BUTTON*
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
//*CLEAR PUSH BUTTON* END

//*HELP PUSH BUTTONS*
void orc::on_quaternionsFromHelpPushButton_clicked()
{

}
void orc::on_quaternionsToHelpPushButton_clicked()
{

}


void orc::on_angleAxisFromHelpPushButton_clicked()
{

}
void orc::on_angleAxisToHelpPushButton_clicked()
{

}


void orc::on_angleAxisWithMagnitudeFromHelpPushButton_clicked()
{

}
void orc::on_angleAxisWithMagnitudeToHelpPushButton_clicked()
{

}


void orc::on_eulerAnglesFromHelpPushButton_clicked()
{

}
void orc::on_eulerAnglesToHelpPushButton_clicked()
{

}
//*HELP PUSH BUTTONS* END

//*EXIT PUSH BUTTONS*
void orc::on_fileTabExitPushButton_clicked()
{
    QApplication::quit();
}

void orc::on_conversionTabExitPushButton_clicked()
{
    QApplication::quit();
}
//*EXIT PUSH BUTTONS* END
//***INTERFACE*** END
