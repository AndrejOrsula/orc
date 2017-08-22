#include "spatialrotationconversions.h"

/*TO ROTATION MATRIX*/
spatialRotationConversions::rotationMatrix spatialRotationConversions::quaternions2rotationMatrix(const spatialRotationConversions::quaternions &quat)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::angleAxis2rotationMatrix(const spatialRotationConversions::angleAxis &anAx)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::angleAxis spatialRotationConversions::angleAxisWithMagnitude2angleAxis(const spatialRotationConversions::angleAxisWithMagnitude &anAxWithM)
{
    spatialRotationConversions::angleAxis anAx_Result;
    anAx_Result.theta=sqrt(anAxWithM.rX*anAxWithM.rX+anAxWithM.rY*anAxWithM.rY+anAxWithM.rZ*anAxWithM.rZ);
    anAx_Result.k1=anAxWithM.rX/anAx_Result.theta;
    anAx_Result.k2=anAxWithM.rY/anAx_Result.theta;
    anAx_Result.k3=anAxWithM.rZ/anAx_Result.theta;
    return anAx_Result;
}

spatialRotationConversions::rotationMatrix spatialRotationConversions::XYZiZYXe2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::XZYiYZXe2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::YXZiZXYe2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::YZXiXZYe2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::ZXYiYXZe2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::ZYXiXYZe2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn) //roll-pitch-yaw (extrinsic)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::XYXi2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::XZXi2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::YXYi2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::YZYi2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::ZXZi2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::ZYZi2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::XYXe2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::XZXe2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::YXYe2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::YZYe2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::ZXZe2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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

spatialRotationConversions::rotationMatrix spatialRotationConversions::ZYZe2rotationMatrix(const spatialRotationConversions::eulerAngles &euAn)
{
    spatialRotationConversions::rotationMatrix rotM_Result;
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


/*FROM ROTATION MATRIX*/
spatialRotationConversions::quaternions spatialRotationConversions::rotationMatrix2quaternions(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::quaternions quat_Result;
    quat_Result.w=0.5*sqrt(1+rotM.r11+rotM.r22+rotM.r33);
    quat_Result.q1=(rotM.r32-rotM.r23)/(4*quat_Result.w);
    quat_Result.q2=(rotM.r13-rotM.r31)/(4*quat_Result.w);
    quat_Result.q3=(rotM.r21-rotM.r12)/(4*quat_Result.w);
    return quat_Result;
}

spatialRotationConversions::angleAxis spatialRotationConversions::rotationMatrix2angleAxis(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::angleAxis anAx_Result;
    anAx_Result.theta=acos(0.5*(rotM.r11+rotM.r22+rotM.r33-1));
    anAx_Result.k1=(rotM.r32-rotM.r23)/(2*sin(anAx_Result.theta));
    anAx_Result.k2=(rotM.r13-rotM.r31)/(2*sin(anAx_Result.theta));
    anAx_Result.k3=(rotM.r21-rotM.r12)/(2*sin(anAx_Result.theta));
    return anAx_Result;
}

spatialRotationConversions::angleAxisWithMagnitude spatialRotationConversions::rotationMatrix2angleAxisWithMagnitude(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::angleAxisWithMagnitude anAxWithM_Result;
    spatialRotationConversions::angleAxis anAx_Result=spatialRotationConversions::rotationMatrix2angleAxis(rotM);
    anAxWithM_Result.rX=anAx_Result.k1*anAx_Result.theta;
    anAxWithM_Result.rY=anAx_Result.k2*anAx_Result.theta;
    anAxWithM_Result.rZ=anAx_Result.k3*anAx_Result.theta;
    return anAxWithM_Result;
}

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2XYZiZYXe(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2XZYiYZXe(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2YXZiZXYe(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2YZXiXZYe(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2ZXYiYXZe(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2ZYXiXYZe(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2XYXi(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2XZXi(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2YXYi(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2YZYi(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2ZXZi(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2ZYZi(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2XYXe(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2XZXe(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2YXYe(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2YZYe(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2ZXZe(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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

spatialRotationConversions::eulerAngles spatialRotationConversions::rotationMatrix2ZYZe(const spatialRotationConversions::rotationMatrix &rotM)
{
    spatialRotationConversions::eulerAngles euAn_Result;
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


/*MARGINS*/
bool spatialRotationConversions::isRotationMatrixOrthogonal(const spatialRotationConversions::rotationMatrix &rotM)
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

bool spatialRotationConversions::isQuaternionsNormalised(const spatialRotationConversions::quaternions &quat)
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

bool spatialRotationConversions::isAngleAxisLengthOne(const spatialRotationConversions::angleAxis &anAx)
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
