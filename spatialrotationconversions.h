#ifndef SPATIALROTATIONCONVERSIONS_H
#define SPATIALROTATIONCONVERSIONS_H

#define NORMALISE_ORTHOGONAL_LENGTH_MARGIN 0.005

#include <math.h>

namespace spatialRotationConversions
{
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

/*TO ROTATION MATRIX*/
rotationMatrix quaternions2rotationMatrix(const quaternions &quat);
rotationMatrix angleAxis2rotationMatrix(const angleAxis &anAx);
angleAxis angleAxisWithMagnitude2angleAxis(const angleAxisWithMagnitude &anAxWithM);
rotationMatrix XYZiZYXe2rotationMatrix(const eulerAngles &euAn);
rotationMatrix XZYiYZXe2rotationMatrix(const eulerAngles &euAn);
rotationMatrix YXZiZXYe2rotationMatrix(const eulerAngles &euAn);
rotationMatrix YZXiXZYe2rotationMatrix(const eulerAngles &euAn);
rotationMatrix ZXYiYXZe2rotationMatrix(const eulerAngles &euAn);
rotationMatrix ZYXiXYZe2rotationMatrix(const eulerAngles &euAn);
rotationMatrix XYXi2rotationMatrix(const eulerAngles &euAn);
rotationMatrix XZXi2rotationMatrix(const eulerAngles &euAn);
rotationMatrix YXYi2rotationMatrix(const eulerAngles &euAn);
rotationMatrix YZYi2rotationMatrix(const eulerAngles &euAn);
rotationMatrix ZXZi2rotationMatrix(const eulerAngles &euAn);
rotationMatrix ZYZi2rotationMatrix(const eulerAngles &euAn);
rotationMatrix XYXe2rotationMatrix(const eulerAngles &euAn);
rotationMatrix XZXe2rotationMatrix(const eulerAngles &euAn);
rotationMatrix YXYe2rotationMatrix(const eulerAngles &euAn);
rotationMatrix YZYe2rotationMatrix(const eulerAngles &euAn);
rotationMatrix ZXZe2rotationMatrix(const eulerAngles &euAn);
rotationMatrix ZYZe2rotationMatrix(const eulerAngles &euAn);

/*FROM ROTATION MATRIX*/
quaternions rotationMatrix2quaternions(const rotationMatrix &rotM);
angleAxis rotationMatrix2angleAxis(const rotationMatrix &rotM);
angleAxisWithMagnitude rotationMatrix2angleAxisWithMagnitude(const rotationMatrix &rotM);
eulerAngles rotationMatrix2XYZiZYXe(const rotationMatrix &rotM);
eulerAngles rotationMatrix2XZYiYZXe(const rotationMatrix &rotM);
eulerAngles rotationMatrix2YXZiZXYe(const rotationMatrix &rotM);
eulerAngles rotationMatrix2YZXiXZYe(const rotationMatrix &rotM);
eulerAngles rotationMatrix2ZXYiYXZe(const rotationMatrix &rotM);
eulerAngles rotationMatrix2ZYXiXYZe(const rotationMatrix &rotM);
eulerAngles rotationMatrix2XYXi(const rotationMatrix &rotM);
eulerAngles rotationMatrix2XZXi(const rotationMatrix &rotM);
eulerAngles rotationMatrix2YXYi(const rotationMatrix &rotM);
eulerAngles rotationMatrix2YZYi(const rotationMatrix &rotM);
eulerAngles rotationMatrix2ZXZi(const rotationMatrix &rotM);
eulerAngles rotationMatrix2ZYZi(const rotationMatrix &rotM);
eulerAngles rotationMatrix2XYXe(const rotationMatrix &rotM);
eulerAngles rotationMatrix2XZXe(const rotationMatrix &rotM);
eulerAngles rotationMatrix2YXYe(const rotationMatrix &rotM);
eulerAngles rotationMatrix2YZYe(const rotationMatrix &rotM);
eulerAngles rotationMatrix2ZXZe(const rotationMatrix &rotM);
eulerAngles rotationMatrix2ZYZe(const rotationMatrix &rotM);

/*MARGINS*/
bool isRotationMatrixOrthogonal(const rotationMatrix &rotM);
bool isQuaternionsNormalised(const quaternions &quat);
bool isAngleAxisLengthOne(const angleAxis &anAx);
}
#endif // SPATIALROTATIONCONVERSIONS_H
