/*
 * Made for TeensyPilot quaternion EKF
 */

#pragma GCC optimize("O2")

#include "AP_Math.h"

// multiplication by another Matrix3<T>
// template <typename T>
// Matrix4<T> Matrix4<T>::operator *(const Matrix4<T> &M) const
// {
//     // Matrix4<T> temp (a.x * m.a.x + a.y * m.b.x + a.z * m.c.x, a.x * m.a.y + a.y * m.b.y + a.z * m.c.y, a.x * m.a.z + a.y * m.b.z + a.z * m.c.z),
//     //                  b.x * m.a.x + b.y * m.b.x + b.z * m.c.x, b.x * m.a.y + b.y * m.b.y + b.z * m.c.y, b.x * m.a.z + b.y * m.b.z + b.z * m.c.z),
//     //                  Vector3<T>(c.x * m.a.x + c.y * m.b.x + c.z * m.c.x,
//     //                             c.x * m.a.y + c.y * m.b.y + c.z * m.c.y,
//     //                             c.x * m.a.z + c.y * m.b.z + c.z * m.c.z));
//     return temp;
// }

// template <typename T>
// Matrix3<T> Matrix3<T>::transposed(void) const
// {
//     return Matrix3<T>(Vector3<T>(a.x, b.x, c.x),
//                       Vector3<T>(a.y, b.y, c.y),
//                       Vector3<T>(a.z, b.z, c.z));
// }

// template <typename T>
// T Matrix3<T>::det() const
// {
//     return a.x * (b.y * c.z - b.z * c.y) +
//            a.y * (b.z * c.x - b.x * c.z) +
//            a.z * (b.x * c.y - b.y * c.x);
// }

// template <typename T>
// bool Matrix3<T>::inverse(Matrix3<T>& inv) const
// {
//     const T d = det();

//     if (is_zero(d)) {
//         return false;
//     }

//     inv.a.x = (b.y * c.z - c.y * b.z) / d;
//     inv.a.y = (a.z * c.y - a.y * c.z) / d;
//     inv.a.z = (a.y * b.z - a.z * b.y) / d;
//     inv.b.x = (b.z * c.x - b.x * c.z) / d;
//     inv.b.y = (a.x * c.z - a.z * c.x) / d;
//     inv.b.z = (b.x * a.z - a.x * b.z) / d;
//     inv.c.x = (b.x * c.y - c.x * b.y) / d;
//     inv.c.y = (c.x * a.y - a.x * c.y) / d;
//     inv.c.z = (a.x * b.y - b.x * a.y) / d;

//     return true;
// }

// template <typename T>
// bool Matrix3<T>::invert()
// {
//     Matrix3<T> inv;
//     bool success = inverse(inv);
//     if (success) {
//         *this = inv;
//     }
//     return success;
// }

template <typename T>
void Matrix4<T>::zero(void)
{
    mat[0][0] = mat[0][1] = mat[0][2] = mat[0][3] = 0;
    mat[1][0] = mat[1][1] = mat[1][2] = mat[1][3] = 0;
    mat[2][0] = mat[2][1] = mat[2][2] = mat[2][3] = 0;
    mat[3][0] = mat[3][1] = mat[3][2] = mat[3][3] = 0;
}

// create rotation matrix for rotation about the vector v by angle theta
// See: http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
template <typename T>
void Matrix4<T>::from_omega_dt(const Vector3<T>& omega, float dt){
    static Vector3<T> w;
    w = omega*0.5f*dt;   //sqrt(omega.length_squared()+1);
    mat[0][0] = 1;        mat[0][1] = -w[0];      mat[0][2] = -w[1];     mat[0][3] = -w[2];
    mat[1][0] = w[0];     mat[1][1] =  1;         mat[1][2] =  w[2];     mat[1][3] = -w[1];
    mat[2][0] = w[1];     mat[2][1] = -w[2];      mat[2][2] =  1;        mat[2][3] =  w[0];
    mat[3][0] = w[2];     mat[3][1] =  w[1];      mat[3][2] = -w[0];     mat[3][3] =  1;
}

template <typename T>
Quaternion Matrix4<T>::operator         *(const Quaternion &q) const{
    return Quaternion(  mat[0][0]*q.q1      + mat[0][1]*q.q2        + mat[0][2]*q.q3       + mat[0][3]*q.q4,
                        mat[1][0]*q.q1      + mat[1][1]*q.q2        + mat[1][2]*q.q3       + mat[1][3]*q.q4,
                        mat[2][0]*q.q1      + mat[2][1]*q.q2        + mat[2][2]*q.q3       + mat[2][3]*q.q4,
                        mat[3][0]*q.q1      + mat[3][1]*q.q2        + mat[3][2]*q.q3       + mat[3][3]*q.q4);
}

template <typename T>
void Matrix4<T>::process_noise_cov_mat(const Quaternion &q, const float std_dev, float Ts){
//W_t = [-q_v^T; W_t2]
    static float q_w, k;
    static Vector3f q_v;

    // static Matrix3f W_t_square;
    // const Matrix3f eye3(1,0,0,0,1,0,0,0,1);
    q_w = q.q1;
    q_v.set(q.q2, q.q3, q.q4);

    k = AP_sq(std_dev*Ts)/4.0f;
    // W_t_square.skew_from_vector(q_v);
    // W_t_square += q_w*eye3;

    // col = W_t_square*(-q_v);
    // W_t_square*=W_t_square;

    mat[0][0] = k * q_v.length_squared();
    mat[1][0] = mat[0][1] = -k * q_v.x * q_w;
    mat[2][0] = mat[0][2] = -k * q_v.y * q_w;
    mat[3][0] = mat[0][3] = -k * q_v.z * q_w;

    mat[1][1] = k * (AP_sq(q_w)+AP_sq(q_v.y)+AP_sq(q_v.z));
    mat[2][2] = k * (AP_sq(q_w)+AP_sq(q_v.z)+AP_sq(q_v.x));
    mat[3][3] = k * (AP_sq(q_w)+AP_sq(q_v.x)+AP_sq(q_v.y));

    mat[1][2] = mat[2][1] = -k * q_v.x * q_v.y;
    mat[1][3] = mat[3][1] = -k * q_v.z * q_v.x;
    mat[2][3] = mat[3][2] = -k * q_v.y * q_v.z;
}
// define for float and double
template class Matrix4<float>;
template class Matrix4<double>;
