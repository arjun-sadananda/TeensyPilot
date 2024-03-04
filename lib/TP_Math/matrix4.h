/*
 * Made for TeensyPilot quaternion EKF
 */

#pragma once

#include "ftype.h"

#include "vector3.h"
#include "vector2.h"

template <typename T>
class Vector3;

// 3x3 matrix with elements of type T
template <typename T>
class Matrix4 {
public:

    // Vectors comprising the rows of the matrix
    T mat[4][4];

    // trivial ctor
    // note that the Vector3 ctor will zero the vector elements
    constexpr Matrix4<T>() {}

    
    // setting ctor
    constexpr Matrix4<T>(const T a00, const T a01, const T a02, const T a03,
                         const T a10, const T a11, const T a12, const T a13,
                         const T a20, const T a21, const T a22, const T a23,
                         const T a30, const T a31, const T a32, const T a33){
                            mat[0][0] = a00; mat[0][1] = a01; mat[0][2] = a02; mat[0][3] = a03;
                            mat[1][0] = a10; mat[1][1] = a11; mat[1][2] = a12; mat[1][3] = a13;
                            mat[2][0] = a20; mat[2][1] = a21; mat[2][2] = a22; mat[2][3] = a23;
                            mat[3][0] = a30; mat[3][1] = a31; mat[3][2] = a32; mat[3][3] = a33;
                         }

    // negation
    // Matrix4<T> operator        - (void) const
    // {
    //     return Matrix3<T>(-a,-b,-c);
    // }

    // addition
    // Matrix4<T> operator        + (const Matrix4<T> &m) const
    // {
    //     return Matrix4<T>(mat[0][0]+m[0][0], );
    // }
    // Matrix4<T> &operator        += (const Matrix4<T> &m)
    // {
    //     return *this = *this + m;
    // }

    // // subtraction
    // Matrix4<T> operator        - (const Matrix4<T> &m) const
    // {
    //     return Matrix3<T>(a-m.a, b-m.b, c-m.c);
    // }
    // Matrix4<T> &operator        -= (const Matrix4<T> &m)
    // {
    //     return *this = *this - m;
    // }

    // uniform scaling
    // Matrix4<T> operator        * (const T num) const
    // {
    //     return Matrix3<T>(a*num, b*num, c*num);
    // }
    // Matrix4<T> &operator        *= (const T num)
    // {
    //     return *this = *this * num;
    // }
    // Matrix4<T> operator        / (const T num) const
    // {
    //     return Matrix3<T>(a/num, b/num, c/num);
    // }
    // Matrix4<T> &operator        /= (const T num)
    // {
    //     return *this = *this / num;
    // }

    // multiplication by another Matrix3<T>
    // Matrix4<T> operator *(const Matrix4<T> &m) const;

    // Matrix4<T> &operator        *=(const Matrix4<T> &m)
    // {
    //     return *this = *this * m;
    // }

    // transpose the matrix
    // Matrix4<T>          transposed(void) const;

    // void transpose(void)
    // {
    //     *this = transposed();
    // }

    /**
     * Calculate the determinant of this matrix.
     *
     * @return The value of the determinant.
     */
    // T det() const;

    /**
     * Calculate the inverse of this matrix.
     *
     * @param inv[in] Where to store the result.
     *
     * @return If this matrix is invertible, then true is returned. Otherwise,
     * \p inv is unmodified and false is returned.
     */
    // bool inverse(Matrix3<T>& inv) const;// WARN_IF_UNUSED;

    /**
     * Invert this matrix if it is invertible.
     *
     * @return Return true if this matrix could be successfully inverted and
     * false otherwise.
     */
    // bool invert();// WARN_IF_UNUSED;

    // zero the matrix
    void        zero(void);

    // setup the identity matrix
    void        identity(void) {
        zero();
        mat[0][0] = mat[1][1] = mat[2][2] = mat[3][3] = 1;
    }

    // 
    void from_omega_dt(const Vector3<T>& angular_velocity, float dt);

    // multiplication by a quaternion
    Quaternion operator *(const Quaternion &v) const;

    void process_noise_cov_mat(const Quaternion &q, const float std_dev, float Ts);

};

typedef Matrix4<int16_t>                Matrix4i;
typedef Matrix4<uint16_t>               Matrix4ui;
typedef Matrix4<int32_t>                Matrix4l;
typedef Matrix4<uint32_t>               Matrix4ul;
typedef Matrix4<float>                  Matrix4f;
typedef Matrix4<double>                 Matrix4d;
