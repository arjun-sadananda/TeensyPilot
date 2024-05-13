/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Copyright 2010 Michael Smith, all rights reserved.

// Derived closely from:
/****************************************
* 3D Vector Classes
* By Bill Perone (billperone@yahoo.com)
* Original: 9-16-2002
* Revised: 19-11-2003
*          11-12-2003
*          18-12-2003
*          06-06-2004
*
* Copyright 2003, This code is provided "as is" and you can use it freely as long as
* credit is given to Bill Perone in the application it is used in
*
* Notes:
* if a*b = 0 then a & b are orthogonal
* a%b = -b%a
* a*(b%c) = (a%b)*c
* a%b = a(cast to matrix)*b
* (a%b).length() = area of parallelogram formed by a & b
* (a%b).length() = a.length()*b.length() * sin(angle between a & b)
* (a%b).length() = 0 if angle between a & b = 0 or a.length() = 0 or b.length() = 0
* a * (b%c) = volume of parallelpiped formed by a, b, c
* vector triple product: a%(b%c) = b*(a*c) - c*(a*b)
* scalar triple product: a*(b%c) = c*(a%b) = b*(c%a)
* vector quadruple product: (a%b)*(c%d) = (a*c)*(b*d) - (a*d)*(b*c)
* if a is unit vector along b then a%b = -b%a = -b(cast to matrix)*a = 0
* vectors a1...an are linearly dependent if there exists a vector of scalars (b) where a1*b1 + ... + an*bn = 0
*           or if the matrix (A) * b = 0
*
****************************************/

/*
 *
 * Math Classes used in ArduPilot stripped down to the bare minimum needed for TeensyPilot.
 * - by Arjun Sadananda - 05/2024
 * Quaternion, Matrix3, Vector3, Vector2
 * 
 */


#pragma once
#pragma GCC optimize("O2")

#include <float.h>      // FLT_EPSILON

#include "rotations.h"
#include "vector2.h"
#include "matrix3.h"


template <typename T>
class Matrix3;

template <typename T>
class Vector2;

template <typename T>
class Vector3
{

public:
    T        x, y, z;

    // trivial ctor
    constexpr Vector3<T>()
        : x(0)
        , y(0)
        , z(0) {}

    // setting ctor
    constexpr Vector3<T>(const T x0, const T y0, const T z0)
        : x(x0)
        , y(y0)
        , z(z0) {}

    //Create a Vector3 from a Vector2 with z
    constexpr Vector3<T>(const Vector2<T> &v0, const T z0)
        : x(v0.x)
        , y(v0.y)
        , z(z0) {}

    void set(const T x1, const T y1, const T z1){
        x = x1;
        y = y1;
        z = z1;
    }
    // // test for equality
    // bool operator ==(const Vector3<T> &v) const;

    // // test for inequality
    // bool operator !=(const Vector3<T> &v) const;

    // negation
    Vector3<T> operator -() const{
        return Vector3<T>(-x,-y,-z);
    }

    // addition
    Vector3<T> operator +(const Vector3<T> &v) const{
        return Vector3<T>(x+v.x, y+v.y, z+v.z);
    }

    // subtraction
    Vector3<T> operator -(const Vector3<T> &v) const{
        return Vector3<T>(x-v.x, y-v.y, z-v.z);
    }

    // uniform scaling
    Vector3<T> operator *(const T num) const{
        return Vector3<T>(x*num, y*num, z*num);
    }

    // uniform scaling
    Vector3<T> operator  /(const T num) const{
        return Vector3<T>(x/num, y/num, z/num);
    }

    // addition
    Vector3<T> &operator +=(const Vector3<T> &v){
        x+=v.x; y+=v.y; z+=v.z;
        return *this;
    }

    // subtraction
    Vector3<T> &operator -=(const Vector3<T> &v){
        x -= v.x; y -= v.y; z -= v.z;
        return *this;
    }

    // uniform scaling
    Vector3<T> &operator *=(const T num){
        x*=num; y*=num; z*=num;
        return *this;
    }

    // uniform scaling
    Vector3<T> &operator /=(const T num){
        x /= num; y /= num; z /= num;
        return *this;
    }

    // non-uniform scaling
    Vector3<T> &operator *=(const Vector3<T> &v) {
        x *= v.x; y *= v.y; z *= v.z;
        return *this;
    }

    // allow a vector3 to be used as an array, 0 indexed
    T & operator[](uint8_t i) {
        T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    const T & operator[](uint8_t i) const {
        const T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 3);
#endif
        return _v[i];
    }

    // dot product
    T operator *(const Vector3<T> &v) const {
        return x*v.x + y*v.y + z*v.z;
    }

    // dot product for Lua
    T dot(const Vector3<T> &v) const {
        return *this * v;
    }
    
    // multiply a row vector by a matrix, to give a row vector
    Vector3<T> row_times_mat(const Matrix3<T> &m) const{
        return Vector3<T>(*this * m.colx(),
                        *this * m.coly(),
                        *this * m.colz());
    }

    // cross product
    Vector3<T> operator %(const Vector3<T> &v) const {
        Vector3<T> temp(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
        return temp;
    }

    // cross product for Lua
    Vector3<T> cross(const Vector3<T> &v) const {
        return *this % v;
    }

    // check if all elements are zero
    bool is_zero(void) const{//} WARN_IF_UNUSED {
        return x == 0 && y == 0 && z == 0;
    }

    // scale a vector3
    Vector3<T> scale(const T v) const {
        return *this * v;
    }

    // rotate by a standard rotation
    void rotate(enum Rotation rotation){
        T tmp;
        switch (rotation) {
        case ROTATION_NONE:
            return;
        case ROTATION_YAW_45: {
            tmp = HALF_SQRT_2*(T)(x - y);
            y   = HALF_SQRT_2*(T)(x + y);
            x = tmp;
            return;
        }
        case ROTATION_YAW_90: {
            tmp = x; x = -y; y = tmp;
            return;
        }
        case ROTATION_YAW_135: {
            tmp = -HALF_SQRT_2*(T)(x + y);
            y   =  HALF_SQRT_2*(T)(x - y);
            x = tmp;
            return;
        }
        case ROTATION_YAW_180:
            x = -x; y = -y;
            return;
        case ROTATION_YAW_225: {
            tmp = HALF_SQRT_2*(T)(y - x);
            y   = -HALF_SQRT_2*(T)(x + y);
            x = tmp;
            return;
        }
        case ROTATION_YAW_270: {
            tmp = x; x = y; y = -tmp;
            return;
        }
        case ROTATION_YAW_315: {
            tmp = HALF_SQRT_2*(T)(x + y);
            y   = HALF_SQRT_2*(T)(y - x);
            x = tmp;
            return;
        }
        case ROTATION_ROLL_180: {
            y = -y; z = -z;
            return;
        }
        case ROTATION_ROLL_180_YAW_45: {
            tmp = HALF_SQRT_2*(T)(x + y);
            y   = HALF_SQRT_2*(T)(x - y);
            x = tmp; z = -z;
            return;
        }
        case ROTATION_ROLL_180_YAW_90:
        case ROTATION_PITCH_180_YAW_270: {
            tmp = x; x = y; y = tmp; z = -z;
            return;
        }
        case ROTATION_ROLL_180_YAW_135: {
            tmp = HALF_SQRT_2*(T)(y - x);
            y   = HALF_SQRT_2*(T)(y + x);
            x = tmp; z = -z;
            return;
        }
        case ROTATION_PITCH_180: {
            x = -x; z = -z;
            return;
        }
        case ROTATION_ROLL_180_YAW_225: {
            tmp = -HALF_SQRT_2*(T)(x + y);
            y   =  HALF_SQRT_2*(T)(y - x);
            x = tmp; z = -z;
            return;
        }
        case ROTATION_ROLL_180_YAW_270: 
        case ROTATION_PITCH_180_YAW_90: {
            tmp = x; x = -y; y = -tmp; z = -z;
            return;
        }
        case ROTATION_ROLL_180_YAW_315: {
            tmp =  HALF_SQRT_2*(T)(x - y);
            y   = -HALF_SQRT_2*(T)(x + y);
            x = tmp; z = -z;
            return;
        }
        case ROTATION_ROLL_90: {
            tmp = z; z = y; y = -tmp;
            return;
        }
        case ROTATION_ROLL_90_YAW_45: {
            tmp = z; z = y; y = -tmp;
            tmp = HALF_SQRT_2*(T)(x - y);
            y   = HALF_SQRT_2*(T)(x + y);
            x = tmp;
            return;
        }
        case ROTATION_ROLL_90_YAW_90: {
            tmp = z; z = y; y = -tmp;
            tmp = x; x = -y; y = tmp;
            return;
        }
        case ROTATION_ROLL_90_YAW_135: {
            tmp = z; z = y; y = -tmp;
            tmp = -HALF_SQRT_2*(T)(x + y);
            y   =  HALF_SQRT_2*(T)(x - y);
            x = tmp;
            return;
        }
        case ROTATION_ROLL_270: {
            tmp = z; z = -y; y = tmp;
            return;
        }
        case ROTATION_ROLL_270_YAW_45: {
            tmp = z; z = -y; y = tmp;
            tmp = HALF_SQRT_2*(T)(x - y);
            y   = HALF_SQRT_2*(T)(x + y);
            x = tmp;
            return;
        }
        case ROTATION_ROLL_270_YAW_90: {
            tmp = z; z = -y; y = tmp;
            tmp = x; x = -y; y = tmp;
            return;
        }
        case ROTATION_ROLL_270_YAW_135: {
            tmp = z; z = -y; y = tmp;
            tmp = -HALF_SQRT_2*(T)(x + y);
            y   =  HALF_SQRT_2*(T)(x - y);
            x = tmp;
            return;
        }
        case ROTATION_PITCH_90: {
            tmp = z; z = -x; x = tmp;
            return;
        }
        case ROTATION_PITCH_270: {
            tmp = z; z = x; x = -tmp;
            return;
        }
        case ROTATION_ROLL_90_PITCH_90: {
            tmp = z; z = y; y = -tmp;
            tmp = z; z = -x; x = tmp;
            return;
        }
        case ROTATION_ROLL_180_PITCH_90: {
            y = -y; z = -z;
            tmp = z; z = -x; x = tmp;
            return;
        }
        case ROTATION_ROLL_270_PITCH_90: {
            tmp = z; z = -y; y = tmp;
            tmp = z; z = -x; x = tmp;
            return;
        }
        case ROTATION_ROLL_90_PITCH_180: {
            tmp = z; z = y; y = -tmp;
            x = -x; z = -z;
            return;
        }
        case ROTATION_ROLL_270_PITCH_180: {
            tmp = z; z = -y; y = tmp;
            x = -x; z = -z;
            return;
        }
        case ROTATION_ROLL_90_PITCH_270: {
            tmp = z; z = y; y = -tmp;
            tmp = z; z = x; x = -tmp;
            return;
        }
        case ROTATION_ROLL_180_PITCH_270: {
            y = -y; z = -z;
            tmp = z; z = x; x = -tmp;
            return;
        }
        case ROTATION_ROLL_270_PITCH_270: {
            tmp = z; z = -y; y = tmp;
            tmp = z; z = x; x = -tmp;
            return;
        }
        case ROTATION_ROLL_90_PITCH_180_YAW_90: {
            tmp = z; z = y; y = -tmp;
            x = -x; z = -z;
            tmp = x; x = -y; y = tmp;
            return;
        }
        case ROTATION_ROLL_90_YAW_270: {
            tmp = z; z = y; y = -tmp;
            tmp = x; x = y; y = -tmp;
            return;
        }
        case ROTATION_ROLL_90_PITCH_68_YAW_293: {
            T tmpx = x;
            T tmpy = y;
            T tmpz = z;
            x =  0.14303897231223747232853327204793 * tmpx +  0.36877648650320382639478111741482 * tmpy + -0.91844638134308709265241077446262 * tmpz;
            y = -0.33213277779664740485543461545603 * tmpx + -0.85628942146641884303193137384369 * tmpy + -0.39554550256296522325882847326284 * tmpz;
            z = -0.93232380121551217122544130688766 * tmpx +  0.36162457008209242248497616856184 * tmpy +  0.00000000000000002214311861220361 * tmpz;
            return;
        }
        case ROTATION_PITCH_315: {
            tmp = HALF_SQRT_2*(T)(x - z);
            z   = HALF_SQRT_2*(T)(x + z);
            x = tmp;
            return;
        }
        case ROTATION_ROLL_90_PITCH_315: {
            tmp = z; z = y; y = -tmp;
            tmp = HALF_SQRT_2*(T)(x - z);
            z   = HALF_SQRT_2*(T)(x + z);
            x = tmp;
            return;
        }
        case ROTATION_PITCH_7: {
            const T sin_pitch = 0.1218693434051474899781908334262; // sinF(pitch);
            const T cos_pitch = 0.99254615164132198312785249072476; // cosF(pitch);
            T tmpx = x;
            T tmpz = z;
            x =  cos_pitch * tmpx + sin_pitch * tmpz;
            z = -sin_pitch * tmpx + cos_pitch * tmpz;
            return;
        }
        case ROTATION_ROLL_45: {
            tmp = HALF_SQRT_2*(T)(y - z);
            z   = HALF_SQRT_2*(T)(y + z);
            y = tmp;
            return;
        }
        case ROTATION_ROLL_315: {
            tmp = HALF_SQRT_2*(T)(y + z);
            z   = HALF_SQRT_2*(T)(z - y);
            y = tmp;
            return;
        }
        case ROTATION_CUSTOM_1:
        case ROTATION_CUSTOM_2:
    // #if !APM_BUILD_TYPE(APM_BUILD_AP_Periph)
    //         // Do not support custom rotations on Periph
    //         AP::custom_rotations().rotate(rotation, *this);
    //         return;
    // #endif
        case ROTATION_MAX:
        case ROTATION_CUSTOM_OLD:
        case ROTATION_CUSTOM_END:
            break;
        }
        // rotation invalid
        // INTERNAL_ERROR(AP_InternalError::error_t::bad_rotation);
    }

    void rotate_inverse(enum Rotation rotation){
        Vector3<T> x_vec(1.0f,0.0f,0.0f);
        Vector3<T> y_vec(0.0f,1.0f,0.0f);
        Vector3<T> z_vec(0.0f,0.0f,1.0f);

        x_vec.rotate(rotation);
        y_vec.rotate(rotation);
        z_vec.rotate(rotation);

        Matrix3<T> M(
            x_vec.x, y_vec.x, z_vec.x,
            x_vec.y, y_vec.y, z_vec.y,
            x_vec.z, y_vec.z, z_vec.z
        );

        (*this) = M.mul_transpose(*this);
    }

    // gets the length of this vector squared
    T  length_squared() const
    {
        return (T)(*this * *this);
    }

    // gets the length of this vector
    T length(void) const {
        return sqrt(x*x+y*y+z*z);
    }

    // normalizes this vector
    void normalize()
    {
        *this /= length();
    }

    // zero the vector
    void zero()
    {
        x = y = z = 0;
    }

    // returns the normalized version of this vector
    Vector3<T> normalized() const
    {
        if(length()!=0)
            return *this/length();
        else
            return *this;
    }
    /*
      conversion to/from double
     */
    Vector3<float> tofloat() const {
        return Vector3<float>{float(x),float(y),float(z)};
    }
    Vector3<double> todouble() const {
        return Vector3<double>{x,y,z};
    }
};

typedef Vector3<int16_t>                Vector3i;
typedef Vector3<uint16_t>               Vector3ui;
typedef Vector3<int32_t>                Vector3l;
typedef Vector3<uint32_t>               Vector3ul;
typedef Vector3<float>                  Vector3f;
typedef Vector3<double>                 Vector3d;


template class Vector3<float>;
template class Vector3<double>;


// template Vector3<int32_t> &Vector3<int32_t>::operator +=(const Vector3<int32_t> &v);
// template bool Vector3<int16_t>::operator ==(const Vector3<int16_t> &v) const;