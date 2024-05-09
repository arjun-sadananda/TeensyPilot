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
* 2D Vector Classes
* By Bill Perone (billperone@yahoo.com)
* Original: 9-16-2002
* Revised: 19-11-2003
*          18-12-2003
*          06-06-2004
*
* Copyright 2003, This code is provided "as is" and you can use it freely as long as
* credit is given to Bill Perone in the application it is used in
****************************************/


/*
 *
 * Math Classes used in ArduPilot stripped down to the bare essentials needed for TeensyPilot.
 * - by Arjun Sadananda - 05/2024
 * Quaternion, Matrix3, Vector3, Vector2
 * 
 */

#pragma once
#pragma GCC optimize("O2")

template <typename T>
struct Vector2
{
    T x, y;

    // trivial ctor
    constexpr Vector2<T>()
        : x(0)
        , y(0) {}

    // setting ctor
    constexpr Vector2<T>(const T x0, const T y0)
        : x(x0)
        , y(y0) {}

    // test for equality
    bool operator ==(const Vector2<T> &v) const;

    // test for inequality
    bool operator !=(const Vector2<T> &v) const;

    // negation
    Vector2<T> operator -(void) const;

    // addition
    Vector2<T> operator +(const Vector2<T> &v) const{
        return Vector2<T>(x+v.x, y+v.y);
    }

    // subtraction
    Vector2<T> operator -(const Vector2<T> &v) const{
        return Vector2<T>(x-v.x, y-v.y);
    }

    // uniform scaling
    Vector2<T> operator *(const T num) const{
        return Vector2<T>(x*num, y*num);
    }

    // uniform scaling
    Vector2<T> operator  /(const T num) const{
        return Vector2<T>(x/num, y/num);
    }

    // addition
    Vector2<T> &operator +=(const Vector2<T> &v){
        x+=v.x; y+=v.y;
        return *this;
    }

    // subtraction
    Vector2<T> &operator -=(const Vector2<T> &v){
        x -= v.x; y -= v.y;
        return *this;
    }

    // uniform scaling
    Vector2<T> &operator *=(const T num){
        x*=num; y*=num;
        return *this;
    }

    // uniform scaling
    Vector2<T> &operator /=(const T num){
        x /= num; y /= num;
        return *this;
    }

    // dot product
    T operator *(const Vector2<T> &v) const{
        return x*v.x + y*v.y;
    }

    // dot product (same as above but a more easily understood name)
    T dot(const Vector2<T> &v) const {
        return *this * v;
    }

    // cross product
    T operator %(const Vector2<T> &v) const{
        return x*v.y - y*v.x;
    }


    // allow a vector2 to be used as an array, 0 indexed
    T & operator[](uint8_t i) {
        T *_v = &x;
        return _v[i];
    }

    const T & operator[](uint8_t i) const {
        const T *_v = &x;
        return _v[i];
    }
    
    // zero the vector
    void zero()
    {
        x = y = 0;
    }

    // gets the length of this vector squared
    T length_squared() const{
        return (T)(x*x + y*y);
    }

    // gets the length of this vector
    T length(void) const{
        return sqrt(x*x + y*y);
    }

    // normalizes this vector
    void normalize(){
        *this /= length();
    }

    // returns the normalized vector
    Vector2<T> normalized() const{
        return *this/length();
    }

    // rotate vector by angle in radians
    void rotate(T angle_rad){
        const T cs = cos(angle_rad);
        const T sn = sin(angle_rad);
        T rx = x * cs - y * sn;
        T ry = x * sn + y * cs;
        x = rx;
        y = ry;
    }

    /*
      conversion to/from double
     */
    Vector2<float> tofloat() const {
        return Vector2<float>{float(x),float(y)};
    }
    Vector2<double> todouble() const {
        return Vector2<double>{x,y};
    }
};

typedef Vector2<int16_t>        Vector2i;
typedef Vector2<uint16_t>       Vector2ui;
typedef Vector2<int32_t>        Vector2l;
typedef Vector2<uint32_t>       Vector2ul;
typedef Vector2<float>          Vector2f;
typedef Vector2<double>         Vector2d;


template class Vector2<float>;
template class Vector2<double>;

