/* Copyright (C) 2013-2025, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/)
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 */
/*******************************************************************************
 * File:        point.h
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: point struct and point functions
*******************************************************************************/
#ifndef POINT_H
#define POINT_H

#include <cmath>
#include <string>

namespace bipedlab
{

template <typename T>
struct point2d_t
{
    T x;
    T y;

    point2d_t(void) : x(0), y(0) { }


    point2d_t(T x_, T y_) : x(x_), y(y_) { }

    std::string to_string()
    {
        return std::string("(") + std::to_string(x) + std::string(", ") +
               std::to_string(y) + std::string(")");
    }

    template <typename U>
    point2d_t(const point2d_t<U>& copy) : x(copy.x), y(copy.y) { }

    template <typename U>
    inline double operator* (const point2d_t<U>& rhs)
    {
        return x * rhs.x + y * rhs.y;
    }

    template <typename U>
    inline point2d_t operator+ (const point2d_t<U>& rhs)
    {
        return point2d_t(x + rhs.x, y + rhs.y);
    }

    template <typename U>
    inline point2d_t operator- (const point2d_t<U>& rhs)
    {
        return point2d_t(x - rhs.x, y - rhs.y);
    }

    template <typename U>
    inline point2d_t operator* (U rhs)
    {
        return point2d_t(x * rhs, y * rhs);
    }

    template <typename U>
    inline point2d_t operator/ (U rhs)
    {
        return point2d_t(x / rhs, y / rhs);
    }

    template <typename U>
    inline point2d_t& operator+= (const point2d_t<U>& rhs)
    {
        x = x + rhs.x;
        y = y + rhs.y;
        return *this;
    }

    template <typename U>
    inline point2d_t& operator-= (const point2d_t<U>& rhs)
    {
        x = x - rhs.x;
        y = y - rhs.y;
        return *this;
    }

    template <typename U>
    inline point2d_t& operator*= (U rhs)
    {
        x = x * rhs;
        y = y * rhs;
        return *this;
    }

    template <typename U>
    inline point2d_t& operator/= (U rhs)
    {
        x = x / rhs;
        y = y / rhs;
        return *this;
    }

    template <typename U>
    inline bool operator< (const point2d_t<U>& rhs) const
    {
        return x < rhs.x || (x == rhs.x && y < rhs.y);
    }

    template <typename U>
    inline bool operator> (const point2d_t<U>& rhs) const
    {
        return x > rhs.x || (x == rhs.x && y > rhs.y);
    }

    template <typename U>
    inline bool operator== (const point2d_t<U>& rhs) const
    {
        if (std::is_floating_point<T>::value || std::is_floating_point<U>::value)
        {
            return (std::abs(x - rhs.x) < 0.0001) && (std::abs(y - rhs.y) < 0.0001);
        }
        else
        {
            return (x == rhs.x) && (y == rhs.y);
        }
    }

    template <typename U>
    inline bool operator!= (const point2d_t<U>& rhs) const
    {
      if (std::is_floating_point<T>::value || std::is_floating_point<U>::value)
      {
          return (std::abs(x - rhs.x) >= 0.0001) || (std::abs(y - rhs.y) >= 0.0001);
      }
      else
      {
          return (x != rhs.x) || (y != rhs.y);
      }
    }

    inline point2d_t& operator= (const point2d_t<T>& rhs)
    {
        x = rhs.x;
        y = rhs.y;
        return *this;
    }
};

template <typename T, typename U>
double distance_pts(T x1, T y1, U x2, U y2)
{
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

template <typename T, typename U>
double distance_pts(const point2d_t<T>& p1, const point2d_t<U>& p2)
{
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

template <typename T, typename U>
double squared_distance_pts(T x1, T y1, U x2, U y2)
{
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

template <typename T, typename U>
double squared_distance_pts(const point2d_t<T>& p1, const point2d_t<U>& p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

}

#endif
