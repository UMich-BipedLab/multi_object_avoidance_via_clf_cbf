/* Copyright (C) 2013-2020, The Regents of The University of Michigan.
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
 *
 * AUTHOR: Bruce JK Huang (bjhuang@umich.edu)
 * WEBSITE: https://www.BrucebotStudio.com/
 */
#pragma once

#ifndef LINE_H
#define LINE_H

#include <cmath>
#include <utility> // pair
#include <eigen3/Eigen/Dense>
#include "float_comparision.h"

namespace bipedlab
{
namespace line
{
    struct line_t
    {
        // ax + by + cz + d = 0
        double a;
        double b;
        double c;
        double d;
    };

    // x + b * y + c = 0
    // b * y1 + c = -x1
    // b * y2 + c = -x2
    //
    // [y1 1; y2 1] * [b; c] = [-x1; -x2]
    // [b; c] = [y1 1; y2 1]^-1 * [-x1; -x2]
    // -> B = A^-1 * C
    line_t compute2DLineGivenTwoPoints(double x1, double y1, double x2, double y2)
    {
        line_t params;
        params.a = 1;
        params.c = 0;

        Eigen::Matrix2d A;
        A << y1, 1, y2, 1;

        Eigen::Vector2d C(-x1, -x2);
        Eigen::Vector2d B = A.inverse() * C;
        params.b = B(0);
        params.d = B(1);

        return params;
    }

    // Given points A and B corresponding to line AB and points C and D corresponding
    // to line CD, find the point of intersection of these lines.
    // return <FLT_MAX, FLT_MAX> if parallel
    template <class T>
    std::pair<T, T>
    computeIntersectionOfTwoLineSegments(
            std::pair<T, T> A,
            std::pair<T, T> B,
            std::pair<T, T> C,
            std::pair<T, T> D
            )
    {
        // Line AB represented as a1x + b1y = c1
        T a1 = B.second - A.second;
        T b1 = A.first - B.first;
        T c1 = a1*(A.first) + b1*(A.second);

        // Line CD represented as a2x + b2y = c2
        T a2 = D.second - C.second;
        T b2 = C.first - D.first;
        T c2 = a2*(C.first)+ b2*(C.second);

        T determinant = a1*b2 - a2*b1;

        if (lazy_equal::absolute_fuzzy_equal(determinant, 0.0))
        {
            // The lines are parallel. This is simplified
            // by returning a pair of FLT_MAX
            return std::make_pair(FLT_MAX, FLT_MAX);
        }
        else
        {
            T x = (b2*c1 - b1*c2)/determinant;
            T y = (a1*c2 - a2*c1)/determinant;
            return std::make_pair(x, y);
        }
    }

    std::pair<double, double>
    computeIntersectionOfTwo2Dlines(line_t l1, line_t l2)
    {
    }


} /* line */

} /* bipedlab */



#endif /* ifndef LINE_H */
