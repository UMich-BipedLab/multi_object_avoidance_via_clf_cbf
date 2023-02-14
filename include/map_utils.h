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
 * File:        map_utils.h
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: useful map functions
*******************************************************************************/
#ifndef MAP_UTILS_H
#define MAP_UTILS_H

#include "point.h"
#include <string>
#include <vector>
#include <iostream>

namespace bipedlab
{

//generate a list of points in the neighbors
void getNeighborPoints(point2d_t<int> p, std::vector<point2d_t<int>>& neighbor,
                       int min_x, int max_x, int min_y, int max_y,
                       int mode, double radius);

//compare the distance of points to the origin point
bool compare_dis(const point2d_t<int>& a, const point2d_t<int>& b);

//get the convex hull of a list of points
std::vector<point2d_t<int>> get_convex_hull(const std::vector<point2d_t<int>>& hull);

//calculate the convex area of a convex hull
double convex_area(const std::vector<point2d_t<int>>& hull);

//calculate the approximate boundary area of a convex hull
double boundary_area(const std::vector<point2d_t<int>>& hull);

//calculate the minimal circle boundary of a convex hull
std::vector<double> get_circle_bound(const std::vector<point2d_t<double>>& hull);

}

#endif
