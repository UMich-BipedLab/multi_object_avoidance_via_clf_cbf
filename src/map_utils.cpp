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
 * File:        map_utils.cpp
 *
 * Author:      Jinze Liu (jzliu[at]umich.edu)
 * Created:     10/11/2022
 *
 * Description: useful map functions
*******************************************************************************/
#include "map_utils.h"

namespace bipedlab
{

void getNeighborPoints(point2d_t<int> p, std::vector<point2d_t<int>>& neighbor,
                       int min_x, int max_x, int min_y, int max_y,
                       int mode, double radius)
{
    if (neighbor.size() != 0)
    {
        neighbor.resize(0);
    }
    if (mode == 0)
    {
        //4-neighbor
        if (p.x > min_x)
        {
            neighbor.push_back(point2d_t<int>(p.x - 1, p.y));
        }
        if (p.x < max_x - 1)
        {
            neighbor.push_back(point2d_t<int>(p.x + 1, p.y));
        }
        if (p.y > min_y)
        {
            neighbor.push_back(point2d_t<int>(p.x, p.y - 1));
        }
        if (p.y < max_y - 1)
        {
            neighbor.push_back(point2d_t<int>(p.x, p.y + 1));
        }
    }
    else if (mode == 1)
    {
        //8-neighbor
        if (p.x > min_x)
        {
            neighbor.push_back(point2d_t<int>(p.x - 1, p.y));
        }
        if (p.x < max_x - 1)
        {
            neighbor.push_back(point2d_t<int>(p.x + 1, p.y));
        }
        if (p.y > min_y)
        {
            neighbor.push_back(point2d_t<int>(p.x, p.y - 1));
        }
        if (p.y < max_y - 1)
        {
            neighbor.push_back(point2d_t<int>(p.x, p.y + 1));
        }
        if (p.x > min_x && p.y > min_y)
        {
            neighbor.push_back(point2d_t<int>(p.x - 1, p.y - 1));
        }
        if (p.x > min_x && p.y < max_y - 1)
        {
            neighbor.push_back(point2d_t<int>(p.x - 1, p.y + 1));
        }
        if (p.x < max_x - 1 && p.y > min_y)
        {
            neighbor.push_back(point2d_t<int>(p.x + 1, p.y - 1));
        }
        if (p.x < max_x - 1 && p.y < max_y - 1)
        {
            neighbor.push_back(point2d_t<int>(p.x + 1, p.y + 1));
        }
    }
    else
    {
        //all neighbor within radius
        if (radius < 1)
        {
            return;
        }
        for (int i = -floor(radius); i <= floor(radius); ++i)
        {
            int x = p.x + i;
            if (x >= min_x && x < max_x)
            {
                int len_y = floor(sqrt(radius * radius - i * i));
                for (int j = -len_y; j <= len_y; ++j)
                {
                    int y = p.y + j;
                    if (y >= min_y && y < max_y)
                    {
                        neighbor.push_back(point2d_t<int>(x, y));
                    }
                }
            }
        }
    }
}

bool compare_dis(const point2d_t<int>& a, const point2d_t<int>& b)
{
    int dis_a = a.x * a.x + a.y * a.y, dis_b = b.x * b.x + b.y * b.y;
    return (dis_a < dis_b) || (dis_a == dis_b && a < b);
}

std::vector<point2d_t<int>> get_convex_hull(const std::vector<point2d_t<int>>& hull)
{
    std::vector<point2d_t<int>> convex;
    if (hull.size() < 3)
    {
        return convex;
    }
    point2d_t<int> np = hull[0];
    for (int i = 1; i < hull.size(); ++i)
    {
        if (hull[i] < np)
        {
            np = hull[i];
        }
    }
    convex.push_back(np);
    while (true)
    {
        for (int i = 0; i < hull.size(); ++i)
        {
            if (hull[i] != np)
            {
                np = hull[i];
                break;
            }
        }
        for (int i = 0; i < hull.size(); ++i)
        {
            double cross = (np.x - convex.back().x) * (hull[i].y - np.y) -
                           (hull[i].x - np.x) * (np.y - convex.back().y);
            if (cross == 0 &&
                (np.x - convex.back().x) * (hull[i].x - np.x) +
                (np.y - convex.back().y) * (hull[i].y - np.y) > 0)
            {
                np = hull[i];
            }
            else if (cross > 0)
            {
                np = hull[i];
            }
        }
        if (np == convex[0])
        {
            break;
        }
        convex.push_back(np);
    }
    return convex;
}

double convex_area(const std::vector<point2d_t<int>>& hull)
{
    double area = 0;
    int n = hull.size();
    for (int i = 0; i < n; ++i)
    {
        area += (hull[(i + 1) % n].x + hull[i].x) * (hull[(i + 1) % n].y - hull[i].y);
    }
    return abs(area / 2);
}

double boundary_area(const std::vector<point2d_t<int>>& hull)
{
    double len = 0;
    int n = hull.size();
    for (int i = 0; i < n; ++i)
    {
        len += distance_pts(hull[i], hull[(i + 1) % n]);
    }
    return len * 0.5 - 1;
}

std::vector<double> get_circle_bound(const std::vector<point2d_t<double>>& hull)
{
    int min_id = 0, max_id = 0;
    double min_x = hull[0].x, max_x = hull[0].x;
    for (int i = 1; i < hull.size(); ++i)
    {
        if (hull[i].x < min_x)
        {
            min_x = hull[i].x;
            min_id = i;
        }
        if (hull[i].x > max_x)
        {
            max_x = hull[i].x;
            max_id = i;
        }
    }
    point2d_t<double> center = hull[min_id];
    center = (center + hull[max_id]) / 2;
    double r = distance_pts(hull[min_id], hull[max_id]) / 2;
    for (int i = 0; i < hull.size(); ++i)
    {
        if (i != min_id && i != max_id)
        {
            double dis = distance_pts(hull[i], center);
            if (dis > r)
            {
                point2d_t<double> new_p = (center - hull[i]) / dis * (dis + r) + hull[i];
                r = distance_pts(new_p, hull[i]) / 2;
                center = (new_p + hull[i]) / 2;
            }
        }
    }
    std::vector<double> circle;
    circle.push_back(center.x);
    circle.push_back(center.y);
    circle.push_back(r);
    return circle;
}

}
