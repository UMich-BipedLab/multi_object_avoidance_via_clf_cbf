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
/*
 * Copyright (c) 2021 Agility Robotics
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "planner_info_to_controller_t.h"
#include <stddef.h>
#include <string.h>

void pack_planner_info_to_controller_t(const planner_info_to_controller_t *bus,
  unsigned char bytes[2136])
{
  float x;
  unsigned char y[4];
  int i0;
  unsigned char b_y[12];
  float b_x[3];
  unsigned char c_y[4];
  unsigned char d_y[4];
  unsigned char e_y[4];
  unsigned char f_y[1600];
  float c_x[400];
  unsigned char g_y[240];
  float d_x[60];
  unsigned char h_y[240];
  unsigned char i_y[28];
  float e_x[7];
  x = (float)bus->behavior;
  memcpy((void *)&y[0], (void *)&x, (unsigned int)((size_t)4 * sizeof(unsigned
           char)));
  for (i0 = 0; i0 < 3; i0++) {
    b_x[i0] = (float)bus->velocity[i0];
  }

  memcpy((void *)&b_y[0], (void *)&b_x[0], (unsigned int)((size_t)12 * sizeof
          (unsigned char)));
  x = (float)bus->torso.roll;
  memcpy((void *)&c_y[0], (void *)&x, (unsigned int)((size_t)4 * sizeof(unsigned
           char)));
  x = (float)bus->torso.pitch;
  memcpy((void *)&d_y[0], (void *)&x, (unsigned int)((size_t)4 * sizeof(unsigned
           char)));
  x = (float)bus->torso.yaw;
  memcpy((void *)&e_y[0], (void *)&x, (unsigned int)((size_t)4 * sizeof(unsigned
           char)));
  for (i0 = 0; i0 < 400; i0++) {
    c_x[i0] = (float)bus->terrain[i0];
  }

  memcpy((void *)&f_y[0], (void *)&c_x[0], (unsigned int)((size_t)1600 * sizeof
          (unsigned char)));
  for (i0 = 0; i0 < 60; i0++) {
    d_x[i0] = (float)bus->waypoints[i0];
  }

  memcpy((void *)&g_y[0], (void *)&d_x[0], (unsigned int)((size_t)240 * sizeof
          (unsigned char)));
  for (i0 = 0; i0 < 60; i0++) {
    d_x[i0] = (float)bus->footplacement[i0];
  }

  memcpy((void *)&h_y[0], (void *)&d_x[0], (unsigned int)((size_t)240 * sizeof
          (unsigned char)));
  for (i0 = 0; i0 < 7; i0++) {
    e_x[i0] = (float)bus->pose[i0];
  }

  memcpy((void *)&i_y[0], (void *)&e_x[0], (unsigned int)((size_t)28 * sizeof
          (unsigned char)));
  for (i0 = 0; i0 < 4; i0++) {
    bytes[i0] = y[i0];
  }

  for (i0 = 0; i0 < 12; i0++) {
    bytes[i0 + 4] = b_y[i0];
  }

  for (i0 = 0; i0 < 4; i0++) {
    bytes[i0 + 16] = c_y[i0];
    bytes[i0 + 20] = d_y[i0];
    bytes[i0 + 24] = e_y[i0];
  }

  memcpy(&bytes[28], &f_y[0], 1600U * sizeof(unsigned char));
  for (i0 = 0; i0 < 240; i0++) {
    bytes[i0 + 1628] = g_y[i0];
    bytes[i0 + 1868] = h_y[i0];
  }

  for (i0 = 0; i0 < 28; i0++) {
    bytes[i0 + 2108] = i_y[i0];
  }
}





void unpack_planner_info_to_controller_t(const unsigned char bytes[2136],
  planner_info_to_controller_t *bus)
{
  int i;
  float y;
  unsigned char x[4];
  float b_y[3];
  unsigned char b_x[12];
  float c_y;
  float d_y;
  unsigned char c_x[1600];
  float e_y[400];
  unsigned char d_x[240];
  float f_y[60];
  float g_y[7];
  unsigned char e_x[28];
  for (i = 0; i < 4; i++) {
    x[i] = bytes[i];
  }

  memcpy((void *)&y, (void *)&x[0], (unsigned int)((size_t)1 * sizeof(float)));
  bus->behavior = y;
  for (i = 0; i < 12; i++) {
    b_x[i] = bytes[i + 4];
  }

  memcpy((void *)&b_y[0], (void *)&b_x[0], (unsigned int)((size_t)3 * sizeof
          (float)));
  for (i = 0; i < 3; i++) {
    bus->velocity[i] = b_y[i];
  }

  for (i = 0; i < 4; i++) {
    x[i] = bytes[i + 16];
  }

  memcpy((void *)&y, (void *)&x[0], (unsigned int)((size_t)1 * sizeof(float)));
  for (i = 0; i < 4; i++) {
    x[i] = bytes[i + 20];
  }

  memcpy((void *)&c_y, (void *)&x[0], (unsigned int)((size_t)1 * sizeof(float)));
  for (i = 0; i < 4; i++) {
    x[i] = bytes[i + 24];
  }

  memcpy((void *)&d_y, (void *)&x[0], (unsigned int)((size_t)1 * sizeof(float)));
  bus->torso.roll = y;
  bus->torso.pitch = c_y;
  bus->torso.yaw = d_y;
  memcpy(&c_x[0], &bytes[28], 1600U * sizeof(unsigned char));
  memcpy((void *)&e_y[0], (void *)&c_x[0], (unsigned int)((size_t)400 * sizeof
          (float)));
  for (i = 0; i < 400; i++) {
    bus->terrain[i] = e_y[i];
  }

  memcpy(&d_x[0], &bytes[1628], 240U * sizeof(unsigned char));
  memcpy((void *)&f_y[0], (void *)&d_x[0], (unsigned int)((size_t)60 * sizeof
          (float)));
  for (i = 0; i < 60; i++) {
    bus->waypoints[i] = f_y[i];
  }

  memcpy(&d_x[0], &bytes[1868], 240U * sizeof(unsigned char));
  memcpy((void *)&f_y[0], (void *)&d_x[0], (unsigned int)((size_t)60 * sizeof
          (float)));
  for (i = 0; i < 60; i++) {
    bus->footplacement[i] = f_y[i];
  }

  for (i = 0; i < 28; i++) {
    e_x[i] = bytes[i + 2108];
  }

  memcpy((void *)&g_y[0], (void *)&e_x[0], (unsigned int)((size_t)7 * sizeof
          (float)));
  for (i = 0; i < 7; i++) {
    bus->pose[i] = g_y[i];
  }
}
