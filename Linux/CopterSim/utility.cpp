/*
    Copyright 2014 Benjamin Vedder	benjamin.vedder@sp.se

    This file is part of CopterSim.

    CopterSim is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CopterSim is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CopterSim.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "utility.h"
#include <cmath>

namespace utility {

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
    int16_t res =	((uint8_t) buffer[*index]) << 8 |
                    ((uint8_t) buffer[*index + 1]);
    *index += 2;
    return res;
}

uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
    uint16_t res = 	((uint8_t) buffer[*index]) << 8 |
                    ((uint8_t) buffer[*index + 1]);
    *index += 2;
    return res;
}

int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
    int32_t res =	((uint8_t) buffer[*index]) << 24 |
                    ((uint8_t) buffer[*index + 1]) << 16 |
                    ((uint8_t) buffer[*index + 2]) << 8 |
                    ((uint8_t) buffer[*index + 3]);
    *index += 4;
    return res;
}

uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
    uint32_t res =	((uint8_t) buffer[*index]) << 24 |
                    ((uint8_t) buffer[*index + 1]) << 16 |
                    ((uint8_t) buffer[*index + 2]) << 8 |
                    ((uint8_t) buffer[*index + 3]);
    *index += 4;
    return res;
}

void norm_angle(double *angle) {
    *angle = fmod(*angle, 360.0);

    if (*angle < 0.0) {
        *angle += 360.0;
    }
}

double weight_angle(double angle1, double angle2, double ratio) {
    norm_angle(&angle1);
    norm_angle(&angle2);

    if (fabs(angle1 - angle2) > 180.0) {
        if (angle1 < angle2) {
            angle1 += 360.0;
        } else {
            angle2 += 360.0;
        }
    }

    double angle_weighted = angle1 * ratio + angle2 * (1 - ratio);
    norm_angle(&angle_weighted);

    return angle_weighted;
}

void step_towards(double *value, double goal, double step) {
    if (*value < goal) {
        if ((*value + step) < goal) {
            *value += step;
        } else {
            *value = goal;
        }
    } else if (*value > goal) {
        if ((*value - step) > goal) {
            *value -= step;
        } else {
            *value = goal;
        }
    }
}

int truncate_number(double *number, double min, double max) {
    int did_trunc = 0;

    if (*number > max) {
        *number = max;
        did_trunc = 1;
    } else if (*number < min) {
        *number = min;
        did_trunc = 1;
    }

    return did_trunc;
}

double dist2(const POINT_t &v, const POINT_t &w) {
    return (v.x - w.x) * (v.x - w.x) + (v.y - w.y) * (v.y - w.y);
}

double dist(const POINT_t &v, const POINT_t &w) {
    return sqrt(dist2(v, w));
}

/**
 * @brief closest_point_on_segment
 * Find the closest point on a line segment to a point.
 *
 * @param s
 * The line segment.
 *
 * @param p
 * The point to compare to.
 *
 * @return
 * The closest point on the line segment.
 */
POINT_t closest_point_on_segment(const SEGMENT_t &s, const POINT_t &p) {
    const double a_to_p[] = {p.x - s.p1.x, p.y - s.p1.y};
    const double a_to_b[] = {s.p2.x - s.p1.x, s.p2.y - s.p1.y};
    const double atb2 = a_to_b[0] * a_to_b[0] + a_to_b[1] * a_to_b[1];
    const double atp_dot_atb = a_to_p[0] * a_to_b[0] + a_to_p[1] * a_to_b[1];
    double t = atp_dot_atb / atb2;

    if (t > 1.0) {
        t = 1.0;
    } else if (t < 0.0) {
        t = 0.0;
    }

    POINT_t res;
    res.x = s.p1.x + a_to_b[0] * t;
    res.y = s.p1.y + a_to_b[1] * t;

    return res;
}

}
