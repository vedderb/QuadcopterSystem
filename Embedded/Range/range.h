/*
	Copyright 2013-2015 Benjamin Vedder benjamin@vedder.se
	Copyright 2013-2014 Daniel Skarin	daniel.skarin@sp.se

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

#ifndef RANGE_H_
#define RANGE_H_

#define CLOCK_UDP_PORT		3009
#define MAX_CLOCK_OFFSET	500
#define ANCHOR_NUM			4

typedef enum {
	NOT_SYNCHRONIZED = 0,
	SYNCHRONIZED = 1
} CLOCK_STATUS;

typedef enum {
	LEVEL_OF_SERVICE_LOW = 0,
	LEVEL_OF_SERVICE_MEDIUM,
	LEVEL_OF_SERVICE_HIGH
} LEVEL_OF_SERVICE_t;

#endif /* RANGE_H_ */
