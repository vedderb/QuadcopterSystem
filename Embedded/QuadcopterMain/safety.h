/*
	Copyright 2013-2015 Benjamin Vedder	benjamin@vedder.se

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

#ifndef SAFETY_H_
#define SAFETY_H_

#include "conf_general.h"

// Functions
void safety_init(void);
void safety_update_anchor(ANCHOR_SAFETY_INFO *info);
void safety_msg_received(SAFETY_MSG_t *msg);
void safety_los_int_received(LEVEL_OF_SERVICE_t los);
LEVEL_OF_SERVICE_t safety_get_level_of_service(void);
LEVEL_OF_SERVICE_t safety_get_level_of_service_internal(void);

#endif /* SAFETY_H_ */
