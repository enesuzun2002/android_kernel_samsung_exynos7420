/*
 * Generic Kernel Wakelock-Blocker Driver
 *
 * Copyright (c) 2018  Lukas Berger <mail@lukasberger.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/**
 * Checks if a kernel-wakelock was blocked by user.
 *
 * @name  Name of the wakelock to check
 *
 * @returns  Either true if the wakelock should be blocked or
 *           false if otherwise
 */
extern bool kwakeblock_is_blocked(const char *name);
