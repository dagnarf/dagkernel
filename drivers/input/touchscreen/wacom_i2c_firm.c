/*
 *  wacom_i2c_firm.c - Wacom G5 Digitizer Controller (I2C bus)
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

const unsigned int Binary_nLength = 0x7FFF;
const unsigned char Mpu_type = 0x26;
unsigned int Firmware_version_of_file=0;
const unsigned int Firmware_version_of_file_44 = 0x31A;
const unsigned int Firmware_version_of_file_48 = 0x20A;
unsigned char * Binary;

#include "wacom_i2c_firm_P6_REV02.h"
#include "wacom_i2c_firm_P6_REV03.h"

