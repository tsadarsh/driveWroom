/*  This file is part of DroidPad.
 *
 *  DroidPad is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  DroidPad is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with DroidPad.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DP_UINPUT_H
#define DP_UINPUT_H

#define AXIS_SIZE 16384
#define AXIS_CUTOFF_MULTIPLIER (16384 * 3)

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/uinput.h>

typedef struct dpinfo
{
	int ufile;
	int axisMin;
	int axisMax;
	int buttonNum;
	int axisNum;
	int type;
} dpInfo;

enum {
TYPE_JS,
TYPE_MOUSE,
TYPE_KEYBD
};

extern __u16 joystickKeys[];

#define ARRAY_COUNT(_array, _vartype)	(sizeof(_array) / sizeof(_vartype))

// TODO: Update function and file names to be relative to DroidPad - to stop confusion with <linux/uinput.h>

int uinput_setup(dpInfo *info, int type);
int uinput_close(dpInfo *info);

int uinput_send2Pos(dpInfo *info, int posX, int posY);
int uinput_sendNPos(dpInfo *info, int pos[], int count);
int uinput_sendButtons(dpInfo *info, int buttons[], int count);
int uinput_sendButton(dpInfo *info, int code, int val);

#endif

#ifdef __cplusplus
}
#endif
