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

#ifndef STATFUNCS_H
#define STATFUNCS_H

#include <iostream>
#include <list>
using namespace std;

#include <glibmm/ustring.h>

#define ARRAY_LENGTH(_array, _type) (sizeof(_array) / sizeof(_type))

#define PRINT_LINE() cout << "Line: " << __FILE__ << ":" << __LINE__ << endl

//const char letters[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
//#define LETTERS_LEN sizeof(letters)

int trimMinMax(double num, double min, double max);
int trimMinMax(float num, float min, float max);

unsigned int makeIPaddr(int a, int b, int c, int d);

Glib::ustring getOption(Glib::ustring orig, Glib::ustring p1, Glib::ustring p2);
Glib::ustring getDPOption(Glib::ustring orig, Glib::ustring p);

//list<Glib::ustring> decodeCSV(Glib::ustring str);

#endif
