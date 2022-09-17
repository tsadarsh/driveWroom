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

#ifndef DROIDPAD_H
#define DROIDPAD_H

#include "statfuncs.hpp"
#include "mainwin.hpp"

#include <iostream>

#include <gtkmm/main.h>
#include <gtkmm/window.h>
#include <gtkmm/builder.h>
#include <glibmm/thread.h>

#include <glibmm/i18n.h>
using namespace std;

extern string dataPaths[];
string dataPath;

int main(int argc, char *argv[]);
void sigsegv(int);

#endif
