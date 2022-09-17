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

#include <statfuncs.hpp>


int trimMinMax(double num, double min, double max)
{
	if(num < min)
		return min;
	if(num > max)
		return max;
	return num;
}

int trimMinMax(float num, float min, float max)
{
	if(num < min)
		return min;
	if(num > max)
		return max;
	return num;
}

unsigned int makeIPaddr(int a, int b, int c, int d)
{
	return  (d << 24) +
		(c << 16) +
		(b << 8) +
		 a;
}

Glib::ustring getOption(Glib::ustring orig, Glib::ustring p1, Glib::ustring p2)
{
	int bPos = orig.find(p1) + p1.size();
	return Glib::ustring(
		orig,
		bPos,
		orig.find(p2) - bPos);
}

Glib::ustring getDPOption(Glib::ustring orig, Glib::ustring p)
{
	return getOption(
		orig,
		"<"  + p + ">",
		"</" + p + ">");
}
