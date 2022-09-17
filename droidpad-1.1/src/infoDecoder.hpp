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

#ifndef INFODECODER_H
#define INFODECODER_H

#include <iostream>
using namespace std;

#include <glibmm/ustring.h>

#include <list>
#include <vector>

#define INFO_DECODER_ITEMLIST list<InfoDecoder::DpItem>

class InfoDecoder
{
public:
	enum DpItemType
	{
		BUTTON,
		AXIS,
		SLIDER,
	};
	struct DpItem
	{
		int X, Y;
		bool Pressed;
		DpItemType Type;
	};
	
	static INFO_DECODER_ITEMLIST DecodeOpts(Glib::ustring input);
};

#endif
