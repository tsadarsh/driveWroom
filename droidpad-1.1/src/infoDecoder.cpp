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

#include <infoDecoder.hpp>

#include <glibmm/regex.h>
#include "libdp/uinput.h" // For AXIS_SIZE
#include "statfuncs.hpp"
#include <math.h> // Math funcs

/*
AXES LAYOUTS
  *   *
   *  *
\---*-*-\
 \   **  \
**\*******\** <-X
   \  **   \
    \-*-*---\ <- Phone
      *  *
      *   *
      ^   ^
      Z   Y
*/
INFO_DECODER_ITEMLIST InfoDecoder::DecodeOpts(Glib::ustring input)
{
	int i_beg = input.find('[') + 1; // Find first line only to avoid multiple instances stacking up after phone not flushing stream
	int i_end = input.find(']', i_beg);
	input = Glib::ustring(input, i_beg, i_end - i_beg);
	
	vector<Glib::ustring> items = Glib::Regex::split_simple(";", input);
	
	vector<Glib::ustring> tiltAxes = Glib::Regex::split_simple(",", items[0]); // 0 is the unprocessed axis.
	
	stringstream sX, sY, sZ;
	float X, Y, Z;
	
	sX << Glib::ustring(tiltAxes[0].raw(), 1); // Get X, Y, Z from data.
	sX >> X;
	sY << tiltAxes[1].raw();
	sY >> Y;
	sZ << Glib::ustring(tiltAxes[2].raw(), 0, tiltAxes[2].size() - 1);
	sZ >> Z;
	
	//cout << "X: " << X << ", Y: " << Y << ", Z: " << Z << endl;
	
	INFO_DECODER_ITEMLIST decItems;
	
	InfoDecoder::DpItem axis;
	
	axis.Type = InfoDecoder::AXIS;
	/* AXIS_CUTOFF_MULTIPLIER is (3 * AXIS_SIZE).
	 * This is to make the angle that the phone needs to be tilted to be
	 * -60 to 60 degrees, rather than 180.
	 */
	axis.X = trimMinMax(atan2(X, sqrt(Y * Y + Z * Z)) / M_PI * AXIS_CUTOFF_MULTIPLIER, -AXIS_SIZE, AXIS_SIZE); // Trigonometry functions - including pythagorising Y and Z
	axis.Y = trimMinMax(atan2(Y, Z) / M_PI * AXIS_CUTOFF_MULTIPLIER, -AXIS_SIZE, AXIS_SIZE);
	
	decItems.push_back(axis);
	
	for(int i = 1; i < items.size(); i++)
	{
		InfoDecoder::DpItem tmpAxis;
		if(items[i].size() == 1) // Button
		{
			tmpAxis.Type = InfoDecoder::BUTTON;
			tmpAxis.Pressed = false;
			if(items[i] == "1")
				tmpAxis.Pressed = true;
		}
		else if(items[i].find("{A") != -1) // Axis - already trigonometrised
		{
			vector<Glib::ustring> tmpVals = Glib::Regex::split_simple(",", items[i]);
			tmpAxis.Type = InfoDecoder::AXIS;
			
			stringstream sX, sY;
			sX << Glib::ustring(tmpVals[0], 2).raw(); // Split the "{A" off the beginning
			sX >> tmpAxis.X;
			sY << Glib::ustring(tmpVals[1], 0, tmpVals[1].size() - 1).raw();
			sY >> tmpAxis.Y;
		}
		else if(items[i].find("{S") != -1) // Slider
		{
			tmpAxis.Type = InfoDecoder::SLIDER;
			
			stringstream sX; // In a slider, single value is saved in X
			sX << Glib::ustring(items[i], 2, items[i].size() - 3).raw(); // Cutting off "{S" and "}"
			sX >> tmpAxis.X;
		}
		decItems.push_back(tmpAxis);
	}
	
	return decItems;
}
