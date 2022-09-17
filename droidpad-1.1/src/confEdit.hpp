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

#ifndef CONFEDIT_H
#define CONFEDIT_H

#include <giomm/file.h>
#include <glibmm/ustring.h>

#include <iostream>

#define BUFFER_SIZE	1024

using namespace std;

class ConfEdit
{
public:
	ConfEdit(Glib::ustring dataDir);
	~ConfEdit();
	struct confItem
	{
		Glib::ustring name;
		Glib::ustring value;
	};
	Glib::ustring getVal(Glib::ustring name, Glib::ustring defVal = "");
	void setVal(Glib::ustring name, Glib::ustring val);
private:
	
	Glib::ustring homeDir;
	Glib::RefPtr<Gio::File> dpConf;
	
	bool noConf;
	
	vector<confItem> conf;
};

#endif
