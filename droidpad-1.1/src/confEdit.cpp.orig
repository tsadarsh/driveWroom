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
#include "confEdit.hpp"

#include <unistd.h>	// getuid
#include <sys/types.h>	// getuid
#include <pwd.h>

#include <cstring>

#include <giomm/fileinputstream.h>

#include <glib/gstdio.h>

#include "statfuncs.hpp"

ConfEdit::ConfEdit(Glib::ustring dataDir)
{
	homeDir = getpwuid(getuid())->pw_dir;
	
	Glib::RefPtr<Gio::File> dpDir = Gio::File::create_for_path((Glib::ustring)homeDir + (Glib::ustring)"/.droidpad/");
	if(!dpDir->query_exists())
	{
		if(!dpDir->make_directory())
		{
			cout << "Error: Could not create conf directory." << endl;
		}
	}
	
	dpConf = Gio::File::create_for_path((Glib::ustring)homeDir + (Glib::ustring)"/.droidpad/dp.conf");
	
	noConf = false;
	if(!dpConf->query_exists())
	{
		Glib::RefPtr<Gio::File> defaultConf = Gio::File::create_for_path(dataDir + Glib::ustring("/dp.conf.default"));
		defaultConf->copy(dpConf);
		if(!dpConf->query_exists())
		{
			cout << "Configuration file could not be copied." << endl;
			noConf = true;
			return;
		}
	}
	
	Glib::RefPtr<Gio::FileInputStream> inStream = dpConf->read();
	
	gchar buffer[BUFFER_SIZE];
	memset(buffer, 0, BUFFER_SIZE);
	
	if(!inStream->read(buffer, BUFFER_SIZE))
	{
		noConf = true; // Config can't be read
		return;
	}
	vector<Glib::ustring> items = Glib::Regex::split_simple("\n", Glib::ustring(buffer)); // Split lines
	
	vector<confItem> tmpConf (items.size());
	for(int i = 0; i < items.size(); i++)
	{
		vector<Glib::ustring> currItem = Glib::Regex::split_simple(":", items[i]); // Split key-val
		
		confItem tmpItem;
		if(currItem.size() >= 2)
		{
			tmpItem.name = currItem[0];
			tmpItem.value = currItem[1];
		}
		else
		{
			tmpItem.name = "";
			tmpItem.value = "";
		}
		tmpConf[i] = tmpItem;
	}
	conf = tmpConf;
}

ConfEdit::~ConfEdit()
{
	if(noConf) return;
	
	gchar buffer[BUFFER_SIZE];
	Glib::ustring gbuffer;
	
	for(int i = 0; i < conf.size(); i++)
	{
		gbuffer += conf[i].name + ":" + conf[i].value;
		if(i < (conf.size() - 1)) // All but last iteration
			gbuffer += "\n";
	}
	strcpy(buffer, gbuffer.c_str());
	FILE *file = g_fopen(Glib::ustring(dpConf->get_path()).c_str(), "w");
	fputs(buffer, file);
	fclose(file);
}

Glib::ustring ConfEdit::getVal(Glib::ustring name, Glib::ustring defVal)
{
	if(noConf) return defVal;
	for(int i = 0; i < conf.size(); i++)
	{
		if(conf[i].name == name)
			return conf[i].value;
	}
	return defVal;
}

void ConfEdit::setVal(Glib::ustring name, Glib::ustring val)
{
	if(noConf) return;
	for(int i = 0; i < conf.size(); i++)
	{
		if(conf[i].name == name)
			conf[i].value = val;
	}
}
