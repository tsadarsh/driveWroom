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

#include "droidpad.hpp"

#include <gtkmm/messagedialog.h>

#include <unistd.h>	// getuid
#include <sys/types.h>	// getuid

#include <glibmm/optioncontext.h>

MainWin* mainWin = 0;
string dataPaths[] = {
DPDATADIR,
"data",
"../data",
"."
// Any other possible locations of data may be put here.
};

int main(int argc, char *argv[])
{
	bindtextdomain(GETTEXT_PACKAGE, PROGRAMNAME_LOCALEDIR);
	bind_textdomain_codeset(GETTEXT_PACKAGE, "UTF-8");
	textdomain(GETTEXT_PACKAGE);
	
	Glib::thread_init();
	
	Gtk::Main kit(argc, argv);
	
	signal(SIGSEGV, sigsegv);
	if(getuid() != 0)	// Check root
	{
		Gtk::MessageDialog errdlg(_("Please run DroidPad as root."));
		errdlg.set_secondary_text(_("DroidPad needs to be run as 'root' (Administrator) to operate correctly."));
		errdlg.run();
		return -1;
	}
	
	Glib::RefPtr<Gtk::Builder> refBuilder = Gtk::Builder::create();
	
	bool found = true;
	for(int i = 0; i < ARRAY_LENGTH(dataPaths, string); i++)
	{
		found = true;
		cout << Glib::ustring::compose(_("Searching for layouts in \"%1\" ..."), dataPaths[i]) << endl;
		try
		{
			refBuilder->add_from_file(dataPaths[i] + "/layouts.glade");
		}
		catch(const Glib::FileError& ex)
		{
			found = false;
		}
		catch(const Gtk::BuilderError& ex)
		{
			cerr << "BuilderError: " << ex.what() << endl;
			return 1;
		}
		
		if(found)
		{
			cout << _("Layouts found!") << endl;
			dataPath = dataPaths[i];
			i = ARRAY_LENGTH(dataPaths, string);
		}
	}
	
	if(!found)
	{
		cout << _("Data files can't be found, exiting.") << endl;
		return 1;
	}
	
	mainWin = new MainWin(refBuilder, dataPath);
	
	if(mainWin)
	{
		kit.run(*mainWin);
	}
	
	delete mainWin;
	
	return 0;
}

void sigsegv(int)
{
	signal(SIGSEGV, SIG_DFL);
	
	cout << "Error: Segmentation Fault" << endl;
	
	Gtk::MessageDialog errdlg(_("A fatal error has occured!"));
	errdlg.set_secondary_text(_("DroidPad has crashed!\nError: Segmentation Fault"));
	errdlg.run();
	
	abort();
}
