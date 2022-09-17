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

#ifndef MAINWIN_H
#define MAINWIN_H

#include "wthread.hpp"
#include "confEdit.hpp"

#include <iostream>

#include <gtkmm/main.h>
#include <gtkmm/window.h>
#include <gtkmm/button.h>
#include <gtkmm/checkbutton.h>
#include <gtkmm/entry.h>


#include <gtkmm/builder.h>
#include <gtkmm/frame.h>
#include <gtkmm/box.h>
#include <gtkmm/label.h>
#include <gtkmm/imagemenuitem.h>
#include <gtkmm/progressbar.h>

#include <gtkmm/treeview.h>
#include <gtkmm/liststore.h>
#include <gtkmm/treemodelcolumn.h>

#include <gtkmm/aboutdialog.h>

#include <glibmm/i18n.h>

using namespace std;

#define INIT_WIDGET(_widget)	\
	refBuilder->get_widget(#_widget, _widget)
#define CONNECT_SIG(_widget, _signal, _command)	\
	(_widget)->_signal().connect(sigc::mem_fun(*this, &MainWin::_command))

#define GCONF_DIR "/apps/droidpad"
#define DP_GCONF(_set)	(Glib::ustring(GCONF_DIR) + Glib::ustring("/") + Glib::ustring(#_set))

typedef Glib::RefPtr<Gtk::Builder> gladeBuilder;

class MainWin : public Gtk::Window
{
public:
	MainWin(gladeBuilder refBuilder, string dataPath);
	~MainWin();
	bool Quit();
	
	class UsbTreeColumns : public Gtk::TreeModel::ColumnRecord
	{
	public:
		UsbTreeColumns()
		{
			add(col_id);
			add(col_serial);
		}
		Gtk::TreeModelColumn<unsigned int> col_id;
		Gtk::TreeModelColumn<Glib::ustring> col_serial;
	};
private:
	void on_buttonStart_click();
	void on_buttonStop_click();
	
	void on_menufileQuit_click();
	void on_menuhelpAbout_click();
	
	void on_dpAbout_response(int);
	
	void on_wThread_started(startData data);
	void on_wThread_statUpdate(INFO_DECODER_ITEMLIST data);
	void on_wThread_stopped(int reason);
	bool on_delete_event(GdkEventAny* event);
	
	bool quitting;
	
	Gtk::VBox *topvbox;
	Gtk::Button *buttonStart, *buttonStop;
	Gtk::ImageMenuItem *menufileStart, *menufileStop, *menufileQuit,
		*menuhelpAbout;
	Gtk::Frame *confFrame, *statFrame;
	
	Gtk::HBox *statbuttonbox;
	
	Gtk::Entry *ip1, *ip2, *ip3, *ip4, *port;
	
	Gtk::CheckButton *statbuttons; // Dynamic array of labels
	Gtk::ProgressBar *statprogressX, *statprogressY;
	
	Gtk::TreeView *usbTree;
	Glib::RefPtr<Gtk::ListStore> usbTreeStore;
	UsbTreeColumns usbTreeColumns;
	
	Glib::RefPtr<Gdk::Pixbuf> winIcon;
	
	WThread *wThread;
	startData sData;
	
	Gtk::AboutDialog *dpAbout;
	Glib::RefPtr<Gdk::Pixbuf> dpLogo;
	
	ConfEdit confEdit;
};

#endif
