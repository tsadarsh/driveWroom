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

#include "mainwin.hpp"

#include "config.h"

#include <gtkmm/messagedialog.h>

MainWin::MainWin(gladeBuilder refBuilder, string dataPath) :
confEdit(Glib::ustring(dataPath))
{
	topvbox = 0;
	refBuilder->get_widget("topvbox", topvbox);
	if(topvbox)
	{
		add(*topvbox);
		set_title(_("DroidPad"));
	}
	else
	{
		set_title(_("DroidPad - Layouts corrupted!!!"));
	}
	dpLogo = Gdk::Pixbuf::create_from_file(dataPath + "/icon.xpm");
	set_icon(dpLogo);
	quitting = false;
	wThread = NULL;
	
	// Get widgets from layout
	INIT_WIDGET(buttonStart);
	INIT_WIDGET(buttonStop);
	
	INIT_WIDGET(menufileStart);
	INIT_WIDGET(menufileStop);
	INIT_WIDGET(menufileQuit);
	INIT_WIDGET(menuhelpAbout);
	
	INIT_WIDGET(confFrame);
	INIT_WIDGET(statFrame);
	
	INIT_WIDGET(statbuttonbox);
	
	INIT_WIDGET(statprogressX);
	INIT_WIDGET(statprogressY);
	
	INIT_WIDGET(ip1);
	ip1->set_text(confEdit.getVal("ip1", "192"));
	INIT_WIDGET(ip2);
	ip2->set_text(confEdit.getVal("ip2", "168"));
	INIT_WIDGET(ip3);
	ip3->set_text(confEdit.getVal("ip3", "1"));
	INIT_WIDGET(ip4);
	ip4->set_text(confEdit.getVal("ip4", "100"));
	INIT_WIDGET(port);
	port->set_text(confEdit.getVal("port", "3141"));
	
	// USB list
	INIT_WIDGET(usbTree);
	usbTreeStore = Gtk::ListStore::create(usbTreeColumns);
	
	usbTree->set_model(usbTreeStore);
	Gtk::TreeModel::Row row = *(usbTreeStore->append());
	row[usbTreeColumns.col_id] = 0;
	row[usbTreeColumns.col_serial] = "Usb not yet supported";
	usbTree->append_column(_("Device"), usbTreeColumns.col_id);
	usbTree->append_column(_("Serial No."), usbTreeColumns.col_serial);
	
	INIT_WIDGET(dpAbout);
	dpAbout->set_version(PACKAGE_VERSION);
	dpAbout->set_logo(dpLogo);
	dpAbout->set_icon(dpLogo);
	dpAbout->set_transient_for(*this);
	
	// Signal connecting
	CONNECT_SIG(buttonStart, signal_clicked, on_buttonStart_click);
	CONNECT_SIG(menufileStart, signal_activate, on_buttonStart_click);
	
	CONNECT_SIG(buttonStop, signal_clicked, on_buttonStop_click);
	CONNECT_SIG(menufileStop, signal_activate, on_buttonStop_click);
	
	CONNECT_SIG(menufileQuit, signal_activate, on_menufileQuit_click);
	CONNECT_SIG(menuhelpAbout, signal_activate, on_menuhelpAbout_click);
	
	CONNECT_SIG(dpAbout, signal_response, on_dpAbout_response);
	CONNECT_SIG((Gtk::Window*)this, signal_delete_event, on_delete_event);
}

MainWin::~MainWin()
{
	delete wThread;
}

void MainWin::on_buttonStart_click()
{
	int p1 = atoi(ip1->get_text().c_str());
	if(p1 == 0) ip1->set_text("0");
	int p2 = atoi(ip2->get_text().c_str());
	if(p2 == 0) ip2->set_text("0");
	int p3 = atoi(ip3->get_text().c_str());
	if(p3 == 0) ip3->set_text("0");
	int p4 = atoi(ip4->get_text().c_str());
	if(p4 == 0) ip4->set_text("0");
	
	int portN = atoi(port->get_text().c_str());
	if(portN == 0) port->set_text("3141");
	
	buttonStart->set_sensitive(false);
	menufileStart->set_sensitive(false);
	confFrame->set_sensitive(false);
	wThread = new WThread(p1, p2, p3, p4, portN);
	
	statbuttons = NULL;
	
	CONNECT_SIG(wThread, signal_statUpdate, on_wThread_statUpdate);
	CONNECT_SIG(wThread, signal_started, on_wThread_started);
	CONNECT_SIG(wThread, signal_stopped, on_wThread_stopped);
	
	wThread->Start();
}

void MainWin::on_buttonStop_click()
{
	wThread->Stop();
	buttonStop->set_sensitive(false);
	menufileStop->set_sensitive(false);
	statFrame->set_sensitive(false);
}

void MainWin::on_menufileQuit_click()
{
	if(Quit())
		hide();
}

void MainWin::on_menuhelpAbout_click()
{
	dpAbout->run();
}

void MainWin::on_dpAbout_response(int)
{
	dpAbout->hide();
}

bool MainWin::on_delete_event(GdkEventAny* event)
{
	confEdit.setVal("ip1", ip1->get_text());
	confEdit.setVal("ip2", ip2->get_text());
	confEdit.setVal("ip3", ip3->get_text());
	confEdit.setVal("ip4", ip4->get_text());
	confEdit.setVal("port", port->get_text());
	return !Quit();
}

bool MainWin::Quit()
{
	if(wThread == NULL)
		return true;
	quitting = true;
	on_buttonStop_click();
	return false;
}

void MainWin::on_wThread_started(startData data)
{
	sData = data;
	cout << _("Started!") << endl;
	buttonStop->set_sensitive(true);
	menufileStop->set_sensitive(true);
	statFrame->set_sensitive(true);
	
	statbuttons = new Gtk::CheckButton[data.buttonNum];
	for(int i = 0; i < sData.buttonNum; i++)
	{
		statbuttonbox->pack_start(statbuttons[i], true, true);
		statbuttons[i].show();
	}
}

void MainWin::on_wThread_stopped(int reason)
{
	Gtk::MessageDialog dialog2(*this, _("Error connecting to phone"));
	dialog2.set_secondary_text(_("Perhaps DroidPad is not running on your phone, or you have typed in the wrong IP address?"));
	Gtk::MessageDialog dialog(*this, _("Error creating connection"));
	dialog.set_secondary_text(_("An unknown error occurred while connecting to the phone"));
	switch(reason)
	{
	case STOP_Socket:
		dialog.run();
		break;
	case STOP_Connect:
		dialog2.run();
		break;
	}
	
	buttonStart->set_sensitive(true);
	menufileStart->set_sensitive(true);
	confFrame->set_sensitive(true);
	
	buttonStop->set_sensitive(false);
	menufileStop->set_sensitive(false);
	statFrame->set_sensitive(false);
	
	statprogressX->set_fraction(0.5);
	statprogressY->set_fraction(0.5);
	
	if(statbuttons != NULL)
	{
		delete[] statbuttons;
		statbuttons = NULL;
	}
	
	if(wThread != NULL)
	{
		delete wThread;
		wThread = NULL;
	}
	if(quitting)
		hide();
}

void MainWin::on_wThread_statUpdate(INFO_DECODER_ITEMLIST data)
{
	statprogressX->set_fraction((double)((double)(data.begin()->X - sData.rangeXmin) / (double)(sData.rangeXmax - sData.rangeXmin)));
	statprogressY->set_fraction((double)((double)(data.begin()->Y - sData.rangeYmin) / (double)(sData.rangeYmax - sData.rangeYmin)));
	
	int num = 0;
	for(INFO_DECODER_ITEMLIST::iterator iter = data.begin(); iter != data.end(); iter++)
	{
		if(iter->Type == InfoDecoder::BUTTON)
		{
			statbuttons[num++].set_active(iter->Pressed);
			//cout << (iter->Pressed ? "1" : "0") << ", ";
		}
	}
	//cout << data.size() << endl;
}

