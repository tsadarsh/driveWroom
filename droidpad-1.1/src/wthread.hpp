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

#ifndef WTHREAD_H
#define WTHREAD_H

#include <iostream>
using namespace std;

#include "statfuncs.hpp"
#include "infoDecoder.hpp"

#include <glibmm/i18n.h>

#include <glibmm/thread.h>
#include <glibmm/dispatcher.h>

struct startData
{
	int buttonNum, axisNum;
	int rangeXmin, rangeXmax, rangeYmin, rangeYmax;
};

enum {
STOP_None,
STOP_Socket,
STOP_Connect,
STOP_NotSupported,
STOP_Finished
};

#define RECV_LEN 10000

#define RADIAN_CONVERT (M_PI / 180)

class WThread
{
public:
	WThread(int ip1,
		int ip2,
		int ip3,
		int ip4,
		int port);
	~WThread();
	
	void Start();
	void Stop();
	
	sigc::signal<void, INFO_DECODER_ITEMLIST> signal_statUpdate();
	sigc::signal<void, startData> signal_started();
	sigc::signal<void, int> signal_stopped();
private:
	Glib::Thread* thread;
	void start();
	
	bool running, stopping;
	Glib::Mutex stopping_Mutex;
	
	int stopReason;
	
	// Thread funcs
	
	void sendStatUpdate();
	Glib::Dispatcher statUpdate_Disp;
	sigc::signal<void, INFO_DECODER_ITEMLIST> signal_statUpdate_;
	INFO_DECODER_ITEMLIST currData, prevData;
	
	void sendStarted();
	Glib::Dispatcher started_Disp;
	sigc::signal<void, startData> signal_started_;
	startData startdata;
	
	void sendStopped();
	Glib::Dispatcher stopped_Disp;
	sigc::signal<void, int> signal_stopped_;
	
	unsigned int ipAddr;
	int portN;
};

#endif
