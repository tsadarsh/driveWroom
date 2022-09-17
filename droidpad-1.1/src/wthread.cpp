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

#include "wthread.hpp"

// Non-global includes
#include <glibmm/timer.h>

extern "C" {		// Various C includes for networking
#include <sys/socket.h>	/* for socket(), connect(), send(), and recv() */
#include <unistd.h>	/* for close() */
#include <arpa/inet.h>  /* for sockaddr_in and inet_addr() */

#include <stdlib.h>

#include "libdp/uinput.h"
}
#include <math.h>

#define SEND_PP_KEY(_key)	uinput_sendButton(dpinfo, _key, 1);	\
				uinput_sendButton(dpinfo, _key, 0);

WThread::WThread(int ip1,
		 int ip2,
		 int ip3,
		 int ip4,
		 int port) :
    thread(0),
    running(true),
    stopping(false)
{
	statUpdate_Disp.connect(sigc::mem_fun(*this, &WThread::sendStatUpdate));
	started_Disp.connect(sigc::mem_fun(*this, &WThread::sendStarted));
	stopped_Disp.connect(sigc::mem_fun(*this, &WThread::sendStopped));
	
	
	ipAddr = makeIPaddr(ip1, ip2, ip3, ip4);
	portN = port;
}

WThread::~WThread()
{
	if(thread != NULL)
		thread->join();
}

void WThread::Start()
{
	thread = Glib::Thread::create(sigc::mem_fun(*this, &WThread::start), true);
}

void WThread::Stop()
{
	stopping_Mutex.lock();
	stopping = true;
	stopping_Mutex.unlock();
}

void WThread::start() // This method contains quite a few C functions (networking, dp-uinput commands etc)
{
	int sock;
	struct sockaddr_in addr;
	
	if((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
	{
		stopReason = STOP_Socket;
		close(sock);
		stopped_Disp();
		return;
	}
	
	memset(&addr, 0, sizeof(addr));     /* Zero out structure */
	addr.sin_family	= AF_INET;
	addr.sin_addr.s_addr = ipAddr;
	addr.sin_port = htons(portN);
	
	if(connect(sock, (struct sockaddr *) &addr, sizeof(addr)) < 0)
	{
		stopReason = STOP_Connect;
		close(sock);
		stopped_Disp();
		return;
	}
	
	char recvBuffer[RECV_LEN];
	int bytesRecvd;
	
	if((bytesRecvd = recv(sock, recvBuffer, RECV_LEN, 0)) <= 0)
	{
		cout << _("Connection lost.") << endl;
		stopReason = STOP_Finished;
		close(sock);
		stopped_Disp();
		return;
	}
	
	Glib::ustring Mode = getDPOption(recvBuffer, "MODE");
	//cout << "Mode: " << Mode.c_str() << endl;
	
	startdata.rangeXmin = -AXIS_SIZE;
	startdata.rangeXmax = AXIS_SIZE;
	startdata.rangeYmin = -AXIS_SIZE;
	startdata.rangeYmax = AXIS_SIZE;
	
	INFO_DECODER_ITEMLIST initItems;
	do
	{
		memset(recvBuffer, 0, sizeof(recvBuffer));
		recv(sock, recvBuffer, RECV_LEN, 0);
		
		initItems = InfoDecoder::DecodeOpts(Glib::ustring(recvBuffer));
	} while(initItems.size() <= 1); // Wait until button screen has loaded and we can count button & axis numbers
	
	startdata.buttonNum = 0;
	startdata.axisNum = 0;
	for(INFO_DECODER_ITEMLIST::iterator iter = initItems.begin(); iter != initItems.end(); iter++)
	{
		if(iter->Type == InfoDecoder::BUTTON)
			startdata.buttonNum++;
		else if(iter->Type == InfoDecoder::AXIS)
			startdata.axisNum += 2;
		else if(iter->Type == InfoDecoder::SLIDER)
			startdata.axisNum++;
	}
	
	dpInfo* dpinfo = new dpInfo;
	dpinfo->axisMin = -AXIS_SIZE;
	dpinfo->axisMax = AXIS_SIZE;
	dpinfo->buttonNum = startdata.buttonNum;
	dpinfo->axisNum = startdata.axisNum;
	
	if(Mode.find("slide") == 0)
	{
		uinput_setup(dpinfo, TYPE_KEYBD);
	}
	else
	{
		if(Mode.find("mouse") == 0)
		{
			uinput_setup(dpinfo, TYPE_MOUSE);
		}
		else // Joystick
		{
			uinput_setup(dpinfo, TYPE_JS);
		}
	}
	
	started_Disp(); // Notifies UI that started
	
	while(running) // Main loop
	{
		memset(recvBuffer, 0, sizeof(recvBuffer));
		if((bytesRecvd = recv(sock, recvBuffer, RECV_LEN, 0)) <= 0)
		{
			cout << _("Connection lost.") << endl;
			stopReason = STOP_Finished;
			close(sock);
			stopped_Disp();
			return;
		}
		
		stopping_Mutex.lock();
		running = !stopping;
		stopping_Mutex.unlock();
		if(stopping)
		{
			Glib::ustring endData = "<STOP>";
			send(sock, endData.c_str(), endData.size(), 0);
		}
		
		currData = InfoDecoder::DecodeOpts(recvBuffer);
		
		if(Mode.find("slide") == 0)
		{
			// Process presentation here
			INFO_DECODER_ITEMLIST::iterator j = ++prevData.begin();
			INFO_DECODER_ITEMLIST::iterator i = ++currData.begin(); // Start at second entry (++)
			
			if((i++)->Pressed & !(j++)->Pressed) //INFO_DECODER_ITEMLIST::iterator i = currData.begin(); Forward
				SEND_PP_KEY(KEY_DOWN);
			if((i++)->Pressed & !(j++)->Pressed) // Backward
				SEND_PP_KEY(KEY_UP);
			if((i++)->Pressed & !(j++)->Pressed) // Start show
				SEND_PP_KEY(KEY_F5);
			if((i++)->Pressed & !(j++)->Pressed) // End show
				SEND_PP_KEY(KEY_ESC);
			if((i++)->Pressed ^ (j++)->Pressed) // Go to white	}
				SEND_PP_KEY(KEY_W);			//	} XOR because we want it to
			if((i++)->Pressed ^ (j++)->Pressed) // Go to black	} press if pressed & unpressed
				SEND_PP_KEY(KEY_B);			//	}
			if((i++)->Pressed & !(j++)->Pressed) // Beginning
				SEND_PP_KEY(KEY_HOME);
			if((i++)->Pressed & !(j++)->Pressed) // End
				SEND_PP_KEY(KEY_END);
			
			prevData = currData;
		}
		else
		{
			if(Mode.find("mouse") == 0)
			{
				// Process Mouse mode here
				INFO_DECODER_ITEMLIST::iterator i = currData.begin();
				
				uinput_send2Pos(dpinfo, i->X / (AXIS_SIZE / 10), i->Y / (AXIS_SIZE / 10));
				//!!Inline iterator increments here!!
				uinput_sendButton(dpinfo, BTN_LEFT, (++i)->Pressed ? 1 : 0);
				uinput_sendButton(dpinfo, BTN_MIDDLE, (++i)->Pressed ? 1 : 0);
				uinput_sendButton(dpinfo, BTN_RIGHT, (++i)->Pressed ? 1 : 0);
			}
			else
			{
				// Proces normal mode here
				int axes[currData.size()];
				int buttons[currData.size()];
				int axesPos = 0, buttonsPos = 0;
				
				for(INFO_DECODER_ITEMLIST::iterator i = currData.begin(); i != currData.end(); i++)
				{
					if(i->Type == InfoDecoder::BUTTON)
					{
						buttons[buttonsPos++] = i->Pressed ? 1 : 0;
					}
					else if(i->Type == InfoDecoder::AXIS)
					{
						axes[axesPos++] = i->X;
						axes[axesPos++] = i->Y;
					}
					else if(i->Type == InfoDecoder::SLIDER)
					{
						axes[axesPos++] = i->X;
					}
				}
				uinput_sendNPos   (dpinfo, axes,    axesPos);
				uinput_sendButtons(dpinfo, buttons, buttonsPos);
			}
		}
		
		statUpdate_Disp();
	}
	
	close(sock);
	uinput_close(dpinfo);
	delete dpinfo;
	
	stopped_Disp();
}

sigc::signal<void, INFO_DECODER_ITEMLIST> WThread::signal_statUpdate()
	{ return signal_statUpdate_; }
void WThread::sendStatUpdate()
	{ signal_statUpdate_(currData); }

sigc::signal<void, startData> WThread::signal_started()
{ return signal_started_; }
void WThread::sendStarted()
	{ if(running) signal_started_(startdata); }

sigc::signal<void, int> WThread::signal_stopped()
{ return signal_stopped_; }
void WThread::sendStopped()
	{ signal_stopped_(stopReason); }
