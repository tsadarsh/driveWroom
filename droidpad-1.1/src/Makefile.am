#   This file is part of DroidPad.
# 
#   DroidPad is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
# 
#   DroidPad is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
# 
#   You should have received a copy of the GNU General Public License
#   along with DroidPad.  If not, see <http://www.gnu.org/licenses/>.

SUBDIRS = libdp

bin_PROGRAMS = droidpad

# Files in backwards order here to allow for quicker compilation when finding
# bugs (headers force all files to recompile)
droidpad_SOURCES =			\
	statfuncs.cpp	statfuncs.hpp	\
	confEdit.cpp	confEdit.hpp	\
	infoDecoder.cpp infoDecoder.hpp \
	wthread.cpp	wthread.hpp	\
	mainwin.cpp	mainwin.hpp	\
	droidpad.cpp	droidpad.hpp	\
					\
	libdp/uinput.h
	
INCLUDES = @GTKmm_CFLAGS@
droidpad_LDADD = @GTKmm_LIBS@ libdp/libdp.a #-lgthread-2.0
AM_CPPFLAGS = -DPREFIX='"$(prefix)"' -DDATADIR='"$(datadir)"' -DDPDATADIR='"$(datadir)/droidpad"' -DPROGRAMNAME_LOCALEDIR=\"${PROGRAMNAME_LOCALEDIR}\" -DGETTEXT_PACKAGE=\"${GETTEXT_PACKAGE}\"
