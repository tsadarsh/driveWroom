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

dpdatadir = $(datadir)/droidpad
dpdata_DATA = layouts.glade icon.xpm dp.conf.default

pixmapsdir = $(datadir)/pixmaps
pixmaps_DATA = droidpad.xpm

menudir = $(datadir)/applications
menu_in_files = droidpad.desktop.in
menu_DATA = $(menu_in_files:.desktop.in=.desktop)
@INTLTOOL_DESKTOP_RULE@

EXTRA_DIST = $(dpdata_DATA) $(pixmaps_DATA) droidpad.desktop.in

DISTCLEANFILES = droidpad.desktop
