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

SUBDIRS = po data src

INTLTOOL_FILES = intltool-extract.in	\
		 intltool-merge.in	\
		 intltool-update.in

EXTRA_DIST = config.rpath m4/ChangeLog  $(INTLTOOL_FILES)

DISTCLEANFILES = ... intltool-extract \
                 intltool-merge \
                 intltool-update \
                 po/.intltool-merge-cache
