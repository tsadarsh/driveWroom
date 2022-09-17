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

AC_PREREQ([2.65])
AC_INIT([droidpad], [1.1], [w.shackleton+droidpad@gmail.com])
AC_CONFIG_SRCDIR([src/droidpad.cpp])
AC_CONFIG_HEADERS([config.h])
AM_INIT_AUTOMAKE

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_RANLIB

IT_PROG_INTLTOOL([0.35.0])

GETTEXT_PACKAGE=droidpad
AC_SUBST(GETTEXT_PACKAGE)
AC_DEFINE_UNQUOTED([GETTEXT_PACKAGE], ["$GETTEXT_PACKAGE"],
                   [The domain to use with gettext])
ALL_LINGUAS="fr"
AM_GLIB_GNU_GETTEXT

PROGRAMNAME_LOCALEDIR=[${datadir}/locale]
AC_SUBST(PROGRAMNAME_LOCALEDIR)

# Checks for header files.
AC_CHECK_HEADER([linux/input.h])
AC_CHECK_HEADER([linux/uinput.h])
AC_CHECK_HEADER([linux/fcntl.h])
AC_CHECK_HEADER([sys/stat.h])
AC_CHECK_HEADER([sys/types.h])
AC_CHECK_HEADER([errno.h])

PKG_CHECK_MODULES([GTKmm], [gtkmm-2.4 >= 2.8.0])

# Checks for library functions.
AC_CONFIG_FILES([
Makefile
po/Makefile.in
data/Makefile
src/Makefile
src/libdp/Makefile
])
AC_OUTPUT
