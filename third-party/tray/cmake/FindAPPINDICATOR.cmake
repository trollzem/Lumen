# Remmina - The GTK+ Remote Desktop Client
#
# Copyright (C) 2011 Marc-Andre Moreau
# Copyright (C) 2014-2015 Antenore Gatta, Fabio Castelli, Giovanni Panozzo
# Copyright (C) 2016-2023 Antenore Gatta, Giovanni Panozzo
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor,
# Boston, MA  02110-1301, USA.

include(FindPackageHandleStandardArgs)

pkg_check_modules(APPINDICATOR ayatana-appindicator3-0.1)
if(APPINDICATOR_FOUND)
    SET(APPINDICATOR_AYATANA 1)
else()
    pkg_check_modules(APPINDICATOR appindicator3-0.1)
    if(APPINDICATOR_FOUND)
        SET(APPINDICATOR_LEGACY 1)
    endif()
endif()

mark_as_advanced(APPINDICATOR_INCLUDE_DIR APPINDICATOR_LIBRARY)
