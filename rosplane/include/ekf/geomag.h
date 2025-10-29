#ifdef __cplusplus
extern "C" {
#endif

// Part of readsb, a Mode-S/ADSB/TIS message decoder.
//
// geomag.h: Geomagnetism calculator (header)
//
// Copyright (c) 2020 Michael Wolf <michael@mictronics.de>
//
// This file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef GEOMAG_H
#define GEOMAG_H

int load_magnetic_model();
int geomag_calc(double alt, double lat, double lon, double decimal_year, double *dec, double *dip, double *ti, double *gv);

#endif /* GEOMAG_H */

#ifdef __cplusplus
}
#endif
