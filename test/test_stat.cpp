//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2011
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################

#include "SlsDetectorDefs.h"
#include "lima/Exceptions.h"
#include <cmath>

DEB_GLOBAL(DebModTest);

using namespace std;
using namespace lima;
using namespace SlsDetector;

bool is_approx(double x, double y, double tol)
{
	return fabs(x - y) <= tol;
}

bool is_approx_rel(double x, double y, double rel_tol)
{
	return is_approx(x, y, fabs(rel_tol * (fabs(x) + fabs(y)) / 2));
}

void test_stat(double slope, double offset, int n)
{
	DEB_GLOBAL_FUNCT();

	XYStat stat;
	for (int i = 0; i < n; ++i)
		stat.add(i, slope * i + offset);
	XYStat::LinRegress r = stat.calcLinRegress();
	DEB_ALWAYS() << DEB_VAR1(r);
	if (r.n != n)
		THROW_HW_ERROR(Error) << DEB_VAR2(r.n, n);
	else if (!is_approx_rel(r.slope, slope, 1e-3))
		THROW_HW_ERROR(Error) << DEB_VAR2(r.slope, slope);
	else if (!is_approx_rel(r.offset, offset, 1e-3))
		THROW_HW_ERROR(Error) << DEB_VAR2(r.offset, offset);
}

int main(int argc, char *argv[])
{
	DEB_GLOBAL_FUNCT();

	double slope, offset;
	int n;

	slope = 0.12345;
	offset = 123.45;
	n = 100;
	test_stat(slope, offset, n);

	return 0;
}
