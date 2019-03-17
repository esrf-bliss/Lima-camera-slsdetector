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

namespace SlsDetector
{

%TypeHeaderCode
#include "SlsDetectorFrameMap.h"
%End

class FrameMap
{
public:
	struct FrameData {
		int frame;
		int valid
		{
		        %GetCode
				sipPy = PyInt_FromLong(sipCpp->valid.to_ulong());
		        %End
		
		        %SetCode
			        sipCpp->valid = lima::SlsDetector::FrameMap::Mask(PyInt_AsLong(sipPy));
		        %End
		};

		FrameData();
		FrameData(int f, int v);
	};

	void setNbItems(int nb_items);
	void setBufferSize(int buffer_size);
	void clear();

	unsigned long getLastGroupFrame() const;
	unsigned long getLastFinishedFrame() const;

private:
	FrameMap();
	FrameMap(const FrameMap& o);
};


}; // namespace SlsDetector
