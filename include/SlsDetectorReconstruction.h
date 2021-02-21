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
#ifndef __SLS_DETECTOR_RECONSTRUCTION_H
#define __SLS_DETECTOR_RECONSTRUCTION_H

#include "processlib/LinkTask.h"
#include "lima/Debug.h"
#include "lima/Exceptions.h"

namespace lima
{

namespace SlsDetector
{

/*******************************************************************
 * Reconstruction
 *******************************************************************/

class Reconstruction : public LinkTask
{
	DEB_CLASS_NAMESPC(DebModCamera, "Reconstruction", "SlsDetector");

public:
	class CtrlObjProxy {
		DEB_CLASS_NAMESPC(DebModCamera, "Reconstruction::CtrlObjProxy",
				  "SlsDetector");
	public:
		CtrlObjProxy(Reconstruction *r);
		virtual ~CtrlObjProxy();

		LinkTask *getReconstructionTask();

	protected:
		virtual void reconstructionChange(LinkTask *task) = 0;

	private:
		friend class Reconstruction;
		Reconstruction *m_r;
	};

	Reconstruction();
	virtual ~Reconstruction();

	void setActive(bool  active);
	void getActive(bool& active);

private:
	friend class CtrlObjProxy;
	CtrlObjProxy *m_proxy;
	bool m_active;
};

} // namespace SlsDetector

} // namespace lima

#endif // __SLS_DETECTOR_RECONSTRUCTION_H
