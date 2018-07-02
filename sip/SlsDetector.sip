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

%MappedType std::set<int>
{
%TypeHeaderCode
#include <set>

using namespace std;

typedef set<int> CPP_SeqType;
typedef CPP_SeqType::value_type CPP_SeqValType;
%End

%ConvertToTypeCode
	if (sipIsErr == NULL) {
		bool ok = PyList_Check(sipPy);
		for (int i = 0; ok && (i < PyList_Size(sipPy)); ++i)
			ok = PyInt_Check(PyList_GET_ITEM(sipPy, i));
		return ok;
	}

	*sipCppPtr = new CPP_SeqType();
	for (int i = 0; i < PyList_Size(sipPy); ++i) {
		CPP_SeqValType val = PyInt_AS_LONG(PyList_GET_ITEM(sipPy, i));
		(*sipCppPtr)->insert(val);
	}
	return sipGetState(sipTransferObj);
%End

%ConvertFromTypeCode
	PyObject* sip_list = PyList_New(sipCpp->size());
	if (sip_list == NULL)
		return NULL;
	CPP_SeqType::iterator it = sipCpp->begin();
	for (int i = 0; it != sipCpp->end(); ++it, ++i)
		PyList_SET_ITEM(sip_list, i, PyInt_FromLong(*it));
	return sip_list;
%End
};

%MappedType std::vector<unsigned long>
{
%TypeHeaderCode
#include <vector>

using namespace std;

typedef vector<unsigned long> CPP_SeqType;
typedef CPP_SeqType::value_type CPP_SeqValType;
%End

%ConvertToTypeCode
	if (sipIsErr == NULL) {
		bool ok = PyList_Check(sipPy);
		for (int i = 0; ok && (i < PyList_Size(sipPy)); ++i)
			ok = PyLong_Check(PyList_GET_ITEM(sipPy, i));
		return ok;
	}

	*sipCppPtr = new CPP_SeqType();
	for (int i = 0; i < PyList_Size(sipPy); ++i) {
		CPP_SeqValType val = PyLong_AsLong(PyList_GET_ITEM(sipPy, i));
		(*sipCppPtr)->push_back(val);
	}
	return sipGetState(sipTransferObj);
%End

%ConvertFromTypeCode
	PyObject* sip_list = PyList_New(sipCpp->size());
	if (sip_list == NULL)
		return NULL;
	CPP_SeqType::iterator it = sipCpp->begin();
	for (int i = 0; it != sipCpp->end(); ++it, ++i)
		PyList_SET_ITEM(sip_list, i, PyLong_FromLong(*it));
	return sip_list;
%End
};

