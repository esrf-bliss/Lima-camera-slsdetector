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
%End

%ConvertToTypeCode
if(sipIsErr == NULL)
{
bool aReturnFlag = PyList_Check(sipPy);
for(int i = 0;aReturnFlag && i < PyList_Size(sipPy);++i)
	aReturnFlag = PyInt_Check(PyList_GET_ITEM(sipPy,i));
return aReturnFlag;
}

*sipCppPtr = new std::set<int>();
for(int i = 0;i < PyList_Size(sipPy);++i)
	(*sipCppPtr)->insert(PyInt_AS_LONG(PyList_GET_ITEM(sipPy,i)));
return sipGetState(sipTransferObj);
%End

%ConvertFromTypeCode
PyObject* aReturnListPt = PyList_New(sipCpp->size());
std::set<int>::iterator lIter = sipCpp->begin();

for(int i = 0;lIter != sipCpp->end();++lIter,++i)
   PyList_SET_ITEM(aReturnListPt,i,PyInt_FromLong(*lIter));
return aReturnListPt;
%End
};

%MappedType std::map<unsigned long,std::set<int>>
{
%TypeHeaderCode
#include <map>
#include <set>
%End

%ConvertToTypeCode
PyObject *key, *value;
SIP_SSIZE_T pos;

if(sipIsErr == NULL)
{
  bool aReturnFlag = PyDict_Check(sipPy);

  pos = 0;
  while(aReturnFlag && PyDict_Next(sipPy, &pos, &key, &value))
   {
     aReturnFlag = PyInt_Check(key) && PyList_Check(value);
     for(int i = 0;aReturnFlag && i < PyList_Size(value);++i)
	aReturnFlag = PyInt_Check(PyList_GET_ITEM(value,i));
   }
  return aReturnFlag;
}

*sipCppPtr = new std::map<unsigned long,std::set<int> >();
pos = 0;
while(PyDict_Next(sipPy, &pos, &key, &value))
{
  unsigned long anIndex = PyInt_AS_LONG(key);
  std::pair<std::map<unsigned long,std::set<int> >::iterator, bool> aResult = 
  (*sipCppPtr)->insert(std::pair<unsigned long,std::set<int> >(anIndex,std::set<int>()));
  
  for(int i = 0;i < PyList_Size(value);++i)
  {
    int n = PyInt_AS_LONG(PyList_GET_ITEM(value,i));
    aResult.first->second.insert(n);
  }
}
return sipGetState(sipTransferObj);
%End

%ConvertFromTypeCode
PyObject* aReturnDictPt = PyDict_New();


for(std::map<unsigned long,std::set<int> >::iterator mIter = sipCpp->begin();
    mIter != sipCpp->end();++mIter)
   {
    PyObject *aKeyPt = PyInt_FromLong(mIter->first);
    PyObject *aValuePt = PyList_New(mIter->second.size());
    PyDict_SetItem(aReturnDictPt,aKeyPt,aValuePt);
    Py_DECREF(aKeyPt);Py_DECREF(aValuePt);
    std::set<int>::iterator iPt = mIter->second.begin();
    for(int i = 0;iPt != mIter->second.end();++i,++iPt)
      PyList_SET_ITEM(aValuePt,i,PyInt_FromLong(*iPt));
   }
return aReturnDictPt;
%End
};

