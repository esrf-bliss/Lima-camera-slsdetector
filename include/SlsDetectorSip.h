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

#ifndef __SLS_DETECTOR_SIP_H
#define __SLS_DETECTOR_SIP_H

#include <vector>
#include <set>
#include <map>

#include <lima/AutoObj.h>

using namespace std;
using namespace lima;

/*******************************************************
 * SipType
 *******************************************************/

template <class T>
class SipType
{
 public:
	static bool isCheckRequest(int *sipIsErr)
	{ return (sipIsErr == NULL); }

	class ConvertBase
	{
	public:
		ConvertBase(SipType& t, PyObject *p, T *v)
			: m_t(t.Sip_TypeDef), m_p(p), m_v(v)
		{}

		T *getCpp() const
		{ return m_v; }

		PyObject *getPyObject() const
		{ return m_p; }

	protected:
		sipTypeDef *m_t;
		PyObject *m_p;
		T *m_v;
	};

#define M_T ConvertBase::m_t
#define M_V ConvertBase::m_v
#define M_P ConvertBase::m_p

	class CheckConvertToCpp : public ConvertBase
	{
	public:
		CheckConvertToCpp(SipType& t, PyObject *p)
			: ConvertBase(t, p, NULL)
		{
			if (!check())
				throw exception();
		}

	private:
		bool check()
		{
			return sipCanConvertToType(M_P, M_T, SIP_NOT_NONE);
		}
	};

	class ConvertToCpp : public ConvertBase
	{
	public:
		ConvertToCpp(SipType& t, PyObject *p, int *sipIsErr)
			: ConvertBase(t, p, NULL), m_sipIsErr(sipIsErr),
			  m_conv(false)
		{
			convert();
		}

		~ConvertToCpp()
		{
			release();
		}

	private:
		void convert()
		{
			void *p;
			p = sipConvertToType(M_P, M_T, NULL, SIP_NOT_NONE,
					     &m_state, m_sipIsErr);
			M_V = static_cast<T *>(p);
			m_conv = true;
			if (*m_sipIsErr) {
				release();
				throw exception();
			}
		}

		void release()
		{
			if (m_conv)
				sipReleaseType(M_V, M_T, m_state);
		}

	private:
		int m_state;
		int *m_sipIsErr;
		bool m_conv;
	};

	class ConvertFromCpp : public ConvertBase
	{
	public:
		ConvertFromCpp(SipType& t, T *v)
			: ConvertBase(t, NULL, v)
		{
			convert();
		}

		~ConvertFromCpp()
		{
			if (!ownershipXferred())
				delete M_V;
		}

	private:
		void convert()
		{
			M_P = sipConvertFromNewType(M_V, M_T, NULL);
			if (M_P == NULL)
				throw exception();
		}

		bool ownershipXferred() const
		{
			return true;
		}
	};

	SipType(sipTypeDef *Sip_T = NULL) : Sip_TypeDef(Sip_T)
	{}

	CheckConvertToCpp checkConvertToCpp(PyObject *p)
	{ return CheckConvertToCpp(*this, p); }

	ConvertToCpp convertToCpp(PyObject *p, int *sipIsErr)
	{ return ConvertToCpp(*this, p, sipIsErr); }

	ConvertFromCpp convertFromCpp(T *c, bool /*is_new*/ = true)
	{ return ConvertFromCpp(*this, c); }

 private:
	sipTypeDef *Sip_TypeDef;
};

#define SipTypeIntImpl(T)						\
	template <>							\
	inline bool SipType<T>::CheckConvertToCpp::check()		\
	{ return PyInt_Check(M_P); }					\
									\
	template <>							\
	inline void SipType<T>::ConvertToCpp::convert()			\
	{								\
		*m_sipIsErr = 0;					\
		T v = static_cast<T>(PyInt_AS_LONG(M_P));		\
		M_V = new T(v);						\
	}								\
									\
	template <>							\
	inline void SipType<T>::ConvertToCpp::release()			\
	{ delete M_V; }							\
									\
	template <>							\
	inline void SipType<T>::ConvertFromCpp::convert()		\
	{								\
		M_P = PyInt_FromLong(*M_V);				\
		if (!M_P)						\
			throw exception();				\
	}								\
									\
	template <>							\
	inline bool SipType<T>::ConvertFromCpp::ownershipXferred() const \
	{ return false; }

SipTypeIntImpl(int);
SipTypeIntImpl(unsigned long);


/*******************************************************
 * PyObjectPtr
 *******************************************************/

class PyObjectPtr
{
 public:
	PyObjectPtr(PyObject *p = NULL) : m_p(p)
	{}

	~PyObjectPtr()
	{
		if (m_p)
			Py_DECREF(m_p);
	}

	operator PyObject *() const
	{ return m_p; }

	PyObject *forget()
	{
		PyObject *p = m_p;
		m_p = NULL;
		return p;
	}

 private:
	PyObject *m_p;
};


/*******************************************************
 * SipSeq
 *******************************************************/

template <class T>
inline void SipSeqAddVal(vector<T>& cpp_seq, const T& v)
{
	cpp_seq.push_back(v);
}

template <class T>
inline void SipSeqAddVal(set<T>& cpp_seq, const T& v)
{
	if (!cpp_seq.insert(v).second)
		throw exception();
}

template <class SeqType>
class SipSequence
{
 public:
	typedef typename SeqType::value_type ValueType;
	typedef SipType<ValueType> ValueSipType;
	typedef typename ValueSipType::ConvertToCpp ToCpp;
	typedef typename ValueSipType::ConvertFromCpp FromCpp;

	SipSequence(sipTypeDef *Sip_T = NULL) : Sip_Type(Sip_T)
	{}

	int convertToTypeCode(PyObject *sipPy, SeqType **sipCppPtr,
			       int *sipIsErr, PyObject *sipTransferObj)
	{
		if (Sip_Type.isCheckRequest(sipIsErr))
			return checkToCpp(sipPy);

		AutoPtr<SeqType> sipCpp = new SeqType();
		if (!convertToCpp(sipCpp, sipPy, sipIsErr))
			return 0;
		*sipCppPtr = sipCpp.forget();
		return sipGetState(sipTransferObj);
	}

	PyObject *convertFromTypeCode(SeqType *sipCpp)
	{
		PyObjectPtr sipPy = PyList_New(sipCpp->size());
		if (sipPy == NULL)
			return NULL;
		if (!convertFromCpp(sipPy, sipCpp))
			return NULL;
		return sipPy.forget();
	}

 private:
	bool checkToCpp(PyObject *sipPy)
	{
		bool ok = PyList_Check(sipPy);
		for (int i = 0; ok && (i < PyList_Size(sipPy)); ++i) {
			PyObject *value = PyList_GET_ITEM(sipPy, i);
			try {
				Sip_Type.checkConvertToCpp(value);
			} catch (...) {
				ok = false;
			}
		}
		return ok;
	}

	bool convertToCpp(SeqType *sipCpp, PyObject *sipPy, int *sipIsErr)
	{
		for (int i = 0; i < PyList_Size(sipPy); ++i) {
			PyObject *value = PyList_GET_ITEM(sipPy, i);
			try {
				ToCpp conv = Sip_Type.convertToCpp(value,
								   sipIsErr);
				ValueType *cpp_value = conv.getCpp();
				SipSeqAddVal(*sipCpp, *cpp_value);
			} catch (...) {
				return false;
			}
		}
		return true;
	}

	bool convertFromCpp(PyObject *sipPy, SeqType *sipCpp)
	{
		int i = 0;
		typename SeqType::const_iterator it, end = sipCpp->end();
		for (it = sipCpp->begin(); it != end; ++it, ++i) {
			AutoPtr<ValueType> new_val = new ValueType(*it);
			try {
				FromCpp conv = Sip_Type.convertFromCpp(new_val);
				new_val.forget();
				PyObjectPtr value = conv.getPyObject();
				PyList_SET_ITEM(sipPy, i, value);
				value.forget();
			} catch (...) {
				return false;
			}
		}
		return true;
	}

	ValueSipType Sip_Type;
};

/*******************************************************
 * SipMap
 *******************************************************/

template <class MapType>
class SipMap
{
 public:
	typedef typename MapType::key_type KeyType;
	typedef typename MapType::mapped_type MappedType;
	typedef typename MapType::value_type ValueType;
	typedef SipType<KeyType> KeySipType;
	typedef SipType<MappedType> MappedSipType;

	typedef typename KeySipType::ConvertToCpp KeyToCpp;
	typedef typename KeySipType::ConvertFromCpp KeyFromCpp;
	typedef typename MappedSipType::ConvertToCpp MappedToCpp;
	typedef typename MappedSipType::ConvertFromCpp MappedFromCpp;

	SipMap(sipTypeDef *Sip_KeyT = NULL, sipTypeDef *Sip_MappedT = NULL)
		: Sip_KeyType(Sip_KeyT), Sip_MappedType(Sip_MappedT)
	{}

	int convertToTypeCode(PyObject *sipPy, MapType **sipCppPtr,
			       int *sipIsErr, PyObject *sipTransferObj)
	{
		if (Sip_KeyType.isCheckRequest(sipIsErr))
			return checkToCpp(sipPy);

		AutoPtr<MapType> cpp_map = new MapType();
		if (!convertToCpp(cpp_map, sipPy, sipIsErr))
			return 0;
		*sipCppPtr = cpp_map.forget();

		return sipGetState(sipTransferObj);
	}

	PyObject *convertFromTypeCode(MapType *sipCpp)
	{
		PyObjectPtr sipPy = PyDict_New();
		if (sipPy == NULL)
			return NULL;
		if (!convertFromCpp(sipPy, sipCpp))
			return NULL;
		return sipPy.forget();
	}

 private:
	bool checkToCpp(PyObject *sipPy)
	{
		bool ok = PyDict_Check(sipPy);
		PyObject *key, *mval;
		SIP_SSIZE_T pos = 0;
		while (ok && PyDict_Next(sipPy, &pos, &key, &mval)) {
			try {
				Sip_KeyType.checkConvertToCpp(key);
				Sip_MappedType.checkConvertToCpp(mval);
			} catch (...) {
				ok = false;
			}
		}
		return ok;
	}

	bool convertToCpp(MapType *sipCpp, PyObject *sipPy, int *sipIsErr)
	{
		PyObject *key, *mval;
		SIP_SSIZE_T pos = 0;
		while (PyDict_Next(sipPy, &pos, &key, &mval)) {
			try {
				KeyToCpp kconv =
					Sip_KeyType.convertToCpp(key,
								 sipIsErr);
				KeyType *cpp_key = kconv.getCpp();
				MappedToCpp mconv =
					Sip_MappedType.convertToCpp(mval,
								    sipIsErr);
				MappedType *cpp_mval = mconv.getCpp();
				ValueType map_value(*cpp_key, *cpp_mval);
				if (!sipCpp->insert(map_value).second)
					throw exception();
			} catch (...) {
				return false;
			}
		}
		return true;
	}

	bool convertFromCpp(PyObject *sipPy, MapType *sipCpp)
	{
		typename MapType::iterator it, end = sipCpp->end();
		for (it = sipCpp->begin(); it != end; ++it) {
			AutoPtr<KeyType> new_key = new KeyType(it->first);
			AutoPtr<MappedType> new_mval = new MappedType(it->second);
			try {
				KeyFromCpp kconv =
					Sip_KeyType.convertFromCpp(new_key);
				new_key.forget();
				PyObjectPtr key = kconv.getPyObject();
				MappedFromCpp mconv =
					Sip_MappedType.convertFromCpp(new_mval);
				new_mval.forget();
				PyObjectPtr mval = mconv.getPyObject();
				if (PyDict_SetItem(sipPy, key, mval) != 0)
					throw exception();
			} catch (...) {
				return false;
			}
		}
		return true;
	}


	KeySipType Sip_KeyType;
	MappedSipType Sip_MappedType;
};

#endif // __SLS_DETECTOR_SIP_H

