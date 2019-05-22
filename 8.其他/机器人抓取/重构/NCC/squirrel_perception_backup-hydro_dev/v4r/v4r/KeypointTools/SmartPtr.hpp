/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_SMART_PTR_HPP
#define KP_SMART_PTR_HPP

#include <stdlib.h>

// exchange-add operation for atomic operations
#if defined __INTEL_COMPILER && !(defined WIN32 || defined _WIN32)   // atomic increment on the linux version
  #define KP_XADD(addr,delta) _InterlockedExchangeAdd(const_cast<void*>(reinterpret_cast<volatile void*>(addr)), delta)
#elif defined __GNUC__

  #if defined __clang__ && __clang_major__ >= 3 && !defined __ANDROID__
    #ifdef __ATOMIC_SEQ_CST
        #define KP_XADD(addr, delta) __c11_atomic_fetch_add((_Atomic(int)*)(addr), (delta), __ATOMIC_SEQ_CST)
    #else
        #define KP_XADD(addr, delta) __atomic_fetch_add((_Atomic(int)*)(addr), (delta), 5)
    #endif
  #elif __GNUC__*10 + __GNUC_MINOR__ >= 42

    #if !(defined WIN32 || defined _WIN32) && (defined __i486__ || defined __i586__ || \
        defined __i686__ || defined __MMX__ || defined __SSE__  || defined __ppc__) || \
        (defined __GNUC__ && defined _STLPORT_MAJOR)
      #define KP_XADD __sync_fetch_and_add
    #else
      #include <ext/atomicity.h>
      #define KP_XADD __gnu_cxx::__exchange_and_add
    #endif

  #else
    #include <bits/atomicity.h>
    #if __GNUC__*10 + __GNUC_MINOR__ >= 34
      #define KP_XADD __gnu_cxx::__exchange_and_add
    #else
      #define KP_XADD __exchange_and_add
    #endif
  #endif

#elif defined WIN32 || defined _WIN32 || defined WINCE
  namespace kp { CV_EXPORTS int _interlockedExchangeAdd(int* addr, int delta); }
  #define KP_XADD kp::_interlockedExchangeAdd

#else
  static inline int KP_XADD(int* addr, int delta)
  { int tmp = *addr; *addr += delta; return tmp; }
#endif


namespace kp 
{

template<typename T> class SmartPtr
{
public:
    SmartPtr();
    /* take ownership of the pointer and set reference counter to 1 */
    SmartPtr(T* _obj);
    ~SmartPtr();
    /* copies calls increment reference counter */
    SmartPtr(const SmartPtr& ptr);
    template<typename T2> SmartPtr(const SmartPtr<T2>& ptr);
    /* copy operator */
    SmartPtr& operator = (const SmartPtr& ptr);
    /* increment the reference counter */
    void addref();
    /* decrement the reference counter and delete object in case the counter is 0 */
    void release();
    /* deletes the object */
    void delete_obj();
    /* returns true iff obj==NULL */
    bool empty() const;
    /* access the object address */
    T* get();
    const T* get() const;
    /* take ownership of a new object */
    void reset(T* _obj);

    /* cast pointer to another type */
    template<typename T2> SmartPtr<T2> ptr();
    template<typename T2> const SmartPtr<T2> ptr() const;

    /* access operators */
    T* operator -> ();
    const T* operator -> () const;

    operator T* ();
    operator const T*() const;

    T* obj; 
    int* refcount;
};

/**
 * Constructor/ destructor
 */
template<typename T> inline SmartPtr<T>::SmartPtr() : obj(0), refcount(0) 
{
}

template<typename T> inline SmartPtr<T>::SmartPtr(T* _obj) : obj(_obj)
{

  if(obj)
  {
    refcount = (int*)malloc(sizeof(*refcount));
  
    *refcount = 1;
  }
  else
    refcount = 0;
}

template<typename T> inline SmartPtr<T>::~SmartPtr() 
{ 
  release(); 
}



/**
 * addref
 */
template<typename T> inline void SmartPtr<T>::addref()
{ 
  if(refcount) KP_XADD(refcount,1); 
}

/**
 * release
 */
template<typename _Tp> inline void SmartPtr<_Tp>::release()
{
  if( refcount && KP_XADD(refcount, -1) == 1 )
  {
    delete_obj();
    free(refcount);
  }
  refcount = 0;
  obj = 0;
}

/**
 * delete_obj
 */
template<typename T> inline void SmartPtr<T>::delete_obj()
{
  if( obj ) delete obj;
}

/**
 * copy constructor
 */
template<typename T> inline SmartPtr<T>::SmartPtr(const SmartPtr<T> &_ptr)
{
  obj = _ptr.obj;
  refcount = _ptr.refcount;
  addref();
}

/**
 * operators
 */
template<typename T> inline SmartPtr<T>& SmartPtr<T>::operator=(const SmartPtr<T> &_ptr)
{
  int *_refcount = _ptr.refcount;
  if( _refcount ) KP_XADD(_refcount, 1);
  release();
  obj = _ptr.obj;
  refcount = _refcount;
  return *this;
}

template<typename T> inline T* SmartPtr<T>::operator -> () 
{ 
  return obj; 
}

template<typename T> inline const T* SmartPtr<T>::operator -> () const 
{ 
  return obj; 
}

template<typename T> inline SmartPtr<T>::operator T* () 
{ 
  return obj; 
}

template<typename T> inline SmartPtr<T>::operator const T*() const 
{ 
  return obj; 
}

template<typename T> inline bool SmartPtr<T>::empty() const 
{ 
  return obj == 0; 
}

template<typename T> inline T* SmartPtr<T>::get()
{
  return obj;
}

template<typename T> const inline T* SmartPtr<T>::get() const
{
  return obj;
}

template<typename T> inline void SmartPtr<T>::reset(T* _obj)
{
  release();
  
  if(_obj)
  {
    obj = _obj;
    refcount = (int*)malloc(sizeof(*refcount));
    *refcount = 1;
  }
}

/**
 * type casts
 */
template<typename T> template<typename T2> SmartPtr<T>::SmartPtr(const SmartPtr<T2>& p)
  : obj(0), refcount(0)
{
  if (p.empty())
      return;

  T* p_casted = dynamic_cast<T*>(p.obj);
  if (!p_casted)
      return;

  obj = p_casted;
  refcount = p.refcount;
  addref();
}

template<typename T> template<typename T2> inline SmartPtr<T2> SmartPtr<T>::ptr()
{
  SmartPtr<T2> p;
  if( !obj )
      return p;

  T2* obj_casted = dynamic_cast<T2*>(obj);
  if (!obj_casted)
      return p;

  if( refcount ) KP_XADD(refcount, 1);

  p.obj = obj_casted;
  p.refcount = refcount;

  return p;
}

template<typename T> template<typename T2> inline const SmartPtr<T2> SmartPtr<T>::ptr() const
{
  SmartPtr<T2> p;
  if( !obj )
      return p;

  T2* obj_casted = dynamic_cast<T2*>(obj);
  if (!obj_casted)
      return p;

  if( refcount ) KP_XADD(refcount, 1);

  p.obj = obj_casted;
  p.refcount = refcount;

  return p;
}


} //--END--

#endif




