#ifndef VISION_SAFEPTR_HPP
#define VISION_SAFEPTR_HPP

#include <mutex>
#include <stdexcept>

namespace Vision
{
/**
 * SafePtr
 *
 * Wrap a pointer of given type
 * with a mutex protection safety
 */
template <class T>
class SafePtr
{
public:
  /**
   * Initialization with the wrapped
   * reference and associated mutex lock
   * If lock is false, the mutex is assumed to
   * be already locked
   */
  SafePtr(T& ref, std::mutex& mutex, bool locked = false) : _mutex(mutex), _isLocked(false), _reference(ref)
  {
    // Acquire the mutex lock
    if (!locked)
    {
      _mutex.lock();
    }
    _isLocked = true;
  }

  /**
   * Release the lock at the end of scope
   */
  ~SafePtr()
  {
    if (_isLocked)
    {
      _mutex.unlock();
      _isLocked = false;
    }
  }

  /**
   * Forbid assignment
   */
  SafePtr<T>& operator=(const SafePtr<T>&) = delete;

  /**
   * Access to contained reference
   */
  inline T& operator*()
  {
    if (!_isLocked)
    {
      throw std::logic_error("SafePtr is unlock");
    }
    return _reference;
  }
  inline T* operator->()
  {
    if (!_isLocked)
    {
      throw std::logic_error("SafePtr is unlock");
    }
    return &_reference;
  }

  /**
   * Explicitly release the lock
   */
  inline void unlock()
  {
    if (_isLocked)
    {
      _mutex.unlock();
      _isLocked = false;
    }
    else
    {
      throw std::logic_error("SafePtr already unlock");
    }
  }

private:
  /**
   * The protection mutex
   * handling mutual exclusion
   */
  std::mutex& _mutex;

  /**
   * True if the mutex is locked
   */
  bool _isLocked;

  /**
   * The contained reference
   */
  T& _reference;
};
}  // namespace Vision

#endif
