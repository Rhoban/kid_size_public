#ifndef VISION_PARAMSCONTAINER_HPP
#define VISION_PARAMSCONTAINER_HPP

#include <map>
#include <string>
#include <functional>
#include <type_traits>
#include <stdexcept>
#include <iostream>

namespace Vision
{
namespace ParamsContainerImpl
{
/**
 * Typedef for parameters callback on set
 */
template <class T>
using Callback = std::function<void(T&)>;

/**
 * Structure holding both the parameter value
 * (or pointer), dirty flag and its associated set callback
 */
template <class T>
struct Bag
{
  static_assert(!std::is_reference<T>::value, "ParamsContainer should not be a reference type");
  T value;
  bool isDirty;
  Callback<typename std::remove_pointer<T>::type> callback;
};

/**
 * Type trait testing is a type
 * is printable
 */
template <class T>
struct is_printable
{
  template <class U>
  constexpr static auto test(U* u) -> decltype(std::cout << *u, bool())
  {
    return (void)u, true;
  }
  constexpr static bool test(...)
  {
    return false;
  }
  constexpr static bool value = test((T*)0);
};

/**
 * Base container for a single given type T
 */
template <class T>
class Container
{
public:
  /**
   * Return a map representing only the name->value association
   */
  inline std::map<std::string, T> getMap() const
  {
    std::map<std::string, T> result;
    for (const auto& pair : _values)
    {
      result[pair.first] = pair.second.value;
    }
    return result;
  }

  /**
   * Define a new parameter by its name and default value
   */
  inline void define(const std::string& name, const T& value, Callback<T> callback)
  {
    if (exists(name))
    {
      throw std::logic_error("ParamsContainer already registered name '" + name + "'");
    }
    _values[name] = { value, false, callback };
  }

  /**
   * Define a new parameter by its name and its pointer
   */
  inline void define(const std::string& name, T* pointer, Callback<T> callback)
  {
    if (exists(name))
    {
      throw std::logic_error("ParamsContainer already registered name '" + name + "'");
    }
    if (pointer == nullptr)
    {
      throw std::logic_error("ParamsContainer null pointer");
    }
    _pointers[name] = { pointer, false, callback };
  }

  /**
   * Return the value of given parameter name
   */
  inline const T& get(const std::string& name) const
  {
    if (_values.count(name) != 0)
    {
      return _values.at(name).value;
    }
    else if (_pointers.count(name) != 0)
    {
      return *(_pointers.at(name).value);
    }
    else
    {
      throw std::logic_error("ParamsContainer unknown parameter: '" + name + "'");
    }
  }

  /**
   * Return the mutable reference of given
   * parameter name and set dirty flag
   */
  inline T& set(const std::string& name)
  {
    if (_values.count(name) != 0)
    {
      _values[name].isDirty = true;
      return _values[name].value;
    }
    else if (_pointers.count(name) != 0)
    {
      _pointers[name].isDirty = true;
      return *(_pointers[name].value);
    }
    else
    {
      throw std::logic_error("ParamsContainer unknown parameter: '" + name + "'");
    }
  }

  /**
   * Update the given parameter name with given
   * value and call immediately the callback if
   * it exists
   */
  inline void setNow(const std::string& name, const T& value)
  {
    if (_values.count(name) != 0)
    {
      _values[name].value = value;
      _values[name].callback(_values[name].value);
      _values[name].isDirty = false;
    }
    else if (_pointers.count(name) != 0)
    {
      *(_pointers[name].value) = value;
      _pointers[name].callback(*(_pointers[name].value));
      _pointers[name].isDirty = false;
    }
    else
    {
      throw std::logic_error("ParamsContainer unknown parameter '" + name + "'");
    }
  }

  /**
   * Return true if the given parameter name is registered
   */
  inline bool exists(const std::string& name) const
  {
    if (_values.count(name) == 0 && _pointers.count(name) == 0)
    {
      return false;
    }
    else
    {
      return true;
    }
  }

  /**
   * Check and call updated parameter callback
   */
  inline void checkDirty()
  {
    for (auto& it : _values)
    {
      if (it.second.isDirty)
      {
        it.second.callback(it.second.value);
        it.second.isDirty = false;
      }
    }
    for (auto& it : _pointers)
    {
      if (it.second.isDirty)
      {
        it.second.callback(*(it.second.value));
        it.second.isDirty = false;
      }
    }
  }

  /**
   * Print all contained parameters
   * (Using SFINAE for enabling print only
   * for type supporting ostream operator)
   */

  template <class U = void>
  inline typename std::enable_if<is_printable<T>::value, U>::type print(std::ostream& os) const
  {
    for (const auto& it : _values)
    {
      os << it.first << ": " << it.second.value << std::endl;
    }
    for (const auto& it : _pointers)
    {
      os << it.first << ": " << *(it.second.value) << std::endl;
    }
  }
  template <class U = void>
  inline typename std::enable_if<!is_printable<T>::value, U>::type print(std::ostream& os) const
  {
    for (const auto& it : _values)
    {
      os << it.first << ": [Non printable]" << std::endl;
    }
    for (const auto& it : _pointers)
    {
      os << it.first << ": [Non printable]" << std::endl;
    }
  }

private:
  /**
   * Typedef values and pointers container iterator
   */
  typedef typename std::map<std::string, Bag<T>>::iterator ValuesIt;
  typedef typename std::map<std::string, Bag<T>>::const_iterator ValuesItConst;
  typedef typename std::map<std::string, Bag<T*>>::iterator PointersIt;
  typedef typename std::map<std::string, Bag<T*>>::const_iterator PointersItConst;

  /**
   * Values and pointers parameter container
   * indexed by their name
   */
  std::map<std::string, Bag<T>> _values;
  std::map<std::string, Bag<T*>> _pointers;

public:
  unsigned int size() const
  {
    return _pointers.size() + _values.size();
  }

  /**
   * Const and non const Iterator for the Parameter Container
   */
  template <bool isConst>
  class Iterator
  {
  public:
    typedef typename std::conditional<!isConst, ValuesIt, ValuesItConst>::type ValsIt;
    typedef typename std::conditional<!isConst, PointersIt, PointersItConst>::type PtrsIt;
    typedef typename std::conditional<!isConst, T&, const T&>::type Value;

    typedef std::pair<const std::string&, Value> KeyValue;

    Iterator(ValsIt itVals, ValsIt itEndVals, PtrsIt itPtrs)
      : _isIteratingValues(true), _itVals(itVals), _itEndVals(itEndVals), _itPtrs(itPtrs)
    {
      if (_itVals == _itEndVals)
      {
        _isIteratingValues = false;
      }
    }

    template <class U = KeyValue>
    inline typename std::enable_if<isConst, U>::type operator*() const
    {
      if (_isIteratingValues)
      {
        return KeyValue(_itVals->first, _itVals->second.value);
      }
      else
      {
        return KeyValue(_itPtrs->first, *(_itPtrs->second.value));
      }
    }
    template <class U = KeyValue>
    inline typename std::enable_if<!isConst, U>::type operator*()
    {
      if (_isIteratingValues)
      {
        _itVals->second.isDirty = true;
        return KeyValue(_itVals->first, _itVals->second.value);
      }
      else
      {
        _itPtrs->second.isDirty = true;
        return KeyValue(_itPtrs->first, *(_itPtrs->second.value));
      }
    }

    inline Iterator& operator++()
    {
      if (_isIteratingValues)
      {
        _itVals++;
      }
      else
      {
        _itPtrs++;
      }
      if (_itVals == _itEndVals)
      {
        _isIteratingValues = false;
      }
      return *this;
    }

    inline bool operator!=(const Iterator& it) const
    {
      return _itVals != it._itVals || _itPtrs != it._itPtrs;
    }

  private:
    bool _isIteratingValues;
    ValsIt _itVals;
    ValsIt _itEndVals;
    PtrsIt _itPtrs;
  };

  /**
   * Return Iterator on registered parameters
   * as a std::map (use it.first and it.second)
   */
  inline Iterator<false> begin()
  {
    return Iterator<false>(_values.begin(), _values.end(), _pointers.begin());
  }
  inline Iterator<true> begin() const
  {
    return Iterator<true>(_values.begin(), _values.end(), _pointers.begin());
  }
  inline Iterator<false> end()
  {
    return Iterator<false>(_values.end(), _values.end(), _pointers.end());
  }
  inline Iterator<true> end() const
  {
    return Iterator<true>(_values.end(), _values.end(), _pointers.end());
  }
};
}  // namespace ParamsContainerImpl

/**
 * ParamsContainer
 *
 * A generic container parameters for variadic types
 * Provides define, iteration and get/set method
 */
template <class... Types>
class ParamsContainer : private ParamsContainerImpl::Container<ParamsContainer<Types...>>,
                        private ParamsContainerImpl::Container<Types>...
{
public:
  /**
   * Typedef for typed parameter container and
   * typed callback
   */
  template <class T>
  using Container = ParamsContainerImpl::Container<T>;
  template <class T>
  using Callback = ParamsContainerImpl::Callback<T>;

  /**
   * Define a new parameter (value) of given type by its name, and
   * optionally its default value and callback lambda
   */
  template <class T>
  inline void define(const std::string& name, const T& value = T(), Callback<T> callback = [](T&) {})
  {
    convert<T>().define(name, value, callback);
  }

  /**
   * Define a new parameter (pointer) of given type by its
   * name, pointer address and optionally
   * its callback lambda
   */
  template <class T>
  inline void define(const std::string& name, T* pointer, Callback<T> callback = [](T&) {})
  {
    convert<T>().define(name, pointer, callback);
  }

  /**
   * Read access to given parameter type and name
   */
  template <class T>
  inline const T& get(const std::string& name) const
  {
    return convert<T>().get(name);
  }

  /**
   * Write access to given parameter type and name
   * (The dirty flag is set)
   */
  template <class T>
  inline T& set(const std::string& name)
  {
    return convert<T>().set(name);
  }

  /**
   * Update the given parameter with given value
   * (Dirty callback is immediately called)
   */
  template <class T>
  inline void setNow(const std::string& name, const T& value)
  {
    convert<T>().setNow(name, value);
  }

  /**
   * Return true if the given parameter name exists
   * in the asked container
   */
  template <class T>
  inline bool exists(const std::string& name) const
  {
    return convert<T>().exists(name);
  }

  /**
   * Direct access to typed container
   * of given type
   */
  template <class T>
  inline const Container<T>& params() const
  {
    return convert<T>();
  }
  template <class T>
  inline Container<T>& params()
  {
    return convert<T>();
  }

  /**
   * Check for updated parameters and call
   * callback lambde function
   */
  inline void checkDirty()
  {
    checkDirtyImpl<Types...>::run(*this);
  }

  /**
   * Print all contained parameters
   */
  inline void print(std::ostream& os = std::cout)
  {
    printImpl<Types...>::run(os, *this);
  }

private:
  /**
   * Check and cast the current instance into
   * the given typed Container
   */
  template <class T>
  const Container<T>& convert() const
  {
    static_assert(std::is_base_of<Container<T>, ParamsContainer>::value, "ParamsContainer template type error");
    return static_cast<const Container<T>&>(*this);
  }
  template <class T>
  Container<T>& convert()
  {
    static_assert(std::is_base_of<Container<T>, ParamsContainer>::value, "ParamsContainer template type error");
    return static_cast<Container<T>&>(*this);
  }

  /**
   * Implementation of checkDirty on all
   * typed containers
   */
  template <class... Args>
  struct checkDirtyImpl;
  template <class T>
  struct checkDirtyImpl<T>
  {
    static inline void run(ParamsContainer<Types...>& self)
    {
      self.convert<T>().checkDirty();
    }
  };
  template <class T, class... Args>
  struct checkDirtyImpl<T, Args...>
  {
    static inline void run(ParamsContainer<Types...>& self)
    {
      self.convert<T>().checkDirty();
      checkDirtyImpl<Args...>::run(self);
    }
  };

  /**
   * Implementation of print on all
   * typed container
   */
  template <class... Args>
  struct printImpl;
  template <class T>
  struct printImpl<T>
  {
    static inline void run(std::ostream& os, ParamsContainer<Types...>& self)
    {
      self.convert<T>().print(os);
    }
  };
  template <class T, class... Args>
  struct printImpl<T, Args...>
  {
    static inline void run(std::ostream& os, ParamsContainer<Types...>& self)
    {
      printImpl<T>::run(os, self);
      printImpl<Args...>::run(os, self);
    }
  };
};
}  // namespace Vision

#endif
