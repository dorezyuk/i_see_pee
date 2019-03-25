#ifndef I_SEE_PEE_UTILS_HPP
#define I_SEE_PEE_UTILS_HPP

#include <cmath>
#include <algorithm>
#include <vector>

namespace i_see_pee {

template <typename T>
T cast_to_range (T _in, T _min, T _max) noexcept {
  static_assert(std::is_arithmetic<T>::value, "must be arithmetic");

  // todo maybe minmax it
  const auto min = std::min(_in, _max);
  return std::max(_min, min);
}

template <typename T, typename Func>
void maybe_insert(std::vector<T>& _in, const T& _data, Func _f){
  if(_in.size() == _in.capacity()){
    if(!_f(_data, _in.back())){
      return;
    }
    _in.pop_back();
  }
  // log n
  const auto pos = std::lower_bound(_in.begin(), _in.end(), _data, _f);
  _in.insert(pos, _data);
}

} // namespace i_see_pee


#endif //I_SEE_PEE_UTILS_HPP
