
#ifndef I_SEE_PEE_MACROS_HPP
#define I_SEE_PEE_MACROS_HPP

#include <string>
#include <ros/console.h>

namespace i_see_pee{

inline std::string substring(const std::string& _in,
        const std::string& _first,
        const std::string& _last){
  const auto first = _in.find(_first);
  if(first == std::string::npos){
    return _in;
  }
  const auto last = first - _in.find(_last, first);
  return _in.substr(first, last);
}

} // namespace i_see_pee

#define __I_SEE_PEE_FUNCTION_NAME__ i_see_pee::substring(__PRETTY_FUNCTION__, "i_see_pee", "(")

#define I_SEE_PEE_DEBUG(args) ROS_DEBUG_STREAM(__I_SEE_PEE_FUNCTION_NAME__ << ": " << args)
#define I_SEE_PEE_INFO(args) ROS_INFO_STREAM(__I_SEE_PEE_FUNCTION_NAME__ << ": " << args)
#define I_SEE_PEE_WARN(args) ROS_WARN_STREAM(__I_SEE_PEE_FUNCTION_NAME__ << ": " << args)

#endif //I_SEE_PEE_MACROS_HPP
