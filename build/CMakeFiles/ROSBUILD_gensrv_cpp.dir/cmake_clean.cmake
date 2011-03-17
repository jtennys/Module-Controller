FILE(REMOVE_RECURSE
  "../src/module_controller/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/module_controller/SetServoPower.h"
  "../srv_gen/cpp/include/module_controller/SetServoAngle.h"
  "../srv_gen/cpp/include/module_controller/GetServoPower.h"
  "../srv_gen/cpp/include/module_controller/GetServoAngle.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
