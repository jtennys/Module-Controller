FILE(REMOVE_RECURSE
  "../src/module_controller/srv"
  "../srv_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/module_controller/srv/__init__.py"
  "../src/module_controller/srv/_SetServoPower.py"
  "../src/module_controller/srv/_SetServoAngle.py"
  "../src/module_controller/srv/_GetModuleTotal.py"
  "../src/module_controller/srv/_GetModuleOffset.py"
  "../src/module_controller/srv/_GetArmTip.py"
  "../src/module_controller/srv/_GetModuleTwist.py"
  "../src/module_controller/srv/_GetServoPower.py"
  "../src/module_controller/srv/_GetServoAngle.py"
  "../src/module_controller/srv/_GetModuleLengths.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
