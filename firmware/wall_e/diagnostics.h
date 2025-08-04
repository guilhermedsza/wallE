#pragma once
#include "../config/build_flags.h"

#if ENABLE_DIAGS
  void runEyesTest();
  void runHeadTest();
  void runNeckTopTest();
  void runNeckBottomTest();
  void runArmsTest();
#else
  inline void runEyesTest()      {}
  inline void runHeadTest()      {}
  inline void runNeckTopTest()   {}
  inline void runNeckBottomTest(){}
  inline void runArmsTest()      {}
#endif