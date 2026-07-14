#pragma once

// 符号可见性控制: 编译 .so 时定义 TACTILE_BUILD_SHARED 导出符号, 用户使用时自动导入
#if defined(_WIN32) || defined(__CYGWIN__)
  #ifdef TACTILE_BUILD_SHARED
    #define TACTILE_API __declspec(dllexport)
  #else
    #define TACTILE_API __declspec(dllimport)
  #endif
#else
  #if __GNUC__ >= 4 || defined(__clang__)
    #define TACTILE_API __attribute__((visibility("default")))
  #else
    #define TACTILE_API
  #endif
#endif