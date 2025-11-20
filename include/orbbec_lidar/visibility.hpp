#pragma once

#ifdef _WIN32
#define OB_EXPORT \
    __declspec(dllexport)  // Use for both exporting and importing on Windows
#else
#define OB_EXPORT   \
    __attribute__(( \
        visibility("default")))  // Use for both exporting and importing on
                                 // Unix-like platforms
#endif

#ifndef OB_NO_EXPORT
#define OB_NO_EXPORT \
    __attribute__((  \
        visibility("hidden")))  // Hidden visibility for Unix-like platforms
#endif

// Deprecated macro
#ifndef OB_DEPRECATED
#ifdef _WIN32
#define OB_DEPRECATED __declspec(deprecated)
#else
#define OB_DEPRECATED __attribute__((deprecated))
#endif
#endif

// Deprecated export and no-export macros
#define OB_DEPRECATED_EXPORT OB_EXPORT OB_DEPRECATED
#define OB_DEPRECATED_NO_EXPORT OB_NO_EXPORT OB_DEPRECATED
