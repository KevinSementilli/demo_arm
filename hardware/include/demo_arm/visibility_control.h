#ifndef DEMO_ARM__VISIBILITY_CONTROL_H_
#define DEMO_ARM__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DEMO_ARM_EXPORT __attribute__((dllexport))
#define DEMO_ARM_IMPORT __attribute__((dllimport))
#else
#define DEMO_ARM_EXPORT __declspec(dllexport)
#define DEMO_ARM_IMPORT __declspec(dllimport)
#endif
#ifdef DEMO_ARM_BUILDING_DLL
#define DEMO_ARM_PUBLIC DEMO_ARM_EXPORT
#else
#define DEMO_ARM_PUBLIC DEMO_ARM_IMPORT
#endif
#define DEMO_ARM_PUBLIC_TYPE DEMO_ARM_PUBLIC
#define DEMO_ARM_LOCAL
#else
#define DEMO_ARM_EXPORT __attribute__((visibility("default")))
#define DEMO_ARM_IMPORT
#if __GNUC__ >= 4
#define DEMO_ARM_PUBLIC __attribute__((visibility("default")))
#define DEMO_ARM_LOCAL __attribute__((visibility("hidden")))
#else
#define DEMO_ARM_PUBLIC
#define DEMO_ARM_LOCAL
#endif
#define DEMO_ARM_PUBLIC_TYPE
#endif

#endif  // DEMO_ARM__VISIBILITY_CONTROL_H_