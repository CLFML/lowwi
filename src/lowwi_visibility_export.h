#ifndef LOWWI__VISIBILITY_CONTROL_H_
#define LOWWI__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef CLFML_LOWWI_CONDA_PACKAGING
#ifdef __GNUC__
#define LOWWI_EXPORT __attribute__((dllexport))
#define LOWWI_IMPORT __attribute__((dllimport))
#else
#define LOWWI_EXPORT __declspec(dllexport)
#define LOWWI_IMPORT __declspec(dllimport)
#endif
#ifdef LOWWI_BUILDING_DLL
#define LOWWI_PUBLIC LOWWI_EXPORT
#else
#define LOWWI_PUBLIC LOWWI_IMPORT
#endif
#else
#define LOWWI_PUBLIC
#endif
#else
#define LOWWI_PUBLIC
#endif

#endif // LOWWI__VISIBILITY_CONTROL_H_
