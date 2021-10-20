#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define TutorialLogging_DLLIMPORT __declspec(dllimport)
#  define TutorialLogging_DLLEXPORT __declspec(dllexport)
#  define TutorialLogging_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define TutorialLogging_DLLIMPORT __attribute__((visibility("default")))
#    define TutorialLogging_DLLEXPORT __attribute__((visibility("default")))
#    define TutorialLogging_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define TutorialLogging_DLLIMPORT
#    define TutorialLogging_DLLEXPORT
#    define TutorialLogging_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef TutorialLogging_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define TutorialLogging_DLLAPI
#  define TutorialLogging_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef TutorialLogging_EXPORTS
#    define TutorialLogging_DLLAPI TutorialLogging_DLLEXPORT
#  else
#    define TutorialLogging_DLLAPI TutorialLogging_DLLIMPORT
#  endif // TutorialLogging_EXPORTS
#  define TutorialLogging_LOCAL TutorialLogging_DLLLOCAL
#endif // TutorialLogging_STATIC