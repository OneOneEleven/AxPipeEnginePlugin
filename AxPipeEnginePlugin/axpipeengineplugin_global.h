#pragma once

#include <QtCore/qglobal.h>

#ifndef BUILD_STATIC
# if defined(AXPIPEENGINEPLUGIN_LIB)
#  define AXPIPEENGINEPLUGIN_EXPORT Q_DECL_EXPORT
# else
#  define AXPIPEENGINEPLUGIN_EXPORT Q_DECL_IMPORT
# endif
#else
# define AXPIPEENGINEPLUGIN_EXPORT
#endif
