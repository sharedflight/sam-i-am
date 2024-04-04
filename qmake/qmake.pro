# CDDL HEADER START
#
# This file and its contents are supplied under the terms of the
# Common Development and Distribution License ("CDDL"), version 1.0.
# You may only use this file in accordance with the terms of version
# 1.0 of the CDDL.
#
# A full copy of the text of the CDDL should have accompanied this
# source.  A copy of the CDDL is also available via the Internet at
# http://www.illumos.org/license/CDDL.
#
# CDDL HEADER END

# Copyright 2020 Justin Snapp. All rights reserved.

# Shared library without any Qt functionality
TEMPLATE = lib
QT -= gui core

CONFIG += plugin debug
CONFIG += warn_off 

QMAKE_APPLE_DEVICE_ARCHS = arm64 x86_64

#Saso suggested the nholomon json requires exceptions... (on mac only?)
#CONFIG -= thread exceptions qt rtti
CONFIG -= thread qt rtti

VERSION = 1.0.0

message("libacfutils path: " $$[LIBACFUTILS])
message("SAMIAM_VERSION set: " $$[SAMIAM_VERSION])

INCLUDEPATH += $$[LIBACFUTILS]/SDK/CHeaders/XPLM
INCLUDEPATH += $$[LIBACFUTILS]/SDK/CHeaders/Widgets
INCLUDEPATH += $$[LIBACFUTILS]/src
INCLUDEPATH += $$[LIBACFUTILS]/acf_apis
INCLUDEPATH += $$[LIBACFUTILS]/glew/include
INCLUDEPATH += $$[LIBACFUTILS]/OpenAL/include
INCLUDEPATH += $${LIBACFUTILS}/cglm/cglm-0.7.9/include

INCLUDEPATH += ../mathc
INCLUDEPATH += ../libquat/src
INCLUDEPATH += ../tinyxml2/

INCLUDEPATH += ../src/

QMAKE_CFLAGS += -std=c11 -g -fvisibility=hidden \
    -mno-ms-bitfields \
#    -W -Wall -Wextra 

# CONCERN: SUPRRESS THESE FOR NOW...
QMAKE_CFLAGS -= -Wunused_parameter -Wno-unused-local-typedefs -Wunused-result \
                -Wmissing-field-initializers

QMAKE_CXXFLAGS += -std=c++17
CONFIG += c++1z

QMAKE_MACOSX_DEPLOYMENT_TARGET = 10.15

# Make sure to disable Qmake's own warnings system, because it overrides
# our warning flags. This breaks CTASSERT, which relies on an unused local
# typedef.
QMAKE_CFLAGS_WARN_ON -= -W -Wall -fPIC
QMAKE_CXXFLAGS_WARN_ON -= -W -Wall -fPIC

# _GNU_SOURCE needed on Linux for getline()
# DEBUG - used by our ASSERT macro
# _FILE_OFFSET_BITS=64 to get 64-bit ftell and fseek on 32-bit platforms.
DEFINES += DEBUG _FILE_OFFSET_BITS=64

# Latest X-Plane APIs. No legacy support needed.
DEFINES += XPLM200 XPLM210 XPLM300 XPLM301 XPLM302 GLEW_BUILD=GLEW_STATIC XPLM_DEPRECATED

# Defines needs to libquat / mathc
DEFINES += MATHC_NO_INT 
DEFINES += MATHC_USE_DOUBLE_FLOATING_POINT
#DEFINES += MATHC_USE_LONG_DOUBLE_FLOATING_POINT
DEFINES += MATHC_USE_UNIONS 
#DEFINES += _USE_MATH_DEFINES

# Some older MinGW builds didn't define M_PI in math.h, so
# to cope with that, we define them all here:
DEFINES += M_PI=3.14159265358979323846
DEFINES += M_PI_2=1.57079632679489661923
DEFINES += M_PI_4=0.785398163397448309616
DEFINES += M_1_PI=0.318309886183790671538
DEFINES += M_2_PI=0.636619772367581343076
DEFINES += M_2_SQRTPI=1.12837916709551257390


SAMIAM_VERSION = $$[SAMIAM_VERSION]
isEmpty(SAMIAM_VERSION) {
	SAMIAM_VERSION = 'experimental'
	message("Setting SAMIAM_VERSION = experimental")
}
DEFINES += SOFTWARE_VERSION=\'\"$${SAMIAM_VERSION}\"\'
DEFINES += COMMIT_HASH_SHORT=\'\"$$system("git rev-parse --short HEAD")\"\'

# Make sure not trailing / on the url...


# Just a generally good idea not to depend on shipped libgcc.
!macx {
	LIBS += -static-libgcc
	LIBS += -lstdc++fs
}



QMAKE_CFLAGS += -Wno-missing-field-initializers

win32 {
	CONFIG += dll
	DEFINES += APL=0 IBM=1 LIN=0 _WIN32_WINNT=0x0600 _WIN32 \
				_USRDLL WIN32 WIN32_LEAN_AND_MEAN CURL_STATICLIB \
				SFCentralServerClientForFlightServer=1 \ 
				_GNU_SOURCE
    # LACF_HIDE_STAT_COMPAT is needed if we use sys/stat.h
	DEFINES += LACF_HIDE_STAT_COMPAT
	TARGET = win.xpl
	QMAKE_DEL_FILE = rm -f
	LIBS += -Wl,--exclude-libs,ALL
	QMAKE_LFLAGS_RELEASE =
}

win32:contains(CROSS_COMPILE, x86_64-w64-mingw32-) {
	QMAKE_CFLAGS += $$system("$$[LIBACFUTILS]/pkg-config-deps win-64 \
	    --static-openal --cflags")

	QMAKE_CXXFLAGS += $$system("$$[LIBACFUTILS]/pkg-config-deps win-64 \
	    --static-openal --cflags")   

	# This must go first for GCC to properly find dependent symbols
	LIBS += $$[LIBACFUTILS]/qmake/win64/libacfutils.a
	LIBS += $$system("$$[LIBACFUTILS]/pkg-config-deps win-64 \
	    --static-openal --libs")

	LIBS += -L$$[LIBACFUTILS]/SDK/Libraries/Win -lXPLM_64
	LIBS += -L$$[LIBACFUTILS]/SDK/Libraries/Win -lXPWidgets_64
	LIBS += -L$$[LIBACFUTILS]/GL_for_Windows/lib -lglu32 -lopengl32

	LIBS += -ldbghelp

	#LIBS += -L../SOIL/build-win32/
	#INCLUDEPATH += ../SOIL/include
}

unix:!macx {
	DEFINES += APL=0 IBM=0 LIN=1 CURL_STATICLIB _GNU_SOURCE
	TARGET = lin.xpl
	LIBS += -nodefaultlibs
	LIBS += -Wl,--exclude-libs,ALL
	LIBS += -lc_nonshared
}

linux-g++-64 {
	QMAKE_CFLAGS += $$system("$$[LIBACFUTILS]/pkg-config-deps linux-64 \
	    --static-openal --cflags")

	QMAKE_CXXFLAGS += $$system("$$[LIBACFUTILS]/pkg-config-deps linux-64 \
	    --static-openal --cflags")   

	LIBS += -L$$[LIBACFUTILS]/qmake/lin64 -lacfutils
	LIBS += $$system("$$[LIBACFUTILS]/pkg-config-deps linux-64 \
	    --static-openal --libs")


	#LIBS += -L../SOIL/build/
	#INCLUDEPATH += ../SOIL/include
}

macx {
	# Prevent linking via clang++ which makes us depend on libstdc++
	#QMAKE_LINK = $$QMAKE_CC
	QMAKE_CFLAGS += -mmacosx-version-min=10.15
	QMAKE_LFLAGS += -mmacosx-version-min=10.15

	DEFINES += APL=1 IBM=0 LIN=0 CURL_STATICLIB
	TARGET = mac.xpl
	LIBS += -F$$[LIBACFUTILS]/SDK/Libraries/Mac
	LIBS += -framework OpenGL -framework AudioToolbox
	LIBS += -framework CoreAudio -framework AudioUnit
	LIBS += -framework XPLM -framework XPWidgets
	
}

macx-clang {
	QMAKE_CFLAGS += $$system("$$[LIBACFUTILS]/pkg-config-deps mac-64 \
	    --static-openal --cflags")

	QMAKE_CXXFLAGS += $$system("$$[LIBACFUTILS]/pkg-config-deps mac-64 \
	    --static-openal --cflags")

	LIBS += $$[LIBACFUTILS]/qmake/mac64/libacfutils.a
	LIBS += $$system("$$[LIBACFUTILS]/pkg-config-deps mac-64 \
	    --static-openal --libs")

	#LIBS += -L../SOIL/build/
}

HEADERS += ../libquat/src/*.h
SOURCES += ../libquat/src/*.c

HEADERS += ../mathc/*.h
SOURCES += ../mathc/*.c

HEADERS += ../tinyxml2/tinyxml2.h
SOURCES += ../tinyxml2/tinyxml2.cpp

HEADERS += ../src/*.h
HEADERS += ../src/*.hpp
SOURCES += ../src/*.cpp