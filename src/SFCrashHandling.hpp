//
//  SFCrashHandling.hpp
//  SFCrashHandling
//
//  Created by Justin Snapp on 3/17/24.
//  Copyright © 2022 Justin Snapp. All rights reserved.
//

#ifndef SFCrashHandling_hpp
#define SFCrashHandling_hpp

// This option enables support for multi threaded crash handling.
// It requires C++11 or newer
#define SUPPORT_BACKGROUND_THREADS 1

#ifndef BUILD_FLIGHT_SERVER
extern "C" {
    #include "XPLMPlugin.h"
}

void set_plugin_id(XPLMPluginID plugin_id);
#endif 

// Registers the global crash handler. Should be called from XPluginStart
void register_crash_handler();

// Unregisters the global crash handler. You need to call this in XPluginStop so we can clean up after ourselves
void unregister_crash_handler();


#if SUPPORT_BACKGROUND_THREADS
// Registers the calling thread with the crash handler. We use this to figure out if a crashed thread belongs to us when we later try to figure out if we caused a crash
void register_thread_for_crash_handler();

// Unregisters the calling thread from the crash handler. MUST be called at the end of thread that was registered via register_thread_for_crash_handler()
void unregister_thread_from_crash_handler();

// A RAII helper class to register and unregister threads to participate in crash detection
class StThreadCrashCookie
{
public:
    StThreadCrashCookie();
    ~StThreadCrashCookie();
};
#endif

#endif /* SFCrashHandling_hpp */



