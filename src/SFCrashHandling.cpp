//
//  SFCrashHandling.cpp
//  SFCrashHandling
//
//  Created by Justin Snapp on 3/17/24.
//  Copyright © 2022 Justin Snapp. All rights reserved.
//

#include "SFCrashHandling.hpp"

#include <cstring>
#include <cstdlib>

#if IBM
#include <mingw.thread.h>
#include <mingw.mutex.h>  
#endif 


#if IBM
#include <windows.h>
#include <dbghelp.h>
#include <psapi.h>
#else
#include <execinfo.h>   /* used for stack tracing */
#endif  /* !IBM */

#include <thread>

#if SUPPORT_BACKGROUND_THREADS
#include <atomic>
#include <set>
#include <mutex>
#endif

#if APL || LIN
#include <signal.h>
#include <execinfo.h>
#include <fcntl.h>
#include <unistd.h>
#endif

extern "C" {
    #include <acfutils/log.h>
    #include <acfutils/assert.h>
    #include <acfutils/thread.h>
}

#if SUPPORT_BACKGROUND_THREADS
static std::thread::id s_main_thread;
static std::atomic_flag s_thread_lock;
static std::set<std::thread::id> s_known_threads;
#endif


// Function called when we detect a crash that was caused by us

#if APL || LIN
static struct sigaction s_prev_sigsegv = {};
static struct sigaction s_prev_sigabrt = {};
static struct sigaction s_prev_sigfpe = {};
static struct sigaction s_prev_sigint = {};
static struct sigaction s_prev_sigill = {};
static struct sigaction s_prev_sigterm = {};

void handle_posix_sig(int sig, siginfo_t *siginfo, void *context);

#else // ! APL && !LIN


#define DATE_FMT    "%Y-%m-%d %H:%M:%S"
#define PREFIX_FMT  "%s %s[%s:%d]: ", timedate, log_prefix, filename, line

#define MAX_STACK_FRAMES    128
#define MAX_MODULES     1024
#define BACKTRACE_STR       "Backtrace is:\n"
#if defined(__GNUC__) || defined(__clang__)
#define BACKTRACE_STRLEN    __builtin_strlen(BACKTRACE_STR)
#else   /* !__GNUC__ && !__clang__ */
#define BACKTRACE_STRLEN    strlen(BACKTRACE_STR)
#endif  /* !__GNUC__ && !__clang__ */


/*
 * Since while dumping stack we are most likely in a fairly compromised
 * state, we statically pre-allocate these buffers to try and avoid having
 * to call into the VM subsystem.
 */
#define MAX_SYM_NAME_LEN    8192
#define MAX_BACKTRACE_LEN   (2 * 64 * 1024)
static char backtrace_buf[MAX_BACKTRACE_LEN] = { 0 };
static char symbol_buf[sizeof (SYMBOL_INFO) +
    MAX_SYM_NAME_LEN * sizeof (TCHAR)];
static char line_buf[sizeof (IMAGEHLP_LINE64)];
/* DbgHelp is not thread-safe, so avoid concurrency */
static mutex_t backtrace_lock;

static HMODULE modules[MAX_MODULES];
static MODULEINFO mi[MAX_MODULES];
static DWORD num_modules;

#define SYMNAME_MAXLEN  8191    /* C++ symbols can be HUUUUGE */


static LPTOP_LEVEL_EXCEPTION_FILTER prev_windows_except_handler = NULL;

LONG WINAPI handle_windows_exception(EXCEPTION_POINTERS *ei);

void write_backtrace_sw64(FILE *fp, PCONTEXT ctx);


#endif 

void handle_crash(void *context);

#if SUPPORT_BACKGROUND_THREADS
// Registers the calling thread with the crash handler. We use this to figure out if a crashed thread belongs to us when we later try to figure out if we caused a crash
void register_thread_for_crash_handler()
{
    while(s_thread_lock.test_and_set(std::memory_order_acquire))
    {}

    s_known_threads.insert(std::this_thread::get_id());
    
    s_thread_lock.clear(std::memory_order_release);
}

// Unregisters the calling thread from the crash handler. MUST be called at the end of thread that was registered via register_thread_for_crash_handler()
void unregister_thread_from_crash_handler()
{
    while(s_thread_lock.test_and_set(std::memory_order_acquire))
    {}

    s_known_threads.erase(std::this_thread::get_id());

    s_thread_lock.clear(std::memory_order_release);
}
#endif



static bool inited = false;


// Registers the global crash handler. Should be called from XPluginStart
void register_crash_handler()
{
    ASSERT(!inited);
    inited = B_TRUE;

#if SUPPORT_BACKGROUND_THREADS
    s_main_thread = std::this_thread::get_id();
#endif

#if APL || LIN

    struct sigaction sig_action; // = { .sa_sigaction = handle_posix_sig };
    sig_action.sa_sigaction = handle_posix_sig;
    sigemptyset(&sig_action.sa_mask);

#if LIN
    static uint8_t alternate_stack[32768];
    stack_t ss;
    ss.ss_sp = (void*)alternate_stack;
    ss.ss_size = 32768;
    ss.ss_flags = 0;

    sigaltstack(&ss, NULL);
    sig_action.sa_flags = SA_SIGINFO | SA_ONSTACK;
#else
    sig_action.sa_flags = SA_SIGINFO;
#endif

    sigaction(SIGSEGV, &sig_action, &s_prev_sigsegv);
    sigaction(SIGABRT, &sig_action, &s_prev_sigabrt);
    sigaction(SIGFPE, &sig_action, &s_prev_sigfpe);
    sigaction(SIGINT, &sig_action, &s_prev_sigint);
    sigaction(SIGILL, &sig_action, &s_prev_sigill);
    sigaction(SIGTERM, &sig_action, &s_prev_sigterm);

#else   /* !LIN && !APL */
    
    mutex_init(&backtrace_lock);

    prev_windows_except_handler =
        SetUnhandledExceptionFilter(handle_windows_exception);

#endif  /* !LIN && !APL */
}

// Unregisters the global crash handler. You need to call this in XPluginStop so we can clean up after ourselves
void unregister_crash_handler()
{
    if (!inited)
        return;
    inited = B_FALSE;

#if LIN || APL
    sigaction(SIGSEGV, &s_prev_sigsegv, NULL);
    sigaction(SIGABRT, &s_prev_sigabrt, NULL);
    sigaction(SIGFPE, &s_prev_sigfpe, NULL);
    sigaction(SIGINT, &s_prev_sigint, NULL);
    sigaction(SIGILL, &s_prev_sigill, NULL);
    sigaction(SIGTERM, &s_prev_sigterm, NULL);
#else   /* !LIN && !APL */
    SetUnhandledExceptionFilter(prev_windows_except_handler);
#endif  /* !LIN && !APL */
}

// A RAII helper class to register and unregister threads to participate in crash detection
StThreadCrashCookie::StThreadCrashCookie()
{
#if SUPPORT_BACKGROUND_THREADS
    register_thread_for_crash_handler();
#endif
}

StThreadCrashCookie::~StThreadCrashCookie()
{
#if SUPPORT_BACKGROUND_THREADS
    unregister_thread_from_crash_handler();
#endif
}

#ifndef BUILD_FLIGHT_SERVER
static XPLMPluginID s_my_plugin_id;

void set_plugin_id(XPLMPluginID plugin_id)
{
    s_my_plugin_id = plugin_id;
}

#endif

// Predicates that returns true if a thread is caused by us
// The main idea is to check the plugin ID if we are on the main thread,
// if not, we check if the current thread is known to be from us.
// Returns false if the crash was caused by code that didn't come from our plugin
bool is_us_executing()
{
#ifdef BUILD_FLIGHT_SERVER
    return true;
#else    
#if SUPPORT_BACKGROUND_THREADS
    const std::thread::id thread_id = std::this_thread::get_id();

    if(thread_id == s_main_thread)
    {
        // Check if the plugin executing is our plugin.
        // XPLMGetMyID() will return the ID of the currently executing plugin. If this is us, then it will return the plugin ID that we have previously stashed away
        return (s_my_plugin_id == XPLMGetMyID());
    }

    if(s_thread_lock.test_and_set(std::memory_order_acquire))
    {
        // We couldn't acquire our lock. In this case it's better if we just say it's not us so we don't eat the exception
        return false;
    }

    const bool is_our_thread = (s_known_threads.find(thread_id) != s_known_threads.end());
    s_thread_lock.clear(std::memory_order_release);

    return is_our_thread;
#else
    return (s_my_plugin_id == XPLMGetMyID());
#endif
#endif
    return false;
}

#if APL || LIN
void handle_posix_sig(int sig, siginfo_t *siginfo, void *context)
{
    if(is_us_executing())
    {
        static bool has_called_out = false;
        
        if(!has_called_out)
        {
            has_called_out = true;
            handle_crash((void *)sig);
        }
        
        abort();
    }

    // Forward the signal to the other handlers
#define FORWARD_SIGNAL(sigact) \
    do { \
        if((sigact)->sa_sigaction && ((sigact)->sa_flags & SA_SIGINFO)) \
            (sigact)->sa_sigaction(sig, siginfo, context); \
        else if((sigact)->sa_handler) \
            (sigact)->sa_handler(sig); \
    } while (0)
    
    switch(sig)
    {
        case SIGSEGV:
            FORWARD_SIGNAL(&s_prev_sigsegv);
            break;
        case SIGABRT:
            FORWARD_SIGNAL(&s_prev_sigabrt);
            break;
        case SIGFPE:
            FORWARD_SIGNAL(&s_prev_sigfpe);
            break;
        case SIGILL:
            FORWARD_SIGNAL(&s_prev_sigill);
            break;
        case SIGTERM:
            FORWARD_SIGNAL(&s_prev_sigterm);
            break;
    }
    
#undef FORWARD_SIGNAL
    
    abort();
}
#endif 

#if IBM
LONG WINAPI handle_windows_exception(EXCEPTION_POINTERS *ei)
{
    if(is_us_executing())
    {
        handle_crash(ei);
        return EXCEPTION_CONTINUE_SEARCH;
    }

    if(prev_windows_except_handler)
        return prev_windows_except_handler(ei);

    return EXCEPTION_CONTINUE_SEARCH;
}
#endif

static char backtrace_filename[100];
static struct tm *sTm;

void handle_crash(void *context)
{
#if APL || LIN
    // NOTE: This is definitely NOT production code
    // backtrace and backtrace_symbols are NOT signal handler safe and are just put in here for demonstration purposes
    // A better alternative would be to use something like libunwind here
    
    void *frames[64];
    int frame_count = backtrace(frames, 64);
    char **names = backtrace_symbols(frames, frame_count);
    char buf[50];

#ifdef BUILD_FLIGHT_SERVER
    time_t now = time (0);
    sTm = gmtime (&now);
    strftime (backtrace_filename, sizeof(backtrace_filename), "%Y-%m-%d-%H:%M:%S-backtrace.txt", sTm);
    const int fd = open(backtrace_filename, O_CREAT | O_RDWR | O_TRUNC | O_SYNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
#else
    const int fd = open("Log_SamIAm_Crash.txt", O_CREAT | O_RDWR | O_TRUNC | O_SYNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
#endif
    
    if(fd >= 0)
    {
        
        sprintf(buf, "Known threads: %d\n", s_known_threads.size());

        write(fd, buf, strlen(buf));

        for(int i = 0; i < frame_count; ++ i)
        {
            write(fd, names[i], strlen(names[i]));
            write(fd, "\n", 1);
        }

        close(fd);

        for(int i = 0; i < frame_count; ++ i)
        {
            logMsg("%s", names[i]);
        }
    }


#else

    EXCEPTION_POINTERS *ei = (EXCEPTION_POINTERS*) context;

    FILE *fp = fopen("Log_SamIAm_Crash.txt", "w"); 

    switch(ei->ExceptionRecord->ExceptionCode) {
        case EXCEPTION_ASSERTION_FAILED:
            fprintf( fp, "Caught EXCEPTION_ASSERTION_FAILED");
            break;
        case EXCEPTION_ACCESS_VIOLATION:
            fprintf( fp, "Caught EXCEPTION_ACCESS_VIOLATION");
            break;
        case EXCEPTION_ARRAY_BOUNDS_EXCEEDED:
            fprintf( fp, "Caught EXCEPTION_ARRAY_BOUNDS_EXCEEDED");
            break;
        case EXCEPTION_BREAKPOINT:
            fprintf( fp, "Caught EXCEPTION_BREAKPOINT");
            break;
        case EXCEPTION_DATATYPE_MISALIGNMENT:
            fprintf( fp, "Caught EXCEPTION_DATATYPE_MISALIGNMENT");
            break;
        case EXCEPTION_FLT_DENORMAL_OPERAND:
            fprintf( fp, "Caught EXCEPTION_FLT_DENORMAL_OPERAND");
            break;
        case EXCEPTION_FLT_DIVIDE_BY_ZERO:
            fprintf( fp, "Caught EXCEPTION_FLT_DIVIDE_BY_ZERO");
            break;
        case EXCEPTION_FLT_INEXACT_RESULT:
            fprintf( fp, "Caught EXCEPTION_FLT_INEXACT_RESULT");
            break;
        case EXCEPTION_FLT_INVALID_OPERATION:
            fprintf( fp, "Caught EXCEPTION_FLT_INVALID_OPERATION");
            break;
        case EXCEPTION_FLT_OVERFLOW:
            fprintf( fp, "Caught EXCEPTION_FLT_OVERFLOW");
            break;
        case EXCEPTION_FLT_STACK_CHECK:
            fprintf( fp, "Caught EXCEPTION_FLT_STACK_CHECK");
            break;
        case EXCEPTION_FLT_UNDERFLOW:
            logMsg("Caught EXCEPTION_FLT_UNDERFLOW");
            break;
        case EXCEPTION_ILLEGAL_INSTRUCTION:
            fprintf( fp, "Caught EXCEPTION_ILLEGAL_INSTRUCTION");
            break;
        case EXCEPTION_IN_PAGE_ERROR:
            fprintf( fp, "Caught EXCEPTION_IN_PAGE_ERROR");
            break;
        case EXCEPTION_INT_DIVIDE_BY_ZERO:
            fprintf( fp, "Caught EXCEPTION_INT_DIVIDE_BY_ZERO");
            break;
        case EXCEPTION_INT_OVERFLOW:
            fprintf( fp, "Caught EXCEPTION_INT_OVERFLOW");
            break;
        case EXCEPTION_INVALID_DISPOSITION:
            fprintf( fp, "Caught EXCEPTION_INVALID_DISPOSITION");
            break;
        case EXCEPTION_NONCONTINUABLE_EXCEPTION:
            fprintf( fp, "Caught EXCEPTION_NONCONTINUABLE_EXCEPTION");
            break;
        case EXCEPTION_PRIV_INSTRUCTION:
            fprintf( fp, "Caught EXCEPTION_PRIV_INSTRUCTION");
            break;
        case EXCEPTION_SINGLE_STEP:
            fprintf( fp, "Caught EXCEPTION_SINGLE_STEP");
            break;
        case EXCEPTION_STACK_OVERFLOW:
            fprintf( fp, "Caught EXCEPTION_STACK_OVERFLOW");
            break;
        default:
            fprintf( fp, "Caught unknown exception %lx",
                ei->ExceptionRecord->ExceptionCode);
            break;
    }
    
    fprintf( fp, "\n");
    fflush(fp);
    write_backtrace_sw64(fp, ei->ContextRecord);
    fflush(fp);
    fclose(fp);

#endif

}


#if IBM

static void
find_symbol(const char *filename, void *addr, char *symname,
    size_t symname_cap)
{
    /*
     * Note the `static' here is deliberate to cause these to become
     * BSS-allocated variables instead of stack-allocated. When parsing
     * through a stack trace we are in a pretty precarious state, so we
     * can't rely on having much stack space available.
     */
    static char symstxtname[MAX_PATH];
    static char prevsym[SYMNAME_MAXLEN + 1];
    static const char *sep;
    static FILE *fp;
    static void *prevptr = NULL;

    *symname = 0;
    *prevsym = 0;

    sep = strrchr(filename, DIRSEP);
    if (sep == NULL)
        return;
    lacf_strlcpy(symstxtname, filename, MIN((uintptr_t)(sep - filename) + 1,
        sizeof (symstxtname)));
    strncat(symstxtname, DIRSEP_S "syms.txt", sizeof (symstxtname));
    fp = fopen(symstxtname, "rb");
    if (fp == NULL)
        return;

    while (!feof(fp)) {
        static char unused_c;
        static void *ptr;
        static char sym[SYMNAME_MAXLEN + 1];

        if (fscanf(fp, "%p %c %" SCANF_STR_AUTOLEN(SYMNAME_MAXLEN) "s",
            &ptr, &unused_c, sym) != 3) {
            /*
             * This might fail if we hit a MASSIVE symbol name
             * which is longer than SYMNAME_MAXLEN. In that case,
             * we want to skip until the next newline.
             */
            int c;
            do {
                c = fgetc(fp);
            } while (c != '\n' && c != '\r' && c != EOF);
            if (c != EOF) {
                continue;
            } else {
                break;
            }
        }
        if (addr >= prevptr && addr < ptr) {
            snprintf(symname, symname_cap, "%s+%x", prevsym,
                (unsigned)((intptr_t )addr - (intptr_t )prevptr));
            break;
        }
        prevptr = ptr;
        lacf_strlcpy(prevsym, sym, sizeof (prevsym));
    }
    fclose(fp);
}

static HMODULE
find_module(LPVOID pc, DWORD64 *module_base)
{
    static DWORD i;
    for (i = 0; i < num_modules; i++) {
        static LPVOID start, end;
        start = mi[i].lpBaseOfDll;
        end = start + mi[i].SizeOfImage;
        if (start <= pc && end > pc) {
            *module_base = (DWORD64)start;
            return (modules[i]);
        }
    }
    *module_base = 0;
    return (NULL);
}

static void
gather_module_info(void)
{
    HANDLE process = GetCurrentProcess();

    EnumProcessModules(process, modules, sizeof (HMODULE) * MAX_MODULES,
        &num_modules);
    num_modules = MIN(num_modules, MAX_MODULES);
    for (DWORD i = 0; i < num_modules; i++)
        GetModuleInformation(process, modules[i], &mi[i], sizeof (*mi));
}

void
write_backtrace_sw64(FILE *fp, PCONTEXT ctx)
{
    
    static char filename[MAX_PATH];
    static DWORD64 pcs[MAX_STACK_FRAMES];
    static unsigned num_stack_frames;
    static STACKFRAME64 sf;
    static HANDLE process, thread;
    static DWORD machine;

    mutex_enter(&backtrace_lock);

    process = GetCurrentProcess();
    thread = GetCurrentThread();

    SymInitialize(process, NULL, TRUE);
    SymSetOptions(SYMOPT_LOAD_LINES);

    gather_module_info();

    memset(&sf, 0, sizeof (sf));
    sf.AddrPC.Mode = AddrModeFlat;
    sf.AddrStack.Mode = AddrModeFlat;
    sf.AddrFrame.Mode = AddrModeFlat;
#if defined(_M_IX86)
    machine = IMAGE_FILE_MACHINE_I386;
    sf.AddrPC.Offset = ctx->Eip;
    sf.AddrStack.Offset = ctx->Esp;
    sf.AddrFrame.Offset = ctx->Ebp;
#elif   defined(_M_X64)
    machine = IMAGE_FILE_MACHINE_AMD64;
    sf.AddrPC.Offset = ctx->Rip;
    sf.AddrStack.Offset = ctx->Rsp;
    sf.AddrFrame.Offset = ctx->Rbp;
#elif   defined(_M_IA64)
    machine = IMAGE_FILE_MACHINE_IA64;
    sf.AddrPC.Offset = ctx->StIIP;
    sf.AddrFrame.Offset = ctx->IntSp;
    sf.AddrBStore.Offset = ctx->RsBSP;
    sf.AddrBStore.Mode = AddrModeFlat;
    sf.AddrStack.Offset = ctx->IntSp;
#else
#error  "Unsupported architecture"
#endif  /* _M_X64 */

    for (num_stack_frames = 0; num_stack_frames < MAX_STACK_FRAMES;
        num_stack_frames++) {
        if (!StackWalk64(machine, process, thread, &sf, ctx, NULL,
            SymFunctionTableAccess64, SymGetModuleBase64, NULL)) {
            break;
        }
        pcs[num_stack_frames] = sf.AddrPC.Offset;
    }

    backtrace_buf[0] = '\0';
    lacf_strlcpy(backtrace_buf, BACKTRACE_STR, sizeof (backtrace_buf));

    for (unsigned i = 0; i < num_stack_frames; i++) {
        static int fill;
        static DWORD64 pc;
        static char symname[SYMNAME_MAXLEN + 1];
        static HMODULE module;
        static DWORD64 mbase;

        fill = strlen(backtrace_buf);
        pc = pcs[i];

        module = find_module((LPVOID)pc, &mbase);
        GetModuleFileNameA(module, filename, sizeof (filename));
        find_symbol(filename, (void *)(pc - mbase),
            symname, sizeof (symname));
        fill += snprintf(&backtrace_buf[fill],
            sizeof (backtrace_buf) - fill,
            "%d %p %s+%p (%s)\n", i, (void *)pc, filename,
            (void *)(pc - mbase), symname);
    }

    fprintf( fp, backtrace_buf);
    fflush(fp);
    fprintf( fp, "\nEnd of Backtrace\n");
    fflush(fp);

    fputs(backtrace_buf, stderr);
    fflush(stderr);
    SymCleanup(process);

    mutex_exit(&backtrace_lock);
}

#endif