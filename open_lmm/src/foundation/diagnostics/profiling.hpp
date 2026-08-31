#pragma once

#ifndef OPEN_LMM_ENABLE_TRACY
#define OPEN_LMM_ENABLE_TRACY 0
#endif
#ifndef OPEN_LMM_ENABLE_TIMING_LOG
#define OPEN_LMM_ENABLE_TIMING_LOG 1
#endif

#if OPEN_LMM_ENABLE_TRACY
#include <tracy/Tracy.hpp>
#define OPEN_LMM_ZONE() ZoneScoped
#define OPEN_LMM_ZONE_N(name) ZoneScopedN(name)
#define OPEN_LMM_ZONE_TEXT(text) ZoneText((text).data(), (text).size())
#define OPEN_LMM_PLOT(name, value) TracyPlot((name), static_cast<double>(value))
#define OPEN_LMM_FRAME_MARK() FrameMark
#define OPEN_LMM_THREAD_NAME(name) tracy::SetThreadName(name)
#else
#define OPEN_LMM_ZONE() ((void)0)
#define OPEN_LMM_ZONE_N(...) ((void)0)
#define OPEN_LMM_ZONE_TEXT(...) ((void)0)
#define OPEN_LMM_PLOT(...) ((void)0)
#define OPEN_LMM_FRAME_MARK() ((void)0)
#define OPEN_LMM_THREAD_NAME(...) ((void)0)
#endif
