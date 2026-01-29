#include "logging.h"
#include <cstdarg>
#include <cstdio>

void log_error(const char *fmt, ...) {
    std::fprintf(stderr, "[error] ");
    va_list args;
    va_start(args, fmt);
    std::vfprintf(stderr, fmt, args);
    va_end(args);
    std::fprintf(stderr, "\n");
}