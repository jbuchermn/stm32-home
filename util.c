#include "util.h"

void halt(const char *msg) {
    static volatile const char *reason;
    reason = msg;
    (void)reason;
    __asm__ volatile("bkpt");
    while (1)
        ;
}

__attribute__((always_inline)) inline void assertm(bool condition,
                                                   const char *msg) {
    if (!__builtin_expect(condition, 1))
        halt(msg);
}
