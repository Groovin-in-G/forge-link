// ForgeLink_context.h - which execution context the caller is in, ONE definition for the whole
// firmware. Header-only, static inline, so each caller gets it inlined into its own code and it
// adds no symbol (and so nothing new to tools/check-irq-ram.sh's IRQ-path walk).
//
// ⛔ WHY ONE DEFINITION. The hardware watchdog (ForgeLink_hwdog.cpp) and the N6 bulk driver
// (ForgeLink_bulk.cpp) both ask "is an exception active?", and two copies of that test that
// drift apart is the whole hazard: the watchdog's alarm-tick feed and the bulk driver's
// never-wait-in-handler-mode rule would then disagree about the same CPU state. They were two
// identical copies pinned by a test until 2026-09-24 (a follow-up from N6 step 3's review).
#pragma once
#include <stdint.h>
#include <stdbool.h>

static inline uint32_t fl_ipsr(void)
{
    uint32_t v;
    __asm volatile ("mrs %0, ipsr" : "=r" (v));
    return v;
}

// Handler mode: an exception is active, so IPSR carries its number (non-zero). Thread mode reads 0.
static inline bool fl_handler_context(void)
{
    return (fl_ipsr() & 0x1FFu) != 0;
}
