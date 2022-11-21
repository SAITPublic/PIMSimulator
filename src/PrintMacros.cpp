#include "PrintMacros.h"

/*
 * Enable or disable PRINT() statements.
 *
 * Set by flag in TraceBasedSim.cpp when compiling standalone DRAMSim tool.
 *
 * The DRAMSim libraries do not include the TraceBasedSim object and thus
 * library users can optionally override the weak definition below.
 */
int __attribute__((weak)) SHOW_SIM_OUTPUT = false;

bool DEBUG_TRANS_Q;
bool DEBUG_CMD_Q;
bool DEBUG_ADDR_MAP;
bool DEBUG_BANKSTATE;
bool DEBUG_BUS;
bool DEBUG_BANKS;
bool DEBUG_POWER;
bool DEBUG_CMD_TRACE;
bool DEBUG_PIM_TIME;
bool DEBUG_PIM_BLOCK;

bool USE_LOW_POWER;
bool VIS_FILE_OUTPUT;
bool PRINT_CHAN_STAT;

bool VERIFICATION_OUTPUT;
bool LOG_OUTPUT;

std::string SIM_TRACE_FILE;
