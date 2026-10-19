// A fixture for the invariants script: the network library, unlike the core, may read the clock and
// include socket headers.
#include <sys/socket.h>

#include <chrono>

void wait() { (void)std::chrono::steady_clock::now(); }
