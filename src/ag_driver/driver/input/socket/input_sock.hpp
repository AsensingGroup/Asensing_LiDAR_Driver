
#pragma once

#ifdef _WIN32

#include "ag_driver/driver/input/socket/input_sock_win.hpp"

#else  //__linux__

#include "ag_driver/driver/input/socket/input_sock_unix.hpp"

#endif

