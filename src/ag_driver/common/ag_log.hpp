
#pragma once

#include <iostream>

#define AG_ERROR   std::cout << "\033[1m\033[31m"  // bold red
#define AG_WARNING std::cout << "\033[1m\033[33m"  // bold yellow
#define AG_INFO    std::cout << "\033[1m\033[32m"  // bold green
#define AG_INFOL   std::cout << "\033[32m"         // green
#define AG_DEBUG   std::cout << "\033[1m\033[36m"  // bold cyan
#define AG_REND    "\033[0m" << std::endl

#define AG_TITLE   std::cout << "\033[1m\033[35m"  // bold magenta
#define AG_MSG     std::cout << "\033[1m\033[37m"  // bold white

