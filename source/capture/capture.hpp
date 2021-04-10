#pragma once

#include "capture/base_capture.hpp"
#include "capture/laji_capture.hpp"

#ifdef MINDVISION
#    include "capture/mind_capture.hpp"

#elif DAHENG

#else
#    include "capture/dahua_capture.hpp"

#endif