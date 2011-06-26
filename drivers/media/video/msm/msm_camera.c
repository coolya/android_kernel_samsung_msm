#if defined(CONFIG_MACH_CALLISTO)
#include "msm_camera_callisto.c"
#elif defined(CONFIG_MACH_BENI)
#include "msm_camera_beni.c"
#elif defined(CONFIG_MACH_TASS)
#include "msm_camera_tass.c"
#elif defined(CONFIG_MACH_LUCAS)
#include "msm_camera_lucas.c"
#elif defined(CONFIG_MACH_COOPER)
#include "msm_camera_cooper.c"
#else
#endif
