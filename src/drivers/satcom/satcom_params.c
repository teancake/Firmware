#include <systemlib/param/param.h>

/**
 * Satellite radio timeout
 *
 * @unit s
 * @min 0
 * @max 120
 * @group Satellite Comms
 */
PARAM_DEFINE_INT32(SATCOM_TIMEOUT, 0);


/**
 * Satellite radio read interval
 *
 * @unit s
 * @min 0
 * @max 300
 * @group Satellite Comms
 */
PARAM_DEFINE_INT32(SATCOM_READINT, 0);
