#include "UserParameters.h"
#include "config.h"

#if USER_PARAMS_ENABLED
// "USR" + 13 chars remaining for param name
const AP_Param::GroupInfo UserParameters::var_info[] = {

    // @Param: _XT_ID
    // @DisplayName: XT system id
    // @Description: Determin where to auto. 0 means no auto.
    // @Range: 0 255
    // @User: Standard
    AP_GROUPINFO("_XT_ID", 0, UserParameters, _xt_id, 0),

    // @Param: _XT_WET
    // @DisplayName: Print weight data on GCS.
    // @Description: Determin wether print weight data or not.0 means disable.
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("_XT_WET", 1, UserParameters, _enPrtWET, 0),

    AP_GROUPEND
};

UserParameters::UserParameters()
{
    AP_Param::setup_object_defaults(this, var_info);
}
#endif // USER_PARAMS_ENABLED
