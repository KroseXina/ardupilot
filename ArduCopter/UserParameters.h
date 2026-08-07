#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    // UserParameters属于g2子类. e.g g2.user_parameters.getxxxparam()
    AP_Int16 get_xt_id() const { return _xt_id; }
    AP_Int8 get_enPrtWET() const {return _enPrtWET;}

private:
    // Put your parameter variable definitions here
    AP_Int16 _xt_id;
    AP_Int8 _enPrtWET;
};
