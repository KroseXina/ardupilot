#include "AC_ADRC.h"
#include "AC_ADRCLogStructure.h"

/* ********ADRC基类函数******* */

float ADRC::fal(float error, float alpha, float delta)
{
    if(alpha <= 0 || alpha >= 1) {
        return error;
    }

    if(error < - delta || error > delta) {
        return sign(error) * powf(fabsf(error), alpha);
    }
    else {
        return error / (powf(delta, 1 - alpha));
    }
}

float ADRC::fhan(float e1, float e2, float r, float T)
{
    float d = r * sq(T);
    float a0 = T * e2;
    float y = e1 + a0;
    float a1 = sqrtf(d * (d + 8 * fabsf(y)));
    float a2 = a0 + sign(y) * 0.5f * (a1 - d);
    float Sy = (sign(y + d) - sign(y - d)) * 0.5f;
    float a = (a0 + y - a2) * Sy + a2;
    float Sa = (sign(a + d) - sign(a - d)) * 0.5f;

    return -r * (a / d - sign(a)) * Sa - r * sign(a);
}


/* ************角速度环ADRC********** */

const AP_Param::GroupInfo ADRC_ANGLE_RATE::var_info[] = {
    // @Param: _WC
    // @DisplayName: ADRC Control Bandwidth
    // @Description: Controller bandwidth in rad/s
    // @Range: 1 20
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("_WC", 1, ADRC_ANGLE_RATE, _Wc, 10.0f),

    // @Param: _WO
    // @DisplayName: ADRC Observer Bandwidth
    // @Description: ESO bandwidth in rad/s
    // @Range: 5 100
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_WO", 2, ADRC_ANGLE_RATE, _Wo, 50.0f),

    // @Param: _Wtd
    // @DisplayName: ADRC TD Bandwidth
    // @Description: TD bandwidth in rad/s
    // @Range: 5 200
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_WTD", 3, ADRC_ANGLE_RATE, _Wtd, 50.0f),

    // @Param: _B0
    // @DisplayName: ADRC Input Gain
    // @Description: Estimated plant gain b0
    // @Range: 10 2000
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("_B0", 4, ADRC_ANGLE_RATE, _b0, 500.0f),

    AP_GROUPEND
};

ADRC_ANGLE_RATE::ADRC_ANGLE_RATE()
{
    AP_Param::setup_object_defaults(this, var_info);
}

float ADRC_ANGLE_RATE::update(float target, float measurement, float dt, float u_prev, bool enable_eso)
{
    if(!isfinite(target) || !isfinite(measurement) || !is_positive(dt))
        return 0.0f;

    if(enable_eso) {
        // 跟踪微分器
        update_td(target, dt);

        // 二阶线性ESO
        const float wo = _Wo.get();
        const float beta1 = 2.0f * wo;
        const float beta2 = sq(wo);
        const float error = measurement - _z1;

        _z1 += (_z2 + beta1 * error + _b0.get() * u_prev) * dt;
        _z2 += beta2 * error * dt;
    }
    else {
        reset(measurement);
        _x1 = target;
        _x2 = 0.0f;
    }

    // 控制律
    _ez1 = _x1 - _z1;
    _u0 = _Wc.get() * _ez1;
    float u = (_u0 - _z2) / _b0.get();

    return constrain_float(u, -1.0f, 1.0f);
}

void ADRC_ANGLE_RATE::update_td(float target, float dt)
{
    const float w = _Wtd.get();
    const float a = w * dt;
    const float e = expf(-a);

    const float x1 = _x1;
    const float x2 = _x2;

    // 离散TD
    _x1 = e * (1.0f + a) * x1 + e * dt * x2 + (1.0f - e * (1.0f + a)) * target;
    _x2 =-e * w * w * dt * x1 + e * (1.0f - a) * x2 + e * w * w * dt * target;
}

void ADRC_ANGLE_RATE::reset(float measurement)
{
    if (isfinite(measurement)) {
        _z1 = measurement;
        _z2 = 0.0f;
        _ez1 = 0.0f;
        _u0 = 0.0f;
    }
}

void ADRC_ANGLE_RATE::write_log(LogMessages id)
{
    //最多10hz记录
    if(AP_HAL::millis() - _log_timer >= 100) {
        const struct log_adrc pkt{
        LOG_PACKET_HEADER_INIT(id),
        time_us : AP_HAL::micros64(),
        z1      : _z1,
        z2      : _z2,
        ez1     : _ez1,
        u0      : _u0
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));

        _log_timer = AP_HAL::millis();
    }
}