#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AC_PID/AC_PID.h>
#include <AP_Logger/AP_Logger.h>

/* 
ADRC基类
*/
class ADRC
{
public:

protected:
    float fal(float error, float alpha, float delta);   //非线性函数
    float fhan(float e1, float e2, float r, float h);   //最速跟踪函数

    template <typename T>
    int sign(T val) {
        return (T(0) < val) - (val < T(0));
    }
};

/* 
角速度ADRC类，一阶控制对象，二阶ESO
*/
class ADRC_ANGLE_RATE : public ADRC
{
public:
    ADRC_ANGLE_RATE();
    static const struct AP_Param::GroupInfo var_info[];

    void reset(float measurement);
    float update(float target, float measurement, float dt, float u_prev, bool enable_eso);
    void write_log(LogMessages id);

private:
    void update_td(float target, float dt);  //跟踪微分器

    //跟踪微分器输出
    float _x1{};  //角速度平滑
    float _x2{};  //角速度微分平滑，暂时不使用

    //ESO输出
    float _z1{};  //输入量估计
    float _z2{};  //总扰动估计

    //控制律输出
    float _ez1{}; //输入误差
    float _u0{};

    uint32_t _log_timer{};
    bool _eso_enabled{false};

    AP_Float _Wc;  //控制带宽
    AP_Float _Wo;  //ESO观测带宽,约为Wc的3~5倍
    AP_Float _b0;  //总体增益
    AP_Float _Wtd; //TD带宽,Wc的3～10倍,调节相位差
};