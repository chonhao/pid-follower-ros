#include "pid.h"

// static void abs_limit(double *a, double ABS_MAX)
// {
//   if (*a > ABS_MAX)
//       *a = ABS_MAX;
//   if (*a < -ABS_MAX)
//       *a = -ABS_MAX;
// }

// /**
//   * @brief     PID 初始化函数
//   * @param[in] pid: PID 结构体
//   * @param[in] max_out: 最大输出
//   * @param[in] intergral_limit: 积分限幅
//   * @param[in] kp/ki/kd: 具体 PID 参数
//   */
// void pid_init(pid_m *pid, double max_out, double integral_limit, \
//               double kp, double ki, double kd)
// {
//   pid->integral_limit = integral_limit;
//   pid->max_output     = max_out;

//   pid->p = kp;
//   pid->i = ki;
//   pid->d = kd;
// }

// /**
//   * @brief     PID 计算函数，使用位置式 PID 计算
//   * @param[in] pid: PID 结构体
//   * @param[in] get: 反馈数据
//   * @param[in] set: 目标数据
//   * @retval    PID 计算输出
//   */
// double pid_calc(pid_m *pid, double get, double set)
// {
//   pid->get = get;
//   pid->set = set;
//   pid->err[NOW] = set - get; 
  
//   pid->pout = pid->p * pid->err[NOW];
//   pid->iout += pid->i * pid->err[NOW];
//   pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);

//   abs_limit(&(pid->iout), pid->integral_limit);
//   pid->out = pid->pout + pid->iout + pid->dout;
//   abs_limit(&(pid->out), pid->max_output);

//   pid->err[LAST]  = pid->err[NOW];
//   return pid->out;
// }

/**
  * @brief     PID 参数复位函数
  * @param[in] pid: PID 结构体
  * @param[in] kp/ki/kd: 具体 PID 参数
  */
void pid_reset(pid_m *pid, double kp, double ki, double kd)
{
  pid->p = kp;
  pid->i = ki;
  pid->d = kd;
  
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out  = 0;
}

void pid_reset2(pid_m *pid){
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out  = 0;
}
