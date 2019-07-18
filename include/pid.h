#ifndef __pid_H__
#define __pid_H__

enum
{
  LAST  = 0,
  NOW   = 1,
};

/**
  * @brief     PID 结构体
  */
typedef struct
{
  /* p、i、d参数 */
  double p;
  double i;
  double d;

  /* 目标值、反馈值、误差值 */
  double set;
  double get;
  double err[2];

  /* p、i、d各项计算出的输出 */
  double pout; 
  double iout; 
  double dout; 

  /* pid公式计算出的总输出 */
  double out;

  /* pid最大输出限制  */
  double max_output;
  
  /* pid积分输出项限幅 */
  double integral_limit;
 
} pid_m;

/**
  * @brief     PID 初始化函数
  * @param[in] pid: PID 结构体
  * @param[in] max_out: 最大输出
  * @param[in] intergral_limit: 积分限幅
  * @param[in] kp/ki/kd: 具体 PID 参数
  */
void pid_init(pid_m *pid, double max_out, double integral_limit, \
              double kp, double ki, double kd);

/**
  * @brief     PID 参数复位函数
  * @param[in] pid: PID 结构体
  * @param[in] kp/ki/kd: 具体 PID 参数
  */
void pid_reset(pid_m *pid, double kp, double ki, double kd);

/**
  * @brief     PID 计算函数，使用位置式 PID 计算
  * @param[in] pid: PID 结构体
  * @param[in] get: 反馈数据
  * @param[in] set: 目标数据
  * @retval    PID 计算输出
  */
double pid_calc(pid_m *pid, double get, double set);

void pid_reset2(pid_m *pid);
#endif
