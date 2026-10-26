#ifndef _LQR_H
#define _LQR_H

#include "chassis.h"
#include "balance.h"

void LQR_calc(ten_balance_observation *observe,ten_balance_target *balance_targets,four_control_data *control_datas);

#endif
