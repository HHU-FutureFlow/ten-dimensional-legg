#ifndef _LQR_H
#define _LQR_H

#include "chassis.h"
#include "balance.h"

/* air_mask 按 GRF_AIR_LEFT/RIGHT 标记离地侧。空侧摆角目标为世界系竖直。 */
void LQR_calc(const ten_balance_observation *observe, const ten_balance_target *balance_targets,
              uint8_t air_mask, four_control_data *control_datas);

#endif
