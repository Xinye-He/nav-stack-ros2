float32 pre_speed_kmh        # 预速度：0/4/8/1(-4) (或其他档)
float32 angle_deg            # 方向档：-20/-10/0/10/20 (正为右转)
float32 dist_to_target_m     # 当前到目标/下一个路点的距离

bool   pick                  # 拾取状态位
bool   unload                # 卸货状态位
bool   dump                  # dump 状态位
bool   pick_action           # 触发捡拾动作位

bool   valid                 # 是否为有效命令
