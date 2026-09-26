# Assumed trajectory for distributed MPC coordination
# 假定轨迹用于分布式MPC协调避碰
# 存储编队偏移向量序列 δ = [δ_pos, δ_vel, δ_acc] (9维状态 × N+1时间步)

Header header

# UAV identifier
uint8 uav_id

# Trajectory dimensions
uint32 num_states      # 状态维度 (固定为9: px,py,pz,vx,vy,vz,ax,ay,az)
uint32 num_timesteps   # 时间步数 (N+1)

# Flattened state trajectory in column-major order
# 展平的状态轨迹 (列优先顺序: nq×(N+1) → [col0, col1, ..., colN])
# states[i*num_states + j] = state_j at timestep_i
float64[] states

# Terminal rest position c (exploration algorithm). Length nq (chain-padded);
# empty for the legacy algorithm, which carries no rest position.
float64[] rest_position

# Trajectory validity flag
bool valid
