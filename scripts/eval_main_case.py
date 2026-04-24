import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import os

def load_sim_results(result_path):
    """加载MATLAB仿真结果"""
    data = sio.loadmat(result_path)
    sim_log = data['sim_log']
    return {
        "time": sim_log['time'][0][0].flatten(),
        "x": sim_log['x'][0][0],
        "e_form": sim_log['e_form'][0][0],
        "d_hat": sim_log['d_hat'][0][0]
    }

def evaluate_fixed_topology():
    """评估固定拓扑仿真结果"""
    # 加载结果
    result_path = "../outputs/fixed_topology_sim_log.mat"
    if not os.path.exists(result_path):
        raise FileNotFoundError("Simulation results not found! Run MATLAB script first.")
    res = load_sim_results(result_path)
    
    # 计算关键指标
    final_error = np.linalg.norm(res['e_form'][:, :, -1])
    avg_disturbance_error = np.mean(np.linalg.norm(res['d_hat'] - np.tile([0.5, 0.3, 0], (4, res['time'].shape[0])).T, axis=0))
    convergence_time = np.where(np.linalg.norm(res['e_form'], axis=0) < 0.01)[0][0] * 0.01  # 误差<0.01的收敛时间
    
    # 打印评估结果
    print("=== Fixed Topology Simulation Evaluation ===")
    print(f"Final formation error: {final_error:.4f} m")
    print(f"Average disturbance estimation error: {avg_disturbance_error:.4f} N")
    print(f"Convergence time (error < 0.01): {convergence_time:.2f} s")
    
    # 可视化评估
    plt.figure(figsize=(12, 8))
    # 编队误差
    plt.subplot(2,2,1)
    plt.plot(res['time'], np.linalg.norm(res['e_form'], axis=0))
    plt.xlabel('Time (s)'); plt.ylabel('Formation Error Norm (m)')
    plt.title('Formation Error Convergence'); plt.grid(True)
    
    # 单无人机位置
    plt.subplot(2,2,2)
    for i in range(4):
        plt.plot(res['x'][0, i, :], res['x'][1, i, :], label=f'UAV {i+1}')
    plt.xlabel('X (m)'); plt.ylabel('Y (m)'); plt.legend(); plt.grid(True)
    plt.title('UAV Trajectory'); plt.axis('equal')
    
    # 扰动估计误差
    plt.subplot(2,2,3)
    d_true = np.tile([0.5, 0.3, 0], (4, res['time'].shape[0])).T
    plt.plot(res['time'], np.linalg.norm(res['d_hat'] - d_true, axis=0))
    plt.xlabel('Time (s)'); plt.ylabel('Disturbance Estimation Error (N)')
    plt.title('USDE Performance'); plt.grid(True)
    
    # 收敛速度统计
    plt.subplot(2,2,4)
    error_norm = np.linalg.norm(res['e_form'], axis=0)
    plt.hist(error_norm, bins=20); plt.xlabel('Error Norm (m)'); plt.ylabel('Count')
    plt.title('Error Distribution'); plt.grid(True)
    
    plt.tight_layout()
    plt.savefig("../outputs/evaluation_results.png")
    plt.show()

if __name__ == "__main__":
    evaluate_fixed_topology()