import json
import os
import matlab.engine  # 需安装MATLAB Engine for Python

def load_config(config_path):
    """加载仿真配置"""
    with open(config_path, 'r') as f:
        return json.load(f)

def run_nominal_fixed_topology():
    """运行标称固定拓扑控制（调用MATLAB引擎）"""
    # 加载配置
    config = load_config("../configs/sim_main.json")
    
    # 启动MATLAB引擎
    eng = matlab.engine.start_matlab()
    eng.cd(os.path.join(os.getcwd(), "../matlab"), nargout=0)
    
    # 传递参数到MATLAB（可选，替代MATLAB内置参数）
    eng.workspace['uav_num'] = config['uav_params']['uav_num']
    eng.workspace['sim_time'] = config['sim_params']['sim_time']
    
    # 运行MATLAB主脚本
    eng.run_fixed_topology_main(nargout=0)
    eng.quit()
    
    print("Nominal fixed topology control simulation completed!")

if __name__ == "__main__":
    run_nominal_fixed_topology()