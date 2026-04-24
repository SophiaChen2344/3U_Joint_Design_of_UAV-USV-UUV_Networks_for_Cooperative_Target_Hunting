% 编队拓扑图模型：实现论文Eq.(32)-(33)的图论邻接矩阵/拉普拉斯矩阵
% 输入：
%   uav_num: 无人机数量
%   topology_type: 拓扑类型（固定拓扑，如"ring"/"star"）
% 输出：
%   L: 拉普拉斯矩阵（Eq.33）
%   A: 邻接矩阵（Eq.32）
function [L, A] = formation_graph(uav_num, topology_type)
    % 初始化邻接矩阵
    A = zeros(uav_num);
    switch topology_type
        case 'ring' % 环形拓扑（固定）
            for i = 1:uav_num
                A(i, mod(i, uav_num)+1) = 1;
                A(mod(i, uav_num)+1, i) = 1;
            end
        case 'star' % 星形拓扑（固定）
            A(1, 2:uav_num) = 1;
            A(2:uav_num, 1) = 1;
        otherwise
            error('Unsupported topology type: %s', topology_type);
    end
    
    % 论文Eq.(33): 拉普拉斯矩阵 L = D - A（D为度矩阵）
    D = diag(sum(A, 2));
    L = D - A;
end
