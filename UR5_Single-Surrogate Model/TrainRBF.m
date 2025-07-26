function [net,ps_input,ps_output] = TrainRBF(AP,sizepop)
    res = AP(1:sizepop,1:19);
    num_samples = size(res, 1);
    res(randperm(num_samples), :);
    % 检查并移除完全重复的数据行
    [~, unique_idx] = unique(res, 'rows', 'stable');
    res = res(unique_idx, :);
    
    P_train = res(:,1:18)';T_train = res(:,19)';
    
    %  数据归一化
    [p_train, ps_input] = mapminmax(P_train, 0, 1);
    [t_train, ps_output] = mapminmax(T_train, 0, 1);
    
    %  创建网络
    rbf_spread = 100;                       
    net = newrbe(p_train, t_train, rbf_spread);
end