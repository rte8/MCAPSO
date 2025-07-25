function accuracy = TestCnn(net,apop,ps_input)
    num_dim = 18;               % 特征维度

    %  设置变量存储数据
    P_test = apop(:,1:18);
    T_test = apop(:,end);

    %  数据转置
    P_test = P_test';
    T_test = T_test';

    %  得到训练集和测试样本个数
    N = size(P_test , 2);

    %  数据归一化
    P_test  = mapminmax('apply', P_test, ps_input);

    %  数据平铺
    p_test  =  double(reshape(P_test , num_dim, 1, 1, N));

    %  预测模型
    t_sim = predict(net, p_test ); 

    %  反归一化
    T_sim = vec2ind(t_sim');

    accuracy = sum((T_sim == T_test )) / N;       %  准确率
end