function apop = TestCnn_1(net,pop,ps_input,net1,ps_input1)
    %  分析数据
    num_dim = 18;               % 特征维度

    %  设置变量存储数据
    P_test = pop;

    %  数据转置
    P_test = P_test';

    %  得到训练集和测试样本个数
    N = size(P_test , 2);

    %  数据归一化
    P_test  = mapminmax('apply', P_test, ps_input);
    P_test1  = mapminmax('apply', P_test, ps_input1);

    %  数据平铺
    p_test  =  double(reshape(P_test , num_dim, 1, 1, N));
    p_test1  =  double(reshape(P_test1 , num_dim, 1, 1, N));

    %  预测模型
    t_sim = predict(net, p_test); 
    t_sim1 = predict(net1, p_test1); 

    %  反归一化
    T_sim = vec2ind(t_sim');
    T_sim1 = vec2ind(t_sim1');

    apop = [pop,zeros(size(pop,1),1),T_sim',T_sim1'];
end