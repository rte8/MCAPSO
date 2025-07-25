function [T_sim] = TestRBF(pop,net,ps_input,ps_output)
    P_test = pop';
    %  数据归一化
    p_test = mapminmax('apply', P_test, ps_input);
    
    t_sim = sim(net, p_test);
    T_sim = mapminmax('reverse', t_sim, ps_output);
end