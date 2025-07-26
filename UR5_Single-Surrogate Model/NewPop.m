function [pop,V,fitness] = NewPop(pop,sizepop,popmax,popmin,Vmax,Vmin,goal_position)
    %  重新生成所有个体进行重新搜索
    rng(3);
    fitness = zeros(sizepop,1);  
    
    for i = 1:sizepop
        u_0 = 2*rand() - 1; 
        v_0 = 2*rand() - 1;
        for j = 1:18 
            % 混沌初始化
            u_1 = sin(2/u_0);
            v_1 = sin(2/v_0);
            pop(i,j) = popmin(j) + ((u_1 + 1)*(popmax(j) - popmin(j))/2);
            V(i,j) = Vmin + ((v_1 + 1)*(Vmax - Vmin)/2); 
            u_0 = u_1; 
            v_0 = v_1;
        end
        % 计算适应度值
        fitness(i,:) = fun(pop(i,:),goal_position);
    end
    
end