function [newpop,newfit] = ConstraintHandle(apop,fitness,goal_position,popmax,popmin,AP,sizepop)
    %  MCRT
    u = rand;
    newpop = apop(:,1:18);
    [net,ps_input,ps_output] = TrainRBF(AP,sizepop);
    for i = 1:size(apop,1)
        if apop(i,end) == 1
            if u <= 0.6
                %  使用混沌初始化
                for j = 1:18
                    u_0 = 2*rand() - 1;
                    v_0 = 2*rand() - 1;
                    u_1 = sin(2/u_0);
                    v_1 = sin(2/v_0);
                    newpop(i,:) = popmin(j) + ((u_1 + 1)*(popmax(j) - popmin(j))/2);
                    u_0 = u_1;
                    v_0 = v_1;
                end
                fitness(i) = fun(newpop(i,:),goal_position);
                
            elseif  u > 0.6 && u <= 0.8
                %  使用SRS修复策略
                newpop(i,:) = SRS(apop(i,:),net,ps_input,ps_output);
                %  越界处理
                for c = 1:18
                   u_0 = 2*rand() - 1;
                   u_1 = sin(2/u_0);
                   if newpop(i,c) > popmax(c) 
                       newpop(i,c) = popmin(c) + ((u_1 + 1)*(popmax(c) - popmin(c))/2);
                   elseif  newpop(i,c) < popmin(c)
                       newpop(i,c) = popmin(c) + ((u_1 + 1)*(popmax(c) - popmin(c))/2); 
                   end
                   u_0 = u_1;
                end
                fitness(i) = fun(newpop(i,:),goal_position); 
                
            else
                %  使用模拟二进制交叉
                newpop(i,:) = SBX(apop);
                %  越界处理
                for c = 1:18
                   u_0 = 2*rand() - 1;
                   u_1 = sin(2/u_0);
                   if newpop(i,c) > popmax(c) 
                       newpop(i,c) = popmin(c) + ((u_1 + 1)*(popmax(c) - popmin(c))/2);
                   elseif  newpop(i,c) < popmin(c)
                       newpop(i,c) = popmin(c) + ((u_1 + 1)*(popmax(c) - popmin(c))/2); 
                   end
                   u_0 = u_1;
                end
                fitness(i) = fun(newpop(i,:),goal_position); 
            end
        end
    end
    newfit = fitness;
end