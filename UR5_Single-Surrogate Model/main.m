clc;
clear all;
t1=clock;
sim=remApi('remoteApi'); % 加载库函数并构建对象
sim.simxFinish(-1); % 断开所有之前的连接
clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);  %启动链接，指向vrep中的scence,返回链接ID
% 19999是端口号，5000是通信速率，5是连接超时时间

% 参数初始化
c1 = 2;
c2 = 2;
traingen = 2;  % 收集训练数据的代数
dl_accuracy = 0.6;  % 代理模型重新训练的准确率阈值
ws = 0.9;   %线性递减惯性权重
we = 0.4;
ps = 0.9;
r6 = rand();
l = 5;
F = 0.5;
n = 0;   
i = 0;

maxgen = 400;   % 进化次数  
sizepop = 100;   %种群规模
maxk = 50;
Vmax = 2;
Vmin = -2;

% Q-learning
numBins = 4;             % 状态离散化等级
numStates = numBins^3;  % 总状态数：64
numActions = 4;          % 动作数量
actions = [50, 100, 120, 150];  % 可选动作

% Q-Learning 参数
alpha = 0.1;             % 学习率
gamma = 0.9;             % 折扣因子
epsilon = 1.0;           % 初始探索率
epsilonDecay = 0.8;     % 探索率衰减
minEpsilon = 0.1;        % 最小探索率

% 初始化 Q 表为全零
Q = zeros(numStates, numActions);
Div = [];


%关节角度限制
popmax = [2*pi,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi,2*pi];  
popmin = [-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi,-2*pi];

fitness = zeros(sizepop,1);   %  适应度函数值
AP = [] ;   %  储存真实评估后的个体
allaccuracy = [];
yy = zeros(1,maxgen);
pop = zeros(sizepop,18);
apop = zeros(sizepop,20);  

h = [0,0,0,0,0,0];
nn = [];

% 连接UR5
[~,h(1)] = sim.simxGetObjectHandle(clientID,'UR5_joint1',sim.simx_opmode_blocking);
[~,h(2)] = sim.simxGetObjectHandle(clientID,'UR5_joint2',sim.simx_opmode_blocking);
[~,h(3)] = sim.simxGetObjectHandle(clientID,'UR5_joint3',sim.simx_opmode_blocking);
[~,h(4)] = sim.simxGetObjectHandle(clientID,'UR5_joint4',sim.simx_opmode_blocking);
[~,h(5)] = sim.simxGetObjectHandle(clientID,'UR5_joint5',sim.simx_opmode_blocking);
[~,h(6)] = sim.simxGetObjectHandle(clientID,'UR5_joint6',sim.simx_opmode_blocking);

if (clientID>-1)
    disp('Connected to remote API server');
    % 产生预设轨迹的三个点坐标
    goal_position = [0.1368,-0.3458,0.9443,-0.3215,-0.2154,0.8642,-0.6021,0.3895,0.603];  % 单

    % 固定随机数种子以保证重复性
    rng(3); 

    % 产生初始粒子和速度
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

    % 初始化变量以存储上一次的 accuracy 和 cov
    prevCov = mean(fitness);
    acc = 0; 
    prevAcc = acc;   
    e = 1e-6;  % 避免除以零
    prevDiv = 1/(max(fitness) - min(fitness) + e);

    % 个体极值和群体极值
    [bestfitness,bestindex] = min(fitness);
    zbest = pop(bestindex,:);  
    gbest = pop;   
    fitnessgbest = fitness;  
    fitnesszbest = bestfitness;  

    i = 0;
    % 迭代寻优
    while i < maxgen
        i = i + 1;
        w = ws - (ws - we)*i/maxgen + (0.5 - rand)*(1-i/maxgen)^2;   % CIPSO权重变化
        flag = 0;  
        
        if i == traingen
            [net_cnn,ps_input] = TrainCnn(AP);  % 收集n代数据后训练代理模型
        end
        %  处理真实评价开启标志
        if i == n || i <= traingen 
           flag = 1; 
        end
        if flag == 1
            sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot);
        else
            sim.simxPauseSimulation(clientID,sim.simx_opmode_oneshot);
        end

        for j = 1:sizepop
            % 速度更新
            V(j,:) = w*V(j,:) + c1*rand*(gbest(j,:) - pop(j,:)) + c2*rand*(zbest - pop(j,:));

            %  粒子越界处理
            v_0 = 2*rand() - 1; 
            v_1 = sin(2/v_0);
            V(j,find(V(j,:)>Vmax)) = Vmin + ((v_1 + 1)*(Vmax - Vmin)/2);
            V(j,find(V(j,:)<Vmin)) = Vmin + ((v_1 + 1)*(Vmax - Vmin)/2);
            v_0 = v_1;
            % 种群更新
            pop(j,:) = pop(j,:) + V(j,:);
            for c = 1:18
               u_0 = 2*rand() - 1;
               u_1 = sin(2/u_0);
               if pop(j,c) > popmax(c) 
                   pop(j,c) = popmin(c) + ((u_1 + 1)*(popmax(c) - popmin(c))/2);
               elseif  pop(j,c) < popmin(c)
                   pop(j,c) = popmin(c) + ((u_1 + 1)*(popmax(c) - popmin(c))/2); 
               end
               u_0 = u_1;
            end
            
            %  真实评价+代理
            if flag == 1
                %  每条轨迹之前让机械臂各关节角为0
                sim.simxPauseCommunication(clientID, 1);
                for n = 1:6
                    sim.simxSetJointTargetPosition(clientID,h(n),0,sim.simx_opmode_streaming);
                end
                sim.simxPauseCommunication(clientID, 0);
                pause(2);
                new_goal  = reshape(pop(j,:),6,3)';
                [r1, ~, ~, ~, ~] = sim.simxCallScriptFunction(clientID, 'UR5', sim.sim_scripttype_childscript, 'ClearTable', [], [], [], '', sim.simx_opmode_blocking);
                if r1 == sim.simx_return_ok
                  disp(['第',num2str(i),'代','第',num2str(j),'轮','清除成功']);
                end
                for m = 1:3
                    sim.simxPauseCommunication(clientID, 1);
                    for n = 1:6
                        sim.simxSetJointTargetPosition(clientID,h(n),new_goal(m,n),sim.simx_opmode_streaming);
                    end
                    sim.simxPauseCommunication(clientID, 0);
                    pause(2);
                end
                [r2, ~, retFloats, ~, ~] = sim.simxCallScriptFunction(clientID, 'UR5', sim.sim_scripttype_childscript, 'sendTableToMatlab', [], [], [], '', sim.simx_opmode_blocking);
                if r2 == sim.simx_return_ok
                %  1代表碰到了，2代表没碰到
                    min_distance = min(retFloats);
                    if min_distance == 0
                        collision_flag = 1;
                    else
                        collision_flag = 2;
                    end
                end
                
                apop(j,:) = [pop(j,:),min_distance,collision_flag];
                AP = [AP;apop(j,:)];
            end

            % 适应度值更新
            fitness(j) = fun(pop(j,:),goal_position); 
        end

        % 真实+代理评价
        if flag == 1 && i >= traingen
            accuracy = TestCnn(net_cnn,apop,ps_input);  
            acc = accuracy;
            if accuracy < dl_accuracy   
                %  直接选择allpop全部个体进行训练
                disp(['第',num2str(i),'代代理模型精度较低，为',num2str(accuracy*100),'%，需重新训练']);
                [net_cnn,ps_input] = TrainCnn(AP);
            else
                disp(['第',num2str(i),'代代理模型精度为',num2str(accuracy*100),'%'])
            end 

            if accuracy < dl_accuracy
                if i > 110 && i < 300
                    [pop,V,fitness] = NewPop(pop,sizepop,popmax,popmin,Vmax,Vmin,goal_position);
                    [bestfitness,bestindex] = min(fitness);
                    zbest = pop(bestindex,:);  
                    gbest = pop;  
                    fitnessgbest = fitness; 
                    fitnesszbest = bestfitness;  
                end
            end
            allaccuracy = [allaccuracy,accuracy];
        end

        % 代理评价
        if flag == 0
            apop = TestCnn_1(net_cnn,pop,ps_input);
            [net1,ps1_input,ps1_output] = TrainRBF(AP,sizepop);
            for j = 1:sizepop
                apop(j,19) = TestRBF(apop(1:18),net1,ps1_input,ps1_output);
            end
        end

        %  MCRT
        [pop,fitness] = ConstraintHandle(apop,fitness,goal_position,popmax,popmin,AP,sizepop);
        for j = 1:sizepop 
            if apop(j,end) == 1
               fitness(j) = 10; 
            end
            % 个体最优更新
            if fitness(j) <= fitnessgbest(j)
                gbest(j,:) = pop(j,:);
                fitnessgbest(j) = fitness(j);
            end
            % 群体最优更新
            if fitness(j) <= fitnesszbest
                zbest = pop(j,:);
                fitnesszbest = fitness(j);
            end
        end  

        % 局部变尺度深度搜索
        for k = 1:maxk
            [worsefitness,worseindex] = max(fitness);
            u_0 = 2*rand() - 1;
            for j = 1:sizepop
                u_1 = sin(2/u_0);
                p = rand();
                if p < ps
                    pop(worseindex,:) = zbest * power(1 + u_1 * (1 - k/maxk),exp(l * r6));
                else
                    pop(worseindex,:) = ((u_1 + 1) * (popmax - popmin))/2 + popmin;
                    u_0 = u_1;
                end
            end
            fitness(worseindex) = fun(pop(worseindex,:),goal_position); 
            if fitness(worseindex) <= fitnessgbest(worseindex)
                gbest(worseindex,:) = pop(worseindex,:);
                fitnessgbest(worseindex) = fitness(worseindex);
            end
            if fitness(worseindex) <= fitnesszbest
                zbest = pop(worseindex,:);
                fitnesszbest = fitness(worseindex);
            end
        end
       
        
        if flag == 1 && i >= traingen
            if accuracy >= dl_accuracy   
                nextCov = mean(fitness);
                nextAcc = acc;
                nextDiv = 1/(max(fitness) - min(fitness) + e);
                % 当前状态索引
                currentStateIdx = getStateIndex(prevAcc, prevCov, prevDiv, numBins);

                % 选择动作（ε-贪婪策略）
                if rand() < epsilon
                    actionIdx = randi(numActions);
                else
                    [~, actionIdx] = max(Q(currentStateIdx, :));  
                end
                n = actions(actionIdx) + i;
                reward = (nextAcc - prevAcc) + (prevCov - nextCov) + (prevDiv - nextDiv);
                done = i >= maxgen;
                % Q-Learning 更新
                if ~done
                    nextStateIdx = getStateIndex(nextAcc, nextCov, nextDiv, numBins);
                    [~, bestNextActionIdx] = max(Q(nextStateIdx, :));
                    target = reward + gamma * Q(nextStateIdx, bestNextActionIdx);
                else
                    target = reward;
                end
                Q(currentStateIdx, actionIdx) = Q(currentStateIdx, actionIdx) + alpha * (target - Q(currentStateIdx, actionIdx));

                if epsilon > minEpsilon
                    epsilon = epsilon * epsilonDecay;
                end
                % 更新上一次的 acc 和 cov 为当前的 acc 和 cov
                prevAcc = nextAcc;
                prevCov = nextCov;
                prevDiv = nextDiv;
                Div = [Div,prevDiv];
            else
                n = i + 1;
            end
            disp(n);

        end
        
        yy(i) = fitnesszbest;  % 记录每代的最优适应度   
    end
    
else
    disp('Failed connecting to remote API server');
end

disp(fitnesszbest);    %最优适应度值
disp(zbest);           %最优个体

% 检测最优解是否符合约束
CheckCollision(clientID,sim,h,zbest);

sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot);  % 停止仿真
sim.delete(); % 断开连接

disp('Program ended');

t2=clock;
figure;
plot(1:size(yy,2),yy)
title('the fitness curve');

figure;
plot(1:size(allaccuracy,2),allaccuracy)
hold on
yline(dl_accuracy,'r','Minimum accuracy')
title('Surrogate Model Accuracy');

[y1,y2] = testfun(zbest,goal_position);
disp(['最优个体的轨迹距离为',num2str(y1)]);
disp(['最优个体的关节角变化量为',num2str(y2)]);



function stateIdx = getStateIndex(accuracy, cov, diversity, numBins)
    % 离散化 accuracy
    if accuracy < 0.7 && accuracy >= 0.6
        accBin = 1;
    elseif accuracy < 0.8
        accBin = 2;
    elseif accuracy < 0.9
        accBin = 3;
    else
        accBin = 4;
    end

    % 离散化 cov
    if cov < 0.4
        covBin = 1;
    elseif cov < 0.8
        covBin = 2;
    elseif cov < 1.2
        covBin = 3;
    else 
        covBin = 4;
    end

    % 离散化 diversity
    if diversity < 1 * 0.25
        divBin = 1;
    elseif diversity < 1 * 0.5
        divBin = 2;
    elseif diversity < 1 * 0.75
        divBin = 3;
    else
        divBin = 4;
    end

    stateIdx = (accBin - 1) * numBins^2 + (covBin - 1) * numBins + divBin;
end


function normalized_value = normalize(value, minVal, maxVal)
    normalized_value = (value - minVal) / (maxVal - minVal);
    normalized_value = max(min(normalized_value, 1), 0);
end

