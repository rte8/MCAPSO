function [net,ps_input] = TrainCnn(allpop)
    res = allpop;
    %  分析数据
    num_class = length(unique(res(:, end)));              % 类别数
    num_dim = 18;               % 特征维度
    num_res = size(res, 1);                   % 样本数
    res = res(randperm(num_res), :);          

    %  设置变量存储数据
    P_train = res(:,1:18);T_train = res(:,end);

    %  数据转置
    P_train = P_train'; 
    T_train = T_train'; 
    
    M = size(P_train, 2);

    %  数据归一化
    [P_train, ps_input] = mapminmax(P_train, 0, 1);
    t_train =  categorical(T_train)';
 

    %  数据平铺
    p_train =  double(reshape(P_train, num_dim, 1, 1, M));

    %  构造网络结构
    layers = [
     imageInputLayer([num_dim, 1, 1])                          

     convolution2dLayer([2, 1], 16, 'Padding', 'same')          
     batchNormalizationLayer                                   
     reluLayer                                                

     maxPooling2dLayer([2, 1], 'Stride', [2, 1])                

     convolution2dLayer([2, 1], 32, 'Padding', 'same')          
     batchNormalizationLayer                                   
     reluLayer                                               

     fullyConnectedLayer(num_class)                         
     softmaxLayer                                              
     classificationLayer];                                    

    %  参数设置
    options = trainingOptions('adam', ...     
        'MaxEpochs', 500, ...                
        'InitialLearnRate', 1e-3, ...        
        'L2Regularization', 1e-4, ...         
        'LearnRateSchedule', 'piecewise', ... 
        'LearnRateDropFactor', 0.1, ...       
        'LearnRateDropPeriod', 400, ...       
        'Shuffle', 'every-epoch', ...          
        'ValidationPatience', Inf, ...      
        'Verbose', false);

    %  训练模型
    net = trainNetwork(p_train, t_train, layers, options);
end