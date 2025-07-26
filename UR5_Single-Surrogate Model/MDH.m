function position = MDH(a)
    %构建机械臂MDH参数
    
    L(1) = Link('d', 0.089159, 'a', 0, 'alpha', pi/2, 'offset',-pi/2);
    L(2) = Link('d', 0, 'a', -0.425, 'alpha', 0, 'offset',-pi/2);
    L(3) = Link('d', 0, 'a', -0.39225, 'alpha', 0);
    L(4) = Link('d', 0.10915, 'a', 0, 'alpha', pi/2, 'offset',-pi/2);
    L(5) = Link('d', 0.09465, 'a', 0, 'alpha', -pi/2);
    L(6) = Link('d', 0.0823, 'a', 0, 'alpha', 0, 'offset',-pi/2);

    %建立机械臂模型
    myrobot= SerialLink(L, 'name', 'sixlink');
    
    a1 = a(1:6);
    a2 = a(7:12);
    a3 = a(13:18);

    %正运动学求解
    T1 = myrobot.fkine(a1).T;   %将fkine的输出变为矩阵形式，便于提取坐标值
    T2 = myrobot.fkine(a2).T;
    T3 = myrobot.fkine(a3).T;
    position = [T1(1,4),T1(2,4),T1(3,4)-0.013,T2(1,4),T2(2,4),T2(3,4)-0.013,T3(1,4),T3(2,4),T3(3,4)-0.013];
end