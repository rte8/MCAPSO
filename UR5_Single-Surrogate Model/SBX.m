function newpop = SBX(apop)
    %  对违反约束的个体进行模拟二进制交叉
    pop = apop(:,1:18);
    index = find(apop(:,end) == 2);
    if length(index) < 2
        index_bad = find(apop(:,end) == 1);
        random_num = randperm(numel(index_bad),2);
        index = [index;random_num'];
    end
    u1=zeros(1,18);
    gama=zeros(1,18);

    random_num = randperm(numel(index),2);
    parent_1 = pop(random_num(1),:);
    parent_2 = pop(random_num(2),:);
    yita1 = randi([5, 20]);
    %进行模拟二进制交叉
    for j = 1:18
       u1(j)=rand(1);
       if u1(j)<0.5
           gama(j)=(2*u1(j))^(1/(yita1+1));
       else
           gama(j)=(1/(2*(1-u1(j))))^(1/(yita1+1));
       end
       newpop(j)=0.5*((1+gama(j))*parent_1(j)+(1-gama(j))*parent_2(j)); 
    end        
end