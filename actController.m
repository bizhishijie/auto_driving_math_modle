%% 初始终态位置
% 四维，后两个维度是角度，分别用ab表示，需要满足模为1
l=0.1;% 小车的长度
Rmin=0.15;% 最小转弯半径
quiet=1;
x=repmat(0.2:0.32:1.8,1,6);
y=repmat(0.2:0.32:1.8,6,1);y=y(:)';
a=zeros(1,length(x));
b=ones(1,length(x));

pos_0=[x;y;a;b];
%clf;
hold on ;axis equal
plot(pos_0(1,:),pos_0(2,:),'o')
warning('off')
for ii=1:length(pos_0)
    arrow([pos_0(1,ii) pos_0(2,ii)] ,[pos_0(1,ii)+l*pos_0(3,ii) pos_0(2,ii)+l*pos_0(4,ii)],'Length',8)
end
warning('on')

load('pos_1_S.mat')
load('pos_1_H.mat')

pos_1=[pos_1_S pos_1_H];
plot(pos_1(1,:),pos_1(2,:),'o')
warning('off')
for ii=1:length(pos_1)
    arrow([pos_1(1,ii) pos_1(2,ii)] ,[pos_1(1,ii)+l*pos_1(3,ii) pos_1(2,ii)+l*pos_1(4,ii)],'Length',8)
end
warning('on')

%% 配对♥
dis=zeros(length(pos_0),length(pos_1));
path_cell=cell(length(pos_0),length(pos_1));
pos_0_th=[pos_0(1,:);pos_0(2,:);atan(pos_0(4,:)./pos_0(3,:))];
pos_1_th=[pos_1(1,:);pos_1(2,:);atan(pos_1(4,:)./pos_1(3,:))-(pos_1(3,:)<0)*pi];
Thre1=0.5;

reedsConnObj = reedsSheppConnection('MinTurningRadius',0.15);
for ii=1:length(pos_0)
    for jj=1:length(pos_1)

        startPose = pos_0_th(:,ii)';
        goalPose =  pos_1_th(:,jj)';
        [pathSegObj,pathCosts] = connect(reedsConnObj,startPose,goalPose);
        poses = interpolate(pathSegObj{1},0:0.001:pathCosts);
        %show(pathSegObj{1})
        %path=dubins_curve(pos_0_th(:,ii),pos_1_th(:,jj),Rmin,quiet);
        dis(ii,jj)=pathCosts;
        path_cell{ii,jj}=poses;
    end
end

var1=(dis-mean(mean(dis))).^2;
alpha=0.5;

warning off
% [i, j] = linear_sum_assignment((1-alpha)*dis+alpha*var1);
[i, j] = linear_sum_assignment(dis);
warning on

Car_for_Goal = i;
Goal_of_Car = j;
for ii = 1:36
    Goal_of_Car(ii) = find(Car_for_Goal==ii);
end

dis_0 = max(diag(dis(1:36,Goal_of_Car(1:36))));
%% 优化
% 每个路径规划方案是一个State，存着配对表
initState.Car_for_Goal = Car_for_Goal;
initState.Goal_of_Car = Goal_of_Car;

% 广度优先搜索，寻找最长路径最短的State
serchList = {};
serchList{1} = initState;
lengTarget = 0.5919;
idx = 1;

while ~isempty(serchList)
    disp(idx);
    currentState = serchList{1};
    serchList(1) = [];
    nextSerchList = nextState(currentState);
    
    len = stateMaxLength(currentState);
    if len < lengTarget
        lengTarget = len;
        bestState = currentState;
    end
    
    for ii = 1:length(nextSerchList)
        serchList{end+1} = nextSerchList{ii};
    end
    idx = idx+1;
end

Car_for_Goal = bestState.Car_for_Goal;
Goal_of_Car = bestState.Goal_of_Car;

Goal_of_Car([6,12,30,36])=[30,31,34,35];
Car_for_Goal([30,31,34,35])=[6,12,30,36];


%%
figure(1)
clf;
hold on;axis equal
plot(pos_0(1,:),pos_0(2,:),'o');
plot(pos_1(1,:),pos_1(2,:),'o');
warning('off')
for ii=1:length(pos_1)
    x1=pos_1(1,ii)-l*pos_1(3,ii)/2;x2=pos_1(1,ii)+l*pos_1(3,ii)/2;
    y1=pos_1(2,ii)-l*pos_1(4,ii)/2;y2=pos_1(2,ii)+l*pos_1(4,ii)/2;
    arrow([x1 y1] ,[x2 y2],'Length',8)
end
warning('on')
for ii=1:length(pos_1)
    path=path_cell{Car_for_Goal(ii),ii};
    plot(path(:,1),path(:,2),'color',[0.3,0.3,0.3])
    text(pos_0(1,ii),pos_0(2,ii),sprintf('%d',ii),'color',[0.5,0.5,0.5])
    text(pos_1(1,ii),pos_1(2,ii),sprintf('%d',ii),'color',[0.1,0.1,0.1])
end
axis([0 2 0 2]);grid on
%%
% pos_0(3:4,:)=pos_0(3:4,:)/10;
% pos_1(3:4,:)=pos_1(3:4,:)/10;

dt=0.1;vmax=0.1;    %折合0.1m/s
a=0.1;              % 控制小车行驶在线上
err=0.05;           %具有5%的误差
act=cell(1,3);      %储存命令
path_ds = 0.001;    %路径数组的间隔长度0.001m
%Th1=0.03;Th2=0.01;
t=0;
%realpos_cell=cell(1,length(pos_0));
offset = zeros(36,2);
target_pois = cell(1,4);
act_count = 1;
while 1
    % get cars' states and deviation from path 
    if t == 0
        pos_realtime = pos_0(1:2,:)';
    else
        p.Marker = '.';
        p.Color = [0.5 0.8 0.5];
        %pos_realtime = realpos_cell{end};
    end

    idealLength = vmax*t;
    for ii=1:length(pos_0)
        for jj = 1:4  %存一下小车们理论上到未来四个dt的点
            idx = round( min(idealLength+(jj-1)*dt*vmax,dis(ii,Goal_of_Car(ii))) /path_ds)+1;
            path_poi=path_cell{ii,Goal_of_Car(ii)}(idx,1:2);% 最优路径上的点
            target_pois{jj}(ii,:) = path_poi;
        end
        offset(ii,1:2)=target_pois{1}(ii,:)-pos_realtime(ii,:);% 当前偏移路线的程度       
    end
    
    % set and send act command from controller   
    if mod(round(t/dt),3)==0
        for ii = 1:3
            act{ii}=target_pois{ii+1}-target_pois{ii}+0.3*offset;
        end
        act_count = 1;
    end
    
    % update cars' states
    r = sqrt(rand(36,1));
    theta = rand(36,1)*2*pi;
    errVec = [r.*cos(theta),r.*sin(theta)];
    errVec = err*vecnorm(act{act_count},2,2).*errVec;
    %plot(errVec(:,1),errVec(:,2),'o');
    pos_realtime = pos_realtime+act{act_count}+errVec;
    act_count = act_count+1;
    
    % update figure
    axis equal;axis([0 2 0 2]);
    p = plot(pos_realtime(:,1),pos_realtime(:,2),'o','color','k');
    drawnow
    pause(dt)
    disp(t)
    t=t+dt;
    
    %break
    if abs(vecnorm(sum(pos_realtime-pos_1(1:2,:)'),2,2))<0.01
        disp(t)
        break
    end
end

%% func
function stateList = nextState(state)
Goal_of_Car = state.Goal_of_Car;
Car_for_Goal = state.Car_for_Goal;
dis = evalin('base','dis');

%找到最长路径 ij 
dis_max = max(diag(dis(1:36,Goal_of_Car(1:36))));
[iii,jjj] = find(dis==dis_max);

stateList = {};

for ii = 1:length(iii)
    kkk1 = find(dis(:,jjj(ii))<dis_max);
    kkk2 = Car_for_Goal(dis(iii(ii),:)<dis_max);
    kkk = intersect(kkk1,kkk2); % 允许经交换变得更短的路径 k
    lll = Goal_of_Car(kkk);     % 允许经交换变得更短的路径 l
    if ~isempty(kkk)
        stateListTmp = cell(1,length(kkk));
        for jj = 1:length(kkk)
            tmpState = state;
            tmpState.Goal_of_Car(iii(ii)) = lll(jj);
            tmpState.Goal_of_Car(kkk(jj)) = jjj(ii);
            tmpState.Car_for_Goal(jjj(ii)) = kkk(jj);
            tmpState.Car_for_Goal(lll(jj)) = iii(ii);
            stateListTmp{jj} = tmpState;
        end
        stateList{end+1} = stateListTmp{:};
    else
        stateList={};
    end
end
end

function dis_max = stateMaxLength(state)
Goal_of_Car = state.Goal_of_Car;
dis = evalin('base','dis');
dis_max = max(diag(dis(1:36,Goal_of_Car(1:36))));
end
