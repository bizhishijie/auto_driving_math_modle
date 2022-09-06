
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

Goal_of_Car([6,12,30,36])=[30,31,34,35];
Car_for_Goal([30,31,34,35])=[6,12,30,36];

% 当前State经过一次交换操作可以变得更短的话，就返回那些更好的State列表
% 交换操作：路径 [ij kl]<-->路径 [il kj]
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

