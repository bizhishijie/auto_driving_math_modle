x=[];
y=[];
d0=0.2;% 车之间的间距
for ii=1:8
    x=[x zeros(1,8+1-ii)+d0*ii];
    y=[y d0:d0:(8+1-ii)*d0];
end
%  plot(pos_0(1,:),pos_0(2,:),'o');
% hold on
% 第一列共计8辆车,间距0.2m
pos_0=[x;y];% 生成完毕小车
load("pos_1_heart.mat")
%  plot(pos_1(1,:),pos_1(2,:),'o');

dis=zeros(length(pos_0),length(pos_1));
for ii=1:length(pos_0)
    for jj=1:length(pos_1)
        dis(ii,jj)=sqrt(sum((pos_0(:,ii)-pos_1(:,jj)).^2));
    end
end
var1=(dis-mean(mean(dis))).^2;
alpha=0.5;
[i, j] = linear_sum_assignment((1-alpha)*dis+alpha*var1);
% for ii=1:36
%     line([pos_0(1,i(ii)) pos_1(1,j(ii))],[pos_0(2,i(ii)) pos_1(2,j(ii))])
% end
%%
clf;hold on
plot(pos_0(1,:),pos_0(2,:),'o');
plot(pos_1(1,:),pos_1(2,:),'o');

for ii=1:length(pos_1)
    line([pos_0(1,i(ii)) pos_1(1,j(ii))],[pos_0(2,i(ii)) pos_1(2,j(ii))])
    text(pos_0(1,ii),pos_0(2,ii),sprintf('%d',ii))
    text(pos_1(1,ii),pos_1(2,ii),sprintf('%d',ii))
end

clearvars -except pos_0 pos_1 i j x y time
pos=pos_0;
a1=3;% 受力系数，即最大受力情况
a2=5;
Th1=0.03;Th2=0.01;
t=0;cnt=0;
dt=0.01;
vmax=0.1;
while 1
    % 小车间作用力
    f1=zeros(size(pos));
    x=pos(1,:);y=pos(2,:);
    for ii=1:length(x)
        x1=x(ii);%第一个点的坐标
        y1=y(ii);
        for jj=ii+1:length(x)%第二个点在第一个点之后
            x2=x(jj);
            y2=y(jj);
            Dist=sqrt((x2-x1)^2+(y2-y1)^2);%勾股定理
            if Dist<Th1
                f1(:,ii)=-(Th1-Dist)*a1/(Th1-Th2)*normalize([x2-x1,y2-y1],'norm');
            end
        end
    end
    % 小车向终点的作用力
    f2=zeros(size(pos));
    % 小车的正前方就有另外的小车，迫使小车减速的力
    f3=zeros(size(pos));
    for ii =1:length(pos_0)
        vc=pos_1(:,j(ii))-pos(:,i(ii));
        f2(:,i(ii))=normalize(vc,'norm');
        if sum(f2(:,i(ii)).*f1(:,ii))<-0.9
            f3(:,ii)=-f1(:,ii)-f2(:,i(ii));
            %             disp(f3(:,ii))
        end
    end
    f=normalize(f1+f2+f3,'norm');
    pos=pos+f*dt*vmax;
    figure(1)
    plot(pos(1,:),pos(2,:),'o');
    drawnow
    pause(dt)
    disp(t)
    if mod(cnt,100)==0
        figure(2);clf;
        hold on
        plot(pos_0(1,:),pos_0(2,:),'o');
        plot(pos_1(1,:),pos_1(2,:),'o');

        for ii=1:length(pos_1)
            line([pos_0(1,i(ii)) pos_1(1,j(ii))],[pos_0(2,i(ii)) pos_1(2,j(ii))])
%             text(pos_0(1,ii),pos_0(2,ii),sprintf('%d',ii))
%             text(pos_1(1,ii),pos_1(2,ii),sprintf('%d',ii))
        end
        plot(pos(1,:),pos(2,:),'o');axis([0 2 0 2])
        drawnow
        saveas(gcf,['./graph/' num2str(cnt) '.jpg']); 
    end
    cnt=cnt+1;
    t=t+0.01;
%     disp(t)
    if abs(sum(sum(pos-pos_1)))<0.001%检测是否到达终点
        disp(t)
        break
    end
end
