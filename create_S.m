t =[linspace(pi*0.46 ,pi,500) flip(linspace(pi*1.46 ,2*pi,500))];
v = 50;
r = 0.3;
rho = r*abs(sin(t)).^ v;
x = rho.*cos(t);
y = rho.*sin(t);

% len=sum(sqrt(diff(x).^2+diff(y).^2));

pos_1=[x;y];
pos_1 (1,:)=(pos_1 (1,:)-min(pos_1 (1,:)))/(max(pos_1 (1,:))-min(pos_1 (1,:)))*0.8+0.1;
pos_1 (2,:)=(pos_1 (2,:)-min(pos_1 (2,:)))/(max(pos_1 (2,:))-min(pos_1 (2,:)))*1.5+0.25;
x=pos_1 (1,:);y=pos_1 (2,:);
diff_pos=normalize([diff(x);diff(y)],'norm');% 按列
diff_pos=-[diff_pos diff_pos(:,end)];
pos_1=[pos_1;diff_pos];

len=1.1;
plot(pos_1 (1,:),pos_1 (2,:),'o')
axis equal
pos=pos_1(:,1);
jj0=1;
for ii =1:(17-1)
    pos_i=pos(1:2,end);
    for jj=jj0:length(pos_1)
        v_i_j=pos_1(1:2,jj)-pos_i;
        if sum(v_i_j.^2>=len/35)
            pos=[pos [pos_1(:,jj)] ];
            jj0=jj;
            break
        end
    end
end

x=pos(1,:);y=pos(2,:);
plot(x,y,'o')
axis equal
pos_1_S=pos;
save('pos_1_S.mat','pos_1_S')
disp(length(pos))