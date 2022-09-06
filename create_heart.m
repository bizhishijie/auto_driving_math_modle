len=190;
t=linspace(0,2*pi*35/36,1000);
x=16*(sin(t)).^3;
y=13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t);
pos_1=[x;y];

pos=[0;5];
jj0=1;
for ii =1:(36-1)
    pos_i=pos(:,end);
    for jj=jj0:length(pos_1)
        v_i_j=pos_1(:,jj)-pos_i;
        if sum(v_i_j.^2>=len/35)
            pos=[pos [pos_1(:,jj)]];
            jj0=jj;
            break
        end
    end
end
pos(1,:)=(pos(1,:)-min(pos(1,:)))/(max(pos(1,:))-min(pos(1,:)))+0.5;
pos(2,:)=(pos(2,:)-min(pos(2,:)))/(max(pos(2,:))-min(pos(2,:)))+0.5;

x=pos(1,:);y=pos(2,:);
axis equal
plot(x,y,'o')
pos_1=pos;
save('pos_1_heart.mat','pos_1')