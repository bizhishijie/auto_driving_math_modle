y1=linspace(0.25,1.75,8);
x1=zeros(1,8)+1.2;
a1=zeros(1,8);
b1=ones(1,8);


x2=1.26:0.24:1.74;
y2=ones(1,3);
a2=ones(1,3);
b2=zeros(1,3);

y3=linspace(0.25,1.75,8);
x3=zeros(1,8)+1.8;
a3=zeros(1,8);
b3=ones(1,8);

pos_1_H=[[x1 x2 x3];[y1 y2 y3];[a1 a2 a3];[b1 b2 b3]];
save('pos_1_H.mat','pos_1_H')