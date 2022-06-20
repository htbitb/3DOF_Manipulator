function draw_01(the,ith)
the1 = the(1);
the2 = the(2);
the3 = the(3);
L1=1; L2=1; L3=1;

x = [    0, 0, L1*cos(the1), cos(the1)*(L1 + L2*cos(the2)), cos(the1)*(L1 + L3*cos(the2 + the3) + L2*cos(the2))];
y = [    0, 0, L1*sin(the1), sin(the1)*(L1 + L2*cos(the2)), sin(the1)*(L1 + L3*cos(the2 + the3) + L2*cos(the2))];
z = [ -0.5, 0,            0,                  L2*sin(the2),                  L3*sin(the2 + the3) + L2*sin(the2)];

figure(ith)
plot3(x(1:2),y(1:2),z(1:2),'-O',x(2:3),y(2:3),z(2:3),'-O',x(3:4),y(3:4),z(3:4),'-O',x(4:5),y(4:5),z(4:5),'-X','linewidth',3);
grid on;
xlim([-3 3]); ylim([-3 3]); zlim([-3 3]);
xlabel('x','fontsize',18);
ylabel('y','fontsize',18);
zlabel('z','fontsize',18);