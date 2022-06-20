clc; clear all;close all;
L1=150;L2=100;L3=50;
%tao ma tran rong
empty=[];empty1=[];empty2=[];
% ve toan bo khong gian lam viec cua robot 
i=0;
for the1=-90:90
    for the2=-90:90
        for the3=-90:90
            i=i+1;
            x= L3*cosd(the1+the2+the3)+L2*cosd(the1+the2)+L1*cosd(the1);
            y= L3*sind(the1+the2+the3)+L2*sind(the1+the2)+L1*sind(the1);
            empty(:,i)=[x;y];
        end
    end
end

%ve khong gian lam viec cua bo nghiem thu nhat (theta2<0)
i=0;
for the1=-90:90
    for the2=-90:0
        for the3=-90:90
            i=i+1;
            x= L3*cosd(the1+the2+the3)+L2*cosd(the1+the2)+L1*cosd(the1);
            y= L3*sind(the1+the2+the3)+L2*sind(the1+the2)+L1*sind(the1);
            empty1(:,i)=[x;y];
        end
    end
end

%ve khong gian lam viec cua bo nghiem thu hai (theta2>0)
i=0;
for the1=-90:90
    for the2=0:90
        for the3=-90:90
            i=i+1;
            x= L3*cosd(the1+the2+the3)+L2*cosd(the1+the2)+L1*cosd(the1);
            y= L3*sind(the1+the2+the3)+L2*sind(the1+the2)+L1*sind(the1);
            empty2(:,i)=[x;y];
        end
    end
end

subplot(2,2,1)
plot(empty(1,:),empty(2,:),'m*','MarkerSize',0.1)
title('toan bo khong gian lam viec')
grid on
xlabel('x')
ylabel('y')
subplot(2,2,2)
plot(empty1(1,:),empty1(2,:),'b*','MarkerSize',0.1)
title('bo nghiem thu 1')
grid on
xlabel('x')
ylabel('y')
subplot(2,2,3)
plot(empty2(1,:),empty2(2,:),'g*','MarkerSize',0.1)
title('bo nghiem thu 2')
grid on
xlabel('x')
ylabel('y')
whitebg('black')