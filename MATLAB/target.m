%Tutorial 1 Question 4 from Autonomous guided systems
clear
clc
% close all;
% Shape of agents
C = [0 1.5 3;0 3 0 ]*100;

%Orientation of target(rad)
phi_t = (5/3)*pi - pi;

%Gain
N = 0.1;

%position of target
x0_t = [9;0];
%Orientation of moving obj(rad)
phi_m = (1/3)*pi;

%position of moving obj
x0_m = [0;0];

%Initial conditions
V_m = 2;
V_t = 1;
x_m = x0_m;
x_t = x0_t;



phi_mdot = 0;

t = 0;
dt = 0.05;
totalTime = 5;

tVec = 0:dt:totalTime;
%Storing trajectories for later ploting 
x_mp = [x0_m];
x_tp = [x0_t];

phi_mp = [phi_m];


termination = false;
R = sqrt((x_t(1,1) - x_m(1,1))^2 + (x_t(2,1) - x_m(2,1))^2);
Rd = -1;

for i = 1:numel(tVec)
    t = t + dt;
    if R < 10
        if Rd > 0
            disp('Finished with a range of: ')
            disp(R)
            termination = true;
            break
        end
    elseif t > 3.0
        termination = true;
    end

    R = sqrt((x_t(1,1) - x_m(1,1))^2 + (x_t(2,1) - x_m(2,1))^2);
    
    x_mdot = [V_m*cos(phi_m); V_m*sin(phi_m)];
    x_tdot = [V_t*cos(phi_t); V_t*sin(phi_t)];
    
    
    
    x_dif = x_t - x_m;
    x_dotDif = x_tdot - x_mdot;
    
    [x_difRot , rot] = rotate(x_dif,phi_m);
    x_dotDifRot = rotate(x_dotDif,phi_m);
    
    Rd = ((x_dif(1,1))*(x_dotDif(1,1)) + (x_dif(2,1))*(x_dotDif(2,1)))/R;
    Vc = -Rd;
    
    y = atan((x_difRot(2,1))/(x_difRot(1,1)));
    
    %sightline rate
    y_dot = (x_dotDifRot(2,1)*x_difRot(1,1) - x_dotDifRot(1,1)*x_difRot(2,1))/(x_difRot(1,1)^2 * (1/cos(y))^2);
    
    x_m = x_m + dt*x_mdot;
    x_t = x_t + dt*x_tdot;
    
    n = N*y_dot*Vc;
    phi_mdot = n/V_m;
    
    phi_m = phi_m + phi_mdot;
    
    x_mp = [x_mp x_m];
    x_tp = [x_tp x_t];

    phi_mp = [phi_mp phi_m];
end



figure(1)
clf;
hold on
title('Stoped at time: '+string(t))
plot(x_mp(1,:),x_mp(2,:),'Displayname','Inceptor')
plot(x_tp(1,:),x_tp(2,:),'DisplayName','Target')
hold off
legend('Location','southeast')
xlim([0 11]);   % Without this, axis resizing can slow things down
ylim([0 9]);
saveas(figure(1),'fig.png')

disp(tVec(1,1:i))
disp(x_mp(1,:))


figure(2) 
clf;
hold on
title('Distance over time')
plot(tVec(1,1:i),x_mp(1,2:end),'Displayname','x-distance')
plot(tVec(1,1:i),x_mp(2,2:end),'Displayname','y-distance')
legend('Location','southeast')
hold off

figure(3)
clf;
hold on
title('Angle over time')
plot(tVec(1,1:i),phi_mp(1,2:end),'Displayname','Angle')
legend('Location','southeast')
hold off

function [c ,rotMat] = rotate(C,a)
    rotMat = [cos(a) sin(a);-sin(a) cos(a)];
    for i = 1:numel(C(1,:))
        c(:,i) = rotMat*C(:,i);
    end
end

