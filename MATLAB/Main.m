% clear
% clc
% % close all
% 
% rng(10)
% 
% %% Simulation time
% 
% 
% dt = 0.5;
% totalTime = 8;
% 
% tVec = 0:dt:totalTime;
% 
% 
% %% Teams
% num_teams = 2;
% robot_radius = 0.15;
% sensorRange = 2;
% showEnv = false;
% Positions = {'Goalkeeper','Defender','Defender'};
% 
% 
% sim = simulation(dt,totalTime,num_teams,robot_radius,showEnv,Positions,sensorRange);
% 
% for i = 1:sim.numRobots
%         sim.robots(i) = sim.robots(i).Make_controller(sim.robots);
% end
% 
% %% Show the occupancy map and planned path
% figure(1)
% sim.robots(1).show_occupancy()
% 
% 
% 
% for idx = 2:numel(tVec)
%     % Update
%     sim.ball = sim.ball.update_kick(idx,sim.ball.V,sim.ball.orientation);
%     for i = 1:sim.numRobots
% 
%         
%         %% robot state flow goes here
%         % If the robot hasnt arrived go to the ball else drone mode
% 
% 
% %         switch sim.robots(i).position_class.name %Checks player position 
% %             case "Attacker"
% %                 switch sim.robots(i).team
% % 
% %                     case 1 %Team Blue
% %                                 switch sim.robots(i).searchBall(sim.ball.Pose) %Looks for ball
% %                                     case 1 %Ball has been found
% %                                         switch sim.robots(i).arrived %Checks to see if player has arrived at ball
% %         
% %                                             case false
% %                                                 sim.robots(i) = sim.robots(i).ToPoint(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
% %                                                 sim.robots(i) = sim.robots(i).update(idx);
% %                                             case true
% %                 %                                 sim.robots(i) = sim.robots(i).ToPoint(idx,[4.5,9],0,4);
% %                                         end
% %                                     case 0 %Ball not found
% %                                 end
% % 
% %                     case 0 %Team Red
% %                                 switch sim.robots(i).searchBall(sim.ball.Pose) %Looks for ball
% %                                     case 1 %Ball has been found
% %                                              switch sim.robots(i).arrived %Checks to see if player has arrived at ball
% %                 
% %                                                 case false
% %                                                         sim.robots(i) = sim.robots(i).ToPoint(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
% %                                                         sim.robots(i) = sim.robots(i).update(idx);
% %                                                 case true
% %                         %                                 sim.robots(i) = sim.robots(i).ToPoint(idx,[4.5,9],0,4);
% %                                               end
% %                                     case 0 %Ball not found
% %                                 end
% %                            
% %                 end
% % 
% %             case "Defender"
% % 
% %             case "Goalkeeper"
% % 
% %         end
%     
%         % robot state flow goes here
% %         If the robot hasnt arrived go to the ball else drone mode
% 
%         if sim.robots(i).arrived == false
%             if sim.robots(i).searchBall(sim.ball.Pose)
% %                 disp("Robot "+i+" found the ball")
%             end
%             if sim.robots(i).searchRobot(i,sim.numRobots,sim.robots)
%                 disp("Robot "+i+" sees another robot")
%             end
%             sim.robots(i) = sim.robots(i).ToPoint(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
%         else
%             sim.robots(i) = sim.robots(i).DroneMode(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
%         end
%                     % colision check
%         sim.robots(i) = sim.robots(i).checkColision(i,sim.robots);
%         
% 
%         % RRT
%         % plan a new path every so often to update obstacles
%         if mod(idx,20) == 0
%             sim.robots(i) = sim.robots(i).Make_controller(sim.robots);
%         end
%         sim.robots(i) = sim.robots(i).RRT(idx);
%         % Update
%         sim.robots(i) = sim.robots(i).update(idx);
%         
%     end
% 
% 
% 
%     % Figure
%     
%     figure(2); clf; hold on; grid off; axis([0 11,0 8]); %set(gca,'visible','off');
%     hold on
%     sim.ball.show();
%     for i = 1:sim.numRobots
%         sim.robots(i).show(idx);
%     end
%     sim.drawpitch();    
%     hold off
% end
% 

%% PLOT
% figure(3); clf; hold on; grid on; axis([0 totalTime,-3 5]);
% % disp(tVec(1:idx))
% % disp(numel(sim.robots(i).poses(1:idx,1)))
% plot(tVec(1:idx),sim.robots(i).poses(1:idx,1))
% plot(tVec,sim.robots(i).poses(:,2))
% title('x and y movement vs Time')
% xlabel('Time')
% ylabel('Position (m)')
% legend('x-position','y-position')
% hold off
% 
% figure(4); clf; hold on; grid on; axis([0 totalTime,-1 2]);
% plot(tVec,sim.robots(i).vels(:,1))
% plot(tVec,sim.robots(i).vels(:,2))
% title('Velocity vs Time')
% xlabel('Time')
% ylabel('Velocity (m/s)')
% legend('x-velocity','y-velocity')
% hold off
% 
% figure(5); clf; hold on; grid on; axis equal;
% plot(tVec,sim.robots(i).angles)
% title('angle vs Time')
% xlabel('Time (s)')
% ylabel('Angle (Rad)')
% hold off

% figure(3); clf; hold on; grid on; axis([0 totalTime,-3 5]);
% % disp(tVec(1:idx))
% % disp(numel(sim.robots(i).poses(1:idx,1)))
% plot(tVec(1:idx),sim.robots(i).poses(1:idx,1))
% plot(tVec,sim.robots(i).poses(:,2))
% title('x and y movement vs Time')
% xlabel('Time')
% ylabel('Position (m)')
% legend('x-position','y-position')
% hold off
% 
% figure(4); clf; hold on; grid on; axis([0 totalTime,-1 2]);
% plot(tVec,sim.robots(i).vels(:,1))
% plot(tVec,sim.robots(i).vels(:,2))
% title('Velocity vs Time')
% xlabel('Time')
% ylabel('Velocity (m/s)')
% legend('x-velocity','y-velocity')
% hold off
% 
% figure(5); clf; hold on; grid on; axis equal;
% plot(tVec,sim.robots(i).angles)
% title('angle vs Time')
% xlabel('Time (s)')
% ylabel('Angle (Rad)')
% hold off

% figure(6); clf; hold on; grid off; axis([0 11,0 8]); %set(gca,'visible','off');
% hold on
% sim.ball.show();
% for i = 1:sim.numRobots
%     sim.robots(i).show(1);
% end
% sim.drawpitch();    
% hold off
% saveas(figure(6),'Images\Startstate.png')


% figure(7); clf; hold on; grid off; axis([0 11,0 8]); %set(gca,'visible','off');
% hold on
% sim.ball.show();
% for i = 1:sim.numRobots
%     sim.robots(i).show(idx);
% end
% sim.drawpitch();    
% hold off
% saveas(figure(7),'Images\Finalstate.png')

% % 创建BallDynamics对象和机器人的位置和半径
% % 创建BallDynamics对象和机器人的位置和半径
% ball = BallDynamics([0;0], [0.2;0.1], [0;0], 0.1, 0.01, 10);
% robot_pose = [0.5;0.5];
% robot_radius = 0.1;
% 
% % 设置MATLAB图形窗口为"hold on"
% clf;
% hold on;
% axis([-1 1 -1 1]);
% 
% % 绘制机器人
% rectangle('Position',[robot_pose(1)-robot_radius, robot_pose(2)-robot_radius, 2*robot_radius, 2*robot_radius], 'Curvature', [1,1], 'FaceColor', 'b');
% 
% % 模拟球的运动
% for i = 1:numel(0:ball.dt:ball.totaltime)
%     ball.show();
%     
%     % 判断机器人与球的位置关系，如果机器人在球身边，则输出一段文字
%     if norm(ball.Pose - robot_pose) < 0.5 + robot_radius
%         disp('Robot passed by the ball!');
%     end
%     
%     % 更新球的位置和速度
%     ball.update(i);
%     
%     drawnow;
% end
% 创建BallDynamics对象和机器人的位置和半径
% clear
% clc
% % close all
% 
% rng(10)
% 
% %% Simulation time
% % 创建BallDynamics对象和机器人的位置、速度和半径
% ball = BallDynamics([1; 1], [0.2; 0.1], [0; 0], 0.1, 0.5, 15);
% robot_pose = [0.5; 0.5];
% robot_velocity = [0.05; 0.05];
% robot_radius = 0.1;
% 
% % 创建一个新的图形窗口
% figure;
% axis([-2 2 -2 2]);
% hold on;
% 
% % 模拟球的运动和机器人的带球
% for i = 1:numel(0:ball.dt:ball.totaltime)
%     % 更新机器人的位置
%     robot_pose = robot_pose + robot_velocity * ball.dt;
%     
%     % 调用Carry方法来模拟机器人带球的情况
%     ball.Carry(robot_pose, robot_radius);
%     
%     % 更新球的位置和方向
%     ball.update(i);
%     
%     % 在图形窗口中显示球和机器人
%     plot(ball.Pose(1), ball.Pose(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
%     plot(robot_pose(1), robot_pose(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
%     
%     drawnow;
% end
% Test script for BallDynamics with robotDribble
% Test script for BallDynamics with robotDribble
% Test script for BallDynamics with robotDribble
% Test script for BallDynamics with robotDribble
clear;
clc;

% Test script for BallDynamics with robotDribble
clear;
clc;

% Set parameters
pose = [0; 0.3];
velocity = [1; 1];
kvelocity = [0; 0];
c = 0.1;
dt = 0.1;
totalTime = 10;
robotSpeed = 1;
Pose = [-1;-0.4];

% Create BallDynamics object
ball = BallDynamics(pose, velocity, kvelocity, c, dt, totalTime, robotSpeed);

% Set robot direction (in radians)
robotDirection = 0.5;

% Initialize robot pose
robotPose = Pose;

% Initialize figure
figure;
hold on;
xlim([-10, 10]);
ylim([-10, 10]);
xlabel('X');
ylabel('Y');
title('Robot Dribbling a Ball');
grid on;

% Loop through time steps
for t = 0 : dt : totalTime
    % Clear the current figure
    clf;
    hold on;
    xlim([-10, 10]);
    ylim([-10, 10]);
    xlabel('X');
    ylabel('Y');
    title('Robot Dribbling a Ball');
    grid on;

    % Update ball dynamics with robot dribble
    ball = ball.robotDribble(robotDirection, robotPose);

    % Update robot pose
    robotVelocity = ball.robotSpeed * [cos(robotDirection); sin(robotDirection)];
    robotPose = robotPose + robotVelocity * dt;

    % Visualize ball and robot positions
    ball.show();
    plot(robotPose(1), robotPose(2), 'o', 'Color', 'b', "MarkerFaceColor", 'b', 'MarkerSize', 8);

    % Pause for visualization
    pause(dt);
end

% Release figure
hold off;