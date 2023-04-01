clear
clc
% close all

rng(10)

%% Simulation time
dt = 0.01;
totalTime = 8;

tVec = 0:dt:totalTime;


%% Teams
num_teams = 2;
robot_radius = 0.15;
sensorRange = 2;
showEnv = false;
Positions = {'Goalkeeper','Defender','Defender'};


sim = simulation(dt,totalTime,num_teams,robot_radius,showEnv,Positions,sensorRange);

for i = 1:sim.numRobots
        sim.robots(i) = sim.robots(i).Make_controller(sim.robots);
end

%% Show the occupancy map and planned path
figure(1)
sim.robots(1).show_occupancy()



for idx = 2:numel(tVec)
    % Update
    sim.ball = sim.ball.update_kick(idx,sim.ball.V,sim.ball.orientation);
    for i = 1:sim.numRobots

        
        %% robot state flow goes here
        % If the robot hasnt arrived go to the ball else drone mode


%         switch sim.robots(i).position_class.name %Checks player position 
%             case "Attacker"
%                 switch sim.robots(i).team
% 
%                     case 1 %Team Blue
%                                 switch sim.robots(i).searchBall(sim.ball.Pose) %Looks for ball
%                                     case 1 %Ball has been found
%                                         switch sim.robots(i).arrived %Checks to see if player has arrived at ball
%         
%                                             case false
%                                                 sim.robots(i) = sim.robots(i).ToPoint(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
%                                                 sim.robots(i) = sim.robots(i).update(idx);
%                                             case true
%                 %                                 sim.robots(i) = sim.robots(i).ToPoint(idx,[4.5,9],0,4);
%                                         end
%                                     case 0 %Ball not found
%                                 end
% 
%                     case 0 %Team Red
%                                 switch sim.robots(i).searchBall(sim.ball.Pose) %Looks for ball
%                                     case 1 %Ball has been found
%                                              switch sim.robots(i).arrived %Checks to see if player has arrived at ball
%                 
%                                                 case false
%                                                         sim.robots(i) = sim.robots(i).ToPoint(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
%                                                         sim.robots(i) = sim.robots(i).update(idx);
%                                                 case true
%                         %                                 sim.robots(i) = sim.robots(i).ToPoint(idx,[4.5,9],0,4);
%                                               end
%                                     case 0 %Ball not found
%                                 end
%                            
%                 end
% 
%             case "Defender"
% 
%             case "Goalkeeper"
% 
%         end
    
        % robot state flow goes here
%         If the robot hasnt arrived go to the ball else drone mode

        if sim.robots(i).arrived == false
            if sim.robots(i).searchBall(sim.ball.Pose)
%                 disp("Robot "+i+" found the ball")
            end
            sim.robots(i) = sim.robots(i).ToPoint(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
        else
            sim.robots(i) = sim.robots(i).DroneMode(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
        end
                    % colision check
        sim.robots(i) = sim.robots(i).checkColision(i,sim.robots);
        

        % RRT
        % plan a new path every so often to update obstacles
        if mod(idx,20) == 0
            sim.robots(i) = sim.robots(i).Make_controller(sim.robots);
        end
        sim.robots(i) = sim.robots(i).RRT(idx);
        % Update
        sim.robots(i) = sim.robots(i).update(idx);
        
    end



    % Figure
    
    figure(2); clf; hold on; grid off; axis([0 11,0 8]); %set(gca,'visible','off');
    hold on
    sim.ball.show();
    for i = 1:sim.numRobots
        sim.robots(i).show(idx,true);
    end
    sim.drawpitch();    
    hold off
end


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
