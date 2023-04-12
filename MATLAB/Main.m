clear
clc
% close all

rng(10)

%% Simulation time

dt = 0.2;
totalTime = 40;

tVec = 0:dt:totalTime;


%% Teams
num_teams = 2;
robot_radius = 0.15;
sensorRange = 2;
showEnv = false;
Positions = {'Goalkeeper','Defender','Defender','Attacker','Attacker'};

fieldPos = [1, 1, 10, 7];
goalLeft = [0.4 2.7 1 5.3];
goalRight = [10 2.7 10.6 5.3];
scoreLeft = 0;
scoreRight = 0;

sim = simulation(dt,totalTime,num_teams,robot_radius,showEnv,Positions,sensorRange);
sim.ball.dt = dt;
sim.ball.orientation = pi;
sim.ball.V = 0.01;

ballPos = sim.ball.Pose;
tracker = BallTracker(fieldPos, ballPos, goalLeft, goalRight, scoreLeft, scoreRight);

for i = 1:sim.numRobots
        sim.robots(i).goalPose = sim.robots(i).position_class.getGoalpose(sim.ball);
        sim.robots(i) = sim.robots(i).Make_controller(sim.robots,sim.ball);
        
end

% Show the occupancy map and planned path
% figure(1)
% sim.robots(1).show_occupancy()
% hold off



for idx = 2:numel(tVec)
    % Update
    sim.ball = sim.ball.update_kick(idx,sim.ball.V,sim.ball.orientation);

    for i = 1:sim.numRobots
        sim.robots(i).counter = sim.robots(i).counter+1;
%         if sim.robots(i).position_class.name == "Goalkeeper"
%             sim.robots(i),sim.robots(i).checkBoundary()
%         end
        %% robot state flow goes here
%         sim.robots(i) = sim.robots(i).checkColision(sim.robots,idx);
        sim.robots(i).goalPose = sim.robots(i).position_class.getGoalpose(sim.ball);
        sim.robots(i) = sim.robots(i).Make_controller(sim.robots,sim.ball);
        if sim.robots(i).position_class.name == "Attacker" && sim.robots(i).isFallen == false
            
            switch sim.robots(i).team % Checks team

                case 1 %Team Blue
                            switch sim.robots(i).searchBall(sim.ball.Pose) %Looks for ball
                                case 1 %Ball has been found
                                    switch sim.robots(i).arrived %Checks to see if player has arrived at ball
                                        case false

%                                             sim.robots(i) = sim.robots(i).ToPoint(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
                                            sim.robots(i) = sim.robots(i).RRT(idx);
                                        case true

%                                             sim.robots(i) = sim.robots(i).ToPoint(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
                                            sim.robots(i) = sim.robots(i).RRT(idx);
                                            

                                            if sim.robots(i).counter == 0
%                                                 sim.robots(i) = sim.robots(i).Make_controller(sim.robots);
                                            elseif mod(sim.robots(i).counter,20) == 0
                                                sim.robots(i).ID
%                                                 sim.robots(i) = sim.robots(i).Make_controller(sim.robots);
                                            end
                                            sim.robots(i).counter = 1+sim.robots(i).counter;

%                                             sim.robots(i) = sim.robots(i).RRT(idx);


                                    end
                                   sim.ball = sim.ball.robotDribble(sim.robots(i).pose(3), sim.robots(i).pose(1:2),sim.robots(i).ID); 
                                case 0 %Ball not found
                                    sim.robots(i) = sim.robots(i).DroneMode();
                            end

                case 0 %Team Red
                            switch sim.robots(i).searchBall(sim.ball.Pose) %Looks for ball
                                case 1 %Ball has been found
                                         switch sim.robots(i).arrived %Checks to see if player has arrived at ball
            
                                             case false
                                                 sim.robots(i) = sim.robots(i).ToPoint(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
                                             case true
                                                
                                                 sim.robots(i) = sim.robots(i).ToPoint(idx,[4.5,9],0,4);
                                         end
                                         
                                case 0 %Ball not found
                                    sim.robots(i) = sim.robots(i).DroneMode();
                            end
                       
            end

        elseif sim.robots(i).position_class.name == "Defender" && sim.robots(i).isFallen == false
            if sim.robots(i).checkBallBoundary(sim.ball.Pose) == false
                sim.robots(i) = sim.robots(i).standstill();
            else
                sim.robots(i) = sim.robots(i).ToPoint(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
            end
        elseif sim.robots(i).position_class.name == "Goalkeeper" && sim.robots(i).isFallen == false
            if sim.robots(i).checkBallBoundary(sim.ball.Pose) == false

                sim.robots(i) = sim.robots(i).standstill();
            else
                sim.robots(i) = sim.robots(i).ToPoint(idx,sim.ball.Pose,sim.ball.orientation,sim.ball.V);
            end
        else
            sim.robots(i) = sim.robots(i).getUp(idx);

        end
        sim.robots(i) = sim.robots(i).update(idx);
        sim.ball = sim.ball.robotDribble(sim.robots(i).pose(3), sim.robots(i).pose(1:2),sim.robots(i).ID);
        tracker.updateBallPos(sim.ball.Pose, scoreLeft, scoreRight);

    end



    % Figure
    
    figure(2); clf; hold on; grid off; axis([0 11,0 8]); %set(gca,'visible','off');
    hold on
    sim.ball.show();
    for i = 1:sim.numRobots
        sim.robots(i).show(idx);
        sim.robots(4).show(idx,true);
        tracker.showScores();
    end
    sim.drawpitch();
    drawnow;
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
