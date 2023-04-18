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
Positions = {'Goalkeeper','Defender','Defender','Attacker'};

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

rightgoal = 0;
leftgoal = 0;


for i = 1:sim.numRobots
    sim.robots(i).goalPose = sim.robots(i).position_class.getGoalpose(sim.ball,sim.robots(i).team);
    sim.robots(i) = sim.robots(i).Make_controller(sim.robots,sim.ball);

end



for idx = 2:numel(tVec)
    % Update
    sim.ball = sim.ball.update_kick(idx,sim.ball.V,sim.ball.orientation);

    for i = 1:sim.numRobots
        %         if sim.robots(i).position_class.name == "Goalkeeper"
        %             sim.robots(i),sim.robots(i).checkBoundary()
        %         end
        %% robot state flow goes here
        [sim.robots(i),sim.ball] = sim.robots(i).checkColision(sim.robots,idx,sim.ball);
        %         sim.robots(i) = sim.robots(i).Make_controller(sim.robots,sim.ball);
        if sim.robots(i).position_class.name == "Attacker" && sim.robots(i).isFallen == false

            switch sim.robots(i).team % Checks team

                case 1 %Team Blue
                    switch  isempty(sim.ball.dribblingRobotID) %Checks to see if player has arrived at ball
                        case true
                            sim.robots(i).goalPose = sim.robots(i).position_class.getGoalpose(sim.ball,sim.robots(i).team);
                            if  mod(sim.robots(i).counter,10) == 0 ||mod(sim.robots(i).counter,20) == 20
                                sim.robots(i) = sim.robots(i).Make_controller(sim.robots,sim.ball);
                            end
                            sim.robots(i).counter = 1+sim.robots(i).counter;
                        case false
                            if i == sim.ball.dribblingRobotID
                                sim.robots(sim.ball.dribblingRobotID).goalPose = [9,5,0];

                                if  mod(sim.robots(i).counter,10) == 0 ||mod(sim.robots(i).counter,20) == 20
                                    sim.robots(i) = sim.robots(i).Make_controller(sim.robots,sim.ball);
                                else
                                end
                                euclidean_distance = sqrt((sim.robots(i).pose(1) - sim.robots(i).goalPose(1))^2 + (sim.robots(i).pose(2) - sim.robots(i).goalPose(2))^2);

                                % Check if the object is within 0.5 meters using an if statement
                                if euclidean_distance <= 1
                                    disp('The object is within 0.5 meters');
                                    sim.ball.V = 0;
                                    [sim.robots(i),Svel,sim.ball] = sim.robots(i).readytoshoot([10,5],sim.ball);
                                    sim.ball.V = Svel;


                                end

                                sim.robots(i).counter = 1+sim.robots(i).counter;
                            else
                                sim.robots(i).goalPose = sim.robots(i).position_class.getGoalpose(sim.ball,sim.robots(i).team);
                                if  mod(sim.robots(i).counter,10) == 0 ||mod(sim.robots(i).counter,20) == 20
                                    sim.robots(i) = sim.robots(i).Make_controller(sim.robots,sim.ball);
                                end

                                sim.robots(i).counter = 1+sim.robots(i).counter;
                            end
                            sim.robots(i) = sim.robots(i).RRT(idx);

                    end
                    sim.robots(i) = sim.robots(i).RRT(idx);
                    sim.ball = sim.ball.robotDribble(sim.robots(i).pose(3), sim.robots(i).pose(1:2),sim.robots(i).ID);

                case 0 %Team Red
                    switch  isempty(sim.ball.dribblingRobotID) %Checks to see if player has arrived at ball
                        case true
                            sim.robots(i).counter = 1+sim.robots(i).counter;
                            sim.robots(i).goalPose = sim.robots(i).position_class.getGoalpose(sim.ball,sim.robots(i).team);
                            if  mod(sim.robots(i).counter,10) == 0 ||mod(sim.robots(i).counter,20) == 20
                                sim.robots(i) = sim.robots(i).Make_controller(sim.robots,sim.ball);
                            end
                            
                        case false
                            if i == sim.ball.dribblingRobotID
                                
                                sim.robots(sim.ball.dribblingRobotID).goalPose = [2,4.5,pi];

                                if  mod(sim.robots(i).counter,20) == 0 ||mod(sim.robots(i).counter,20) == 20
                                    sim.robots(i) = sim.robots(i).Make_controller(sim.robots,sim.ball);
                                else
                                end

                                euclidean_distance = sqrt((sim.robots(i).pose(1) - sim.robots(i).goalPose(1))^2 + (sim.robots(i).pose(2) - sim.robots(i).goalPose(2))^2);

                                % Check if the object is within 0.5 meters using an if statement
                                if euclidean_distance <= 1
                                    [sim.robots(i),Svel,sim.ball] = sim.robots(i).readytoshoot([0,5],sim.ball);
                                    sim.ball.V = Svel;
                                end
                                sim.robots(i).counter = 1+sim.robots(i).counter;
                            end

                        sim.robots(i) = sim.robots(i).RRT(idx);

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
               sim.robots(i) = sim.robots(i).standstill();
            end
        else
            sim.robots(i) = sim.robots(i).getUp(idx);

        end
        sim.robots(i) = sim.robots(i).update(idx);
        sim.ball = sim.ball.robotDribble(sim.robots(i).pose(3), sim.robots(i).pose(1:2),sim.robots(i).ID);
        [sim.ball,tracker] = tracker.isPointInRectangle(sim.ball.Pose(1), sim.ball.Pose(2),sim.ball);
        [sim.ball,tracker] = tracker.isPointInrightRectangle(sim.ball.Pose(1), sim.ball.Pose(2),sim.ball);
        tracker.updateBallPos(sim.ball.Pose, scoreLeft, scoreRight,sim.ball);
        [tracker, sim.ball] = tracker.updateBallPos(sim.ball.Pose, scoreLeft, scoreRight,sim.ball);

    end
    if tracker.rightgoal == true
        rightgoal = rightgoal + 1;
        text(2, 7.5, 'Right goal!', 'Color', 'r', 'FontSize', 20)
        tracker.rightgoal = false;
    
    elseif tracker.leftgoal == true
        leftgoal = leftgoal+1;
        text(2, 7.5, 'Left goal!', 'Color', 'r', 'FontSize', 20);
        tracker.leftgoal = false;
    end


    % Figure

    figure(2); clf; hold on; grid off; axis([0 11,0 8]); %set(gca,'visible','off');
    hold on
    sim.ball.show();
    for i = 1:sim.numRobots
        sim.robots(i).show(idx);
        sim.robots(4).show(idx,true);
        sim.robots(8).show(idx,true);
        text(3, 0.5, num2str(rightgoal), 'FontSize', 24, 'HorizontalAlignment', 'right');
        text(8, 0.5, num2str(leftgoal), 'FontSize', 24, 'HorizontalAlignment', 'left');
        
    end
    sim.drawpitch();
    drawnow;
    hold off


end
