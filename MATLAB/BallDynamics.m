classdef BallDynamics
    properties
        Pose=[5.5;5];
        Velocity=[0.1;0.1];
        KVelocity=[];
        C

        orientation
        V

        r

        poses
        dt
        totaltime
        dribblingRobotID = []
%         robotSpeed
    end
    methods
        function obj=BallDynamics(pose,velocity,kvelocity,c,dt,totalTime)%,robotSpeed)
            if nargin==6
                obj.dt = dt;
                obj.totaltime = totalTime;
                obj.poses = zeros(numel(0:dt:totalTime),2);
                
                obj.r = rateControl(1/dt);

                obj.Velocity = [0;0];
                obj.KVelocity = kvelocity;
                obj.C = c;

                obj.orientation = (5/3)*pi - pi;
%                 obj.V = sqrt(velocity(1,1)^2 + velocity(1,1)^2);
                obj.V = 0.05;
%                 obj.robotSpeed = Nao.vel;

            end 
        end
        
%         function obj = robotDribble(obj, robotDirection, robotPose, robotID)
%             % Calculate robot velocity vector
%             robotVelocity = 0.1333 * [cos(robotDirection); sin(robotDirection)];
% 
%             % Calculate distance between ball and robot
%             distance = norm(obj.Pose - robotPose);
% 
%             desiredBallPose = robotPose + 0.5 * [cos(robotDirection); sin(robotDirection)];
% 
%             % Check if the ball is moving at a lower speed than the robot
%             % and if the distance is less than 0.6
%             % and check if no robot is dribbling the ball, or if the dribbling robot is the current robot
%             if norm(obj.Velocity) < norm(robotVelocity) && distance <= 0.6 && (isempty(obj.dribblingRobotID) || obj.dribblingRobotID == robotID)
%             % Set the ball's velocity equal to the robot's velocity
%                 obj.Velocity = robotVelocity;
%                 obj.Pose = obj.Pose + 0.7* (desiredBallPose - obj.Pose);
%             % Update dribblingRobotID to the current robot's ID
%               obj.dribblingRobotID = robotID;
% 
%             end
%             obj.Pose = obj.Pose + obj.Velocity * obj.dt;
%             obj.Velocity = obj.Velocity - (obj.C * obj.Velocity);
%             % Update ball orientation based on its velocity
%             if norm(obj.Velocity) <= 0
%                 obj.Velocity = [0; 0];
%                 obj.orientation = 0;
%             % Clear dribblingRobotID since no robot is dribbling the ball
%               obj.dribblingRobotID = [];
%             else
%             obj.orientation = asin(obj.Velocity(2) / norm(obj.Velocity));
%             end
%         end
function obj = robotDribble(obj, robotDirection, robotPose, robotID)
    % Calculate robot velocity vector
    robotVelocity = 0.1333 * [cos(robotDirection); sin(robotDirection)];

    % Calculate distance between ball and robot
    distance = norm(obj.Pose - robotPose);

    % Check if the distance is less than 1
    % and check if no robot is dribbling the ball, or if the dribbling robot is the current robot
    if distance <= 1 && (isempty(obj.dribblingRobotID) || obj.dribblingRobotID == robotID)
        
        obj.Velocity = robotVelocity;
        obj.Pose = robotPose + 0.3 * [cos(robotDirection); sin(robotDirection)];
        obj.dribblingRobotID = robotID;
    end

    % Update the ball's position and orientation based on the dribbling robot's position and direction
    if ~isempty(obj.dribblingRobotID) && obj.dribblingRobotID == robotID
        obj.Pose = robotPose + 0.5 * [cos(robotDirection); sin(robotDirection)];
        obj.orientation = robotDirection;
    else
        % If the ball is not being dribbled, update its position and orientation based on its velocity
        obj.Pose = obj.Pose + obj.Velocity * obj.dt;
        obj.Velocity = obj.Velocity - (obj.C * obj.Velocity);

        if norm(obj.Velocity) <= 0
            obj.Velocity = [0; 0];
            obj.orientation = 0;
            % Clear dribblingRobotID since no robot is dribbling the ball
            obj.dribblingRobotID = [];
        else
            obj.orientation = asin(obj.Velocity(2) / norm(obj.Velocity));
        end
    end
end


        function obj= update(obj,idx)

            obj.Pose=obj.Pose+obj.Velocity*obj.dt;

            obj.Velocity=obj.Velocity-(obj.C*obj.Velocity);

%             disp(obj.Pose)
%             disp(obj.Velocity)
                        
            
            
            if obj.Velocity<=0
                obj.Velocity=[0;0];
                obj.orientation = 0;
            else
                obj.orientation = asin(obj.Velocity(1,2)/sqrt(obj.Velocity(1,2)^2+obj.Velocity(1,2)^2));
            end
%             disp(obj.orientation);
%             disp(['vely = ',obj.Velocity])


%             waitfor(obj.r);

            
            


        end

        function obj= update_kick(obj,idx,V,phi)
%             disp(phi)
            x_tdot = [V*cos(phi); V*sin(phi)];
%             disp(x_tdot)
            obj.Pose=obj.Pose+x_tdot*obj.dt;
%             waitfor(obj.r);
        end

        function show(obj)
            plot(obj.Pose(1), obj.Pose(2),'o', 'Color', 'k', "MarkerFaceColor", 'k', 'MarkerSize', 10)

            x_tdot = [obj.V*cos(obj.orientation); obj.V*sin(obj.orientation)];
%             disp(x_tdot)
            plot([obj.Pose(1,1), obj.Pose(1,1)+x_tdot(1)*0.5],[obj.Pose(2,1), obj.Pose(2,1)+x_tdot(2)*0.5],Color='r',LineWidth=1)
        end

        function obj=Drible(obj)
            obj.Pose=obj.Pose+obj.Velocity;
            waitfor(obj.r);
        end
        
        function obj=Kick(obj)
            obj.Pose=obj.Pose+obj.KVelocity;
            obj.KVelocity=obj.KVelocity-(obj.C*obj.KVelocity);
            if obj.Velocity<=0
                obj.Velocity=0;
            end
            pause(0.15);
        end    
    end
end
