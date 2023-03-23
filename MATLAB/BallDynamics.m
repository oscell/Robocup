classdef BallDynamics
    properties
        Pose=[];
        Velocity=[];
        KVelocity=[];
        C

        orientation
        V

        r

        poses
        dt
        totaltime
        robotSpeed
    end
    methods
        function obj=BallDynamics(pose,velocity,kvelocity,c,dt,totalTime,robotSpeed)
            if nargin==7
                obj.dt = dt;
                obj.totaltime = totalTime;
                obj.poses = zeros(numel(0:dt:totalTime),2);
                
                obj.r = rateControl(1/dt);

                obj.Pose = pose;
                obj.Velocity = velocity;
                obj.KVelocity = kvelocity;
                obj.C = c;

                obj.orientation = (5/3)*pi - pi;(5/3)*pi - pi;
%                 obj.V = sqrt(velocity(1,1)^2 + velocity(1,1)^2);
                obj.V = 0.05;
                obj.robotSpeed = robotSpeed;
            end 
        end
        
        function obj = robotDribble(obj, robotDirection, robotPose)
            % Calculate robot velocity vector
            robotVelocity = obj.robotSpeed * [cos(robotDirection); sin(robotDirection)];

            % Calculate distance between ball and robot
            distance = norm(obj.Pose - robotPose);

            desiredBallPose = robotPose + 0.6 * [cos(robotDirection); sin(robotDirection)];

            % Check if the ball is moving at a lower speed than the robot
            % and if the distance is less than 0.6
            if norm(obj.Velocity) < norm(robotVelocity) && distance <= 0.6
            % Set the ball's velocity equal to the robot's velocity
                obj.Velocity = robotVelocity;
                obj.Pose = obj.Pose + 0.3* (desiredBallPose - obj.Pose);
            end
            obj.Pose = obj.Pose + obj.Velocity * obj.dt;
            obj.Velocity = obj.Velocity - (obj.C * obj.Velocity);
            % Update ball orientation based on its velocity
            if norm(obj.Velocity) <= 0
                obj.Velocity = [0; 0];
                obj.orientation = 0;
            else
            obj.orientation = asin(obj.Velocity(2) / norm(obj.Velocity));
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
            x_tdot = [V*cos(phi); V*sin(phi)];
            obj.Pose=obj.Pose+x_tdot*obj.dt;
%             waitfor(obj.r);
        end

        function show(obj)
            plot(obj.Pose(1,1), obj.Pose(2,1),'o', 'Color', 'k', "MarkerFaceColor", 'k', 'MarkerSize', 10)

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
