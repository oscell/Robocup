classdef Nao
    properties
        team
        inipose
        pose
        radius

        ID

        w
        vel
        acc
        V

        position_class

        fov % field of view
        range %Sensor range


        detector

        %% timeseries Data
        dt
        tVec
        poses
        vels
        angles



        colour

        waypoint



        r

        is_repeated
    
        % Boundary for defender not to play too aggressive
        boundary % integer, x-coordinate


        %% states
        arrived
        isFallen


        % Start and goalpose for robot(testing)
        startPose        % Start pose [x y theta]
        goalPose = [1 1 -pi/2];       % Goal pose [x y theta]


        waypoints
        controller
        vehicle 
        map
        solInfo
        ss
        plannedPath
        
        posess %% same as trajectory but is the transfore(this is purely for testing)


        % for state flow
        counter = 0 

    end
    methods
        function obj = Nao(env,num,dt,totaltime,team,position,is_repeated,roboRadius,range,ID)
            obj.is_repeated = is_repeated;
            obj.arrived = false;
            obj.team = team;
            obj.position_class = obj.get_position_class(position);
            obj.ID = ID;

            obj.tVec = 0:dt:totaltime;
            
            obj.radius = roboRadius;

            obj.inipose = obj.position_class.get_pose(team);
            obj.pose = obj.position_class.get_pose(team);%[0;0;0];%[rand(1)*11,rand(1)*8,rand(1)]';
            obj.V = 2;%0.1333;
            obj.w = 0;

            obj.vel = [obj.V*cos(obj.pose(3,1)); obj.V*sin(obj.pose(3,1))]';
            obj.acc = [0,0]';

            obj.fov = 0.873;
            obj.range = range;
            obj.boundary = obj.position_class.get_boundary(team);


            %% Timeseries data
            obj.poses = zeros(numel(0:dt:totaltime),3);
            obj.poses(1,:) = obj.inipose;


            obj.vels = zeros(numel(0:dt:totaltime),2);
            obj.vels(1,:) = obj.vel(1,:);

            obj.angles = zeros(numel(0:dt:totaltime),1);
            obj.angles(1,:) = obj.angles(1,:);

            obj.r = rateControl(1/dt);

            obj.dt = dt;

            obj.isFallen = false;


            %% Pathplanning
            obj.startPose = transpose(obj.inipose);


            obj.colour=obj.makecolour();

            obj.vehicle = obj.Make_vehicle();


        end

        function vehicle = Make_vehicle(obj)
            %% Define Vehicle
            wheelRadius = 0.05;     % Wheel radius [m]
            frontLen = 0.25;        % Distance from CG to front wheels [m]
            rearLen = 0.25;         % Distance from CG to rear wheels [m]
            vehicle = FourWheelSteering(wheelRadius,[frontLen rearLen]);
        end
        
        %% Makes a new occupancy map with all other robots
        function obj = make_map(obj,robots)
            
            map = binaryOccupancyMap(11,9,100);
            for robot = robots
                if robot.ID ~= obj.ID
                    map.setOccupancy([robot.pose(1) robot.pose(2)], 1);
                end
            end
            map.inflate(0.25);
            obj.map = map;
        end

        function obj = Make_controller(obj,robots)
            
            obj = obj.make_map(robots);
%             map = binaryOccupancyMap(11,9,100);
%             inflate(map,0.25); % Inflate the map for planning

            
            % State space
            ss = stateSpaceDubins;
            ss.MinTurningRadius = 0.75;
            ss.StateBounds = [obj.map.XWorldLimits; obj.map.YWorldLimits; [-pi pi]];
            
            % State validator
            sv = validatorOccupancyMap(ss);
            sv.Map = obj.map;
            sv.ValidationDistance = 0.01;
            
            % Path planner
            planner = plannerRRT(ss,sv);
            planner.MaxConnectionDistance = 2.5;

            [plannedPath,solInfo] = plan(planner,transpose(obj.pose),obj.goalPose);
            if plannedPath.NumStates < 1
                disp('No path found. Please rerun the example');
            end
            interpolate(plannedPath,round(2*plannedPath.pathLength)); % Interpolate to approx. 2 waypoints per meter
            obj.waypoints = plannedPath.States(:,1:2);

            controller = controllerPurePursuit;
            controller.Waypoints = obj.waypoints;
            controller.LookaheadDistance = 0.25;
            controller.DesiredLinearVelocity = 1;
            controller.MaxAngularVelocity = 3;

            obj.controller = controller;
            obj.solInfo = solInfo;
            obj.ss = ss;
            obj.plannedPath = plannedPath;
        end

        %% Base Behavioural algorithms
        % Turns in a circle
        function obj = DroneMode(obj,idx,ballPose,ballorientation,ballV)
            obj.w = 0.7;
            obj.vel = [0;0];
        end

        % Standstill
        function obj = standstill(obj)
            obj.pose = obj.pose;
        end

        % Targeting algorithm to ball
        function obj = ToPoint(obj,idx,trgt_pose,orientation,V)

            N = 0.2; %Gain
            x_t = trgt_pose; % Position of target
            %             disp(x_t)
            x_m = obj.pose(1:2,1); % Position of target
            phi_t = orientation;
            phi_m = obj.pose(3,1);
            V_m = obj.V;
            V_t = V;


            OR = sqrt((x_t(1,1) - x_m(1,1))^2 + (x_t(2,1) - x_m(2,1))^2);
            
            if OR < 0.5
                obj.arrived = true;

                obj.pose(1,1) = x_m(1,1);
                obj.pose(2,1) = x_m(2,1);

                obj.poses(idx,1) = obj.pose(1);
                obj.poses(idx,2) = obj.pose(2);


                obj.vels(idx,1) = x_m(1,1);
                obj.vels(idx,2) = x_m(1,1);
                return
            end

            x_mdot = [V_m*cos(phi_m); V_m*sin(phi_m)];
            x_tdot = [V_t*cos(phi_t); V_t*sin(phi_t)];


            x_dif = x_t - x_m;

            x_dotDif = x_tdot - x_mdot;
            %             disp(x_dif)
            %             disp(x_dotDif)

            [x_difRot , rot] = obj.rotate(x_dif,phi_m);
            x_dotDifRot = obj.rotate(x_dotDif,phi_m);


            Rd = ((x_dif(1,1))*(x_dotDif(1,1)) + (x_dif(2,1))*(x_dotDif(2,1)))/OR;
            Vc = -Rd;

            y = atan((x_difRot(2,1))/(x_difRot(1,1)));
            %sightline rate
            y_dot = (x_dotDifRot(2,1)*x_difRot(1,1) - x_dotDifRot(1,1)*x_difRot(2,1))/(x_difRot(1,1)^2 * (1/cos(y))^2);

            n = N*y_dot*Vc;
            %             disp(n)
            phi_mdot = n/V_m;


            %             disp(x_m)
            %             disp(-Rd)
            
            obj.vel = x_mdot;

            obj.w = phi_mdot/obj.dt;




         

            %             waitfor(obj.r);
        end

        % Rapid random tree to goal pose
        % Goes to point asigned in Goal pose
        function obj = RRT(obj,idx)
            

            poses = transpose(obj.poses);

            [vRef,wRef] = obj.controller(poses(:,idx-1));
            [wheelSpeeds,steerAngles] = inverseKinematicsFrontSteer(obj.vehicle,vRef,wRef);
            wheelSpeeds = wheelSpeeds([1 1]); % Use front wheel speed for both
            
            % Compute the velocities
            velB = forwardKinematics(obj.vehicle,wheelSpeeds,steerAngles);
            vel = bodyToWorld(velB,poses(:,idx-1));  % Convert from body to world

            obj.vel = vel(1:2);
            obj.w = vel(3);
        end
        %% Update function
        function obj = update(obj,idx)


            obj.pose(3,1) = obj.pose(3,1) + obj.w*obj.dt;



            if obj.isFallen == true
                obj.pose = obj.pose;
            else            
                obj.pose(1,1) = obj.pose(1,1) + obj.vel(1)*obj.dt;
                obj.pose(2,1) = obj.pose(2,1) + obj.vel(2)*obj.dt;
            end


            %% Append trajectory
            obj.poses(idx,1) = obj.pose(1);
            obj.poses(idx,2) = obj.pose(2);
            obj.poses(idx,3) = obj.pose(3,1);

            obj.vels(idx,1) = obj.vel(1);
            obj.vels(idx,2) = obj.vel(2);

        end
        
        %% Checks for colisions, if true robot is fallen
        function obj = checkColision(obj,robots)
            counter = 1;
            for robot = robots
                

                if counter ~= obj.ID
                    distance = sqrt((robot.pose(1,1)-obj.pose(1,1))^2 +(robot.pose(2,1)-obj.pose(2,1))^2);
                    
                    if distance < obj.radius*2

%                         disp("for robot "+ i+ " range is: "+distance+"from robot: "+counter)
                        obj.isFallen = true;

                    end
                end
                counter = counter + 1;
            end 
        end
        
        %% Colour based on team
        function colour = makecolour(obj)

            if obj.team == 1
                colour = 'blue';
            else
                colour = 'red';
            end


        end

        %% Visualisations
        % Shows the robot, waypoints trajectory
        function show(obj,idx,show_waypoints)

             if ~exist('show_waypoints','var')
                 % third parameter does not exist, so default it to something
                  show_waypoints = false;
             end
            % Show Id
            text(obj.pose(1,1)+obj.radius,obj.pose(2,1)+obj.radius,string(obj.ID))

            % draw trajectory
            plot(obj.poses(1:idx,1),obj.poses(1:idx,2),"Color",obj.colour); 
            
            %Draw robot
            obj.circle(obj.pose(1),obj.pose(2),obj.radius);

            % Wayoints
            if show_waypoints
                plot(obj.waypoints(:,1),obj.waypoints(:,2),'Marker','x')
            end

            % Goal pose
            plot(obj.goalPose(1,1),obj.goalPose(1,2),'Marker','x','Color',obj.colour)
            
            % Direction
            x_mdot = [obj.V*cos(obj.pose(3)); obj.V*sin(obj.pose(3))];
            
%             plot([obj.pose(1,1), obj.pose(1,1)+x_mdot(1)*5],[obj.pose(2,1), obj.pose(2,1)+x_mdot(2)*5],Color='r',LineWidth=1)

            %Draw sensors
            %Left
            left = [cos(obj.pose(3)+obj.fov/2); sin(obj.pose(3)+obj.fov/2)];
            plot([obj.pose(1,1), obj.pose(1,1)+left(1)*obj.range],[obj.pose(2,1), obj.pose(2,1)+left(2)*obj.range],Color='g',LineWidth=1,LineStyle='--')
            %Right
            right = [cos(obj.pose(3)-obj.fov/2); sin(obj.pose(3)-obj.fov/2)];
            plot([obj.pose(1,1), obj.pose(1,1)+right(1)*obj.range],[obj.pose(2,1), obj.pose(2,1)+right(2)*obj.range],Color='g',LineWidth=1,LineStyle='--')
            
            
        end



        function visualizeRRTPath(obj)
            % Plot the path from start to goal
            plot(obj.plannedPath.States(:,1),obj.plannedPath.States(:,2),'r--','LineWidth',1.5);
            % Interpolate each path segment to be smoother and plot it
            tData = obj.solInfo.TreeData;
            print('hey')
            for idx = 3:3:size(tData,1)-2
                p = navPath(obj.ss,tData(idx:idx+1,:));
                interpolate(p,10);
                plot(p.States(:,1),p.States(:,2),':','Color',[0 0.6 0.9]);
            end
        end

        function show_occupancy(obj)
            show(obj.map)
            hold on
            obj.show(0,true)
            obj.visualizeRRTPath()
        end

        %% set position class of each player - Goalkeeper, Attacker of Defender
        function position_class = get_position_class(obj,position)
            if strcmp(position,'Defender')
                position_class = Defender(obj.is_repeated);
            elseif strcmp(position,'Goalkeeper')
                position_class = Goalkeeper(obj.is_repeated);
            else
                position_class = Attacker(obj.is_repeated);
            end
        end

        function h = circle(obj,x,y,r)
            th = 0:pi/50:2*pi;
            xunit = r * cos(th) + x;
            yunit = r * sin(th) + y;
            h = plot(xunit, yunit,'Color',obj.colour,'LineWidth',1);
        end

        function [c ,rotMat] = rotate(obj,C,a)
            rotMat = [cos(a) sin(a);-sin(a) cos(a)];
            for i = 1:numel(C(1,:))
                c(:,i) = rotMat*C(:,i);
            end
        end

        %% Function for searching the ball in front of the nao robot
        %
        % Input  {obj: self, ball_pose: [2x2 array]}
        %
        % Return {foundBall:bool % whether the ball is founded}      
        function foundBall = searchBall(obj, ball_pose)
            
            center_x = obj.pose(1);                                         % Get robot x position
            center_y = obj.pose(2);                                         % Get robot y position
            orientation = obj.pose(3);                                      % Get robot orientation
            ball_x = ball_pose(1,1);                                        % Get ball x position
            ball_y = ball_pose(2,1);                                        % Get ball y position
            theta = [obj.pose(3) - obj.fov/2; obj.pose(3) + obj.fov/2];     % Min and max angle of the sector
            
            distance = (center_x-ball_x)^2 + (center_y-ball_y)^2;           % Distance between the ball and the robot
            
            check_min_angle = (orientation >= theta(1));                    % Check if the orientation of the robot is larger than the min angle of sector
            check_max_angle = (orientation <= theta(2));                    % Check if the orientation of the robot is smaller than the max angle of sector
            check_radius = (distance <= obj.range^2);                       % Check if distance is smaller than the sensor radius
            foundBall = check_min_angle && check_max_angle && check_radius; % Reture true if all cases are true                
        end

        % Function for searching any robot in front of the nao robot
        %
        % Input  {obj: self, robot_idx: Index of searching robot, totalNumRobot: Total number of robots, robots: array of nao robot object}
        %
        % Return {foundRobot:bool % whether any robot is founded}         
        function foundRobot = searchRobot(obj,robot_idx,totalNumRobot,robots)
            
            foundRobot = false;
            center_x = obj.pose(1);                                         % Get robot x position
            center_y = obj.pose(2);                                         % Get robot y position
            orientation = obj.pose(3);                                      % Get robot orientation
            theta = [obj.pose(3) - obj.fov/2; obj.pose(3) + obj.fov/2];     % Min and max angle of the sector
            for i = 1:totalNumRobot
                if i == robot_idx                                           % Skip robot itself
                    continue
                end
                target_x = robots(i).pose(1);
                target_y = robots(i).pose(2);
                distance = (center_x-target_x)^2 + (center_y-target_y)^2;
                check_min_angle = (orientation >= theta(1));                % Check if the orientation of the robot is larger than the min angle of sector
                check_max_angle = (orientation <= theta(2));                % Check if the orientation of the robot is smaller than the max angle of sector
                check_radius = (distance <= obj.range^2);                   % Check if distance is smaller than the sensor radius
                if check_min_angle && check_max_angle && check_radius           
                    foundRobot = true;                                      % Set foundRobot to true if robot is found
                    break;
                end
            end   
            
        end

        %% Function for checking the boundary of the role of robots
        %
        % Input  {obj: self}
        %
        % Return {isWithinBoundary:bool % whether the robot current position is within the boundary}   
        function isWithinBoundary = checkBoundary(obj)
            check_x =  (obj.boundary(1,1) <= obj.pose(1,1)) && (obj.pose(1,1) <= obj.boundary (2,1));
            check_y =  (obj.boundary(1,2) >= obj.pose(2,1)) && (obj.pose(2,1) >= obj.boundary (2,2));
            isWithinBoundary =  (check_x && check_y);            
        end


    end
end