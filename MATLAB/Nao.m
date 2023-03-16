classdef Nao
    properties
        team
        inipose
        pose
        radius

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

        %% Define as differential drive for now
        % Define Vehicle
        R                 % Wheel radius [m]
        L                 % Wheelbase [m]
        dd

        colour

        waypoint

        controller
        r
        is_repeated
        arrived
        %Shotting and passing
        Fmax
        F


    end
    methods
        function obj = Nao(env,num,dt,totaltime,team,position,is_repeated,roboRadius,range)
            obj.is_repeated = is_repeated;
            obj.arrived = false;
            obj.team = team;
            obj.position_class = obj.get_position_class(position);
            
            
            obj.radius = roboRadius;

            obj.inipose = obj.position_class.get_pose(team);
            obj.pose = obj.position_class.get_pose(team);%[0;0;0];%[rand(1)*11,rand(1)*8,rand(1)]';
            obj.V = 0.1333;
            obj.w = 0;

            obj.vel = [obj.V*cos(obj.pose(3,1)); obj.V*sin(obj.pose(3,1))]';
            obj.acc = [0,0]';

            obj.fov = 0.873;
            obj.range = range;


            %% Timeseries data
            obj.poses = zeros(numel(0:dt:totaltime),2);
            obj.poses(1,:) = obj.inipose(1:2,:);

            obj.vels = zeros(numel(0:dt:totaltime),2);
            obj.vels(1,:) = obj.vel(1,:);

            obj.angles = zeros(numel(0:dt:totaltime),1);
            obj.angles(1,:) = obj.angles(1,:);

            obj.r = rateControl(1/dt);

            obj.dt = dt;



            obj.colour=obj.makecolour();

        end

        function obj = DroneMode(obj,idx,ballPose,ballorientation,ballV)
            obj.w = 0.7;
            obj.vel = [0;0];
        end

        %% Targeting algorithm
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
            x_m = x_m + obj.dt*x_mdot;

            %             disp(x_m)
            %             disp(-Rd)
            
            obj.vel = x_mdot;

            obj.w = phi_mdot;




            
            %searchBall(obj,trgt_pose)

            %             waitfor(obj.r);
        end

        function obj = update(obj,idx)
            obj.pose(3,1) = obj.pose(3,1) + obj.w;

            
            obj.pose(1,1) = obj.pose(1,1) + obj.vel(1,1)*obj.dt;
            obj.pose(2,1) = obj.pose(2,1) + obj.vel(2,1)*obj.dt;

            obj.poses(idx,1) = obj.pose(1);
            obj.poses(idx,2) = obj.pose(2);


            obj.vels(idx,1) = obj.vel(1,1);
            obj.vels(idx,2) = obj.vel(2,1);

        end
        
        %% Colour based on team
        function colour = makecolour(obj)

            if obj.team == 1
                colour = 'blue';
            else
                colour = 'red';
            end


        end

        %% shows the robot
        function show(obj,idx)
            
            plot(obj.poses(1:idx,1),obj.poses(1:idx,2),"Color",obj.colour); % draw trajectory
            
            
            obj.circle(obj.pose(1),obj.pose(2),obj.radius);%Draw robot


            
            % Direction
            x_mdot = [obj.V*cos(obj.pose(3)); obj.V*sin(obj.pose(3))];
            
            plot([obj.pose(1,1), obj.pose(1,1)+x_mdot(1)*5],[obj.pose(2,1), obj.pose(2,1)+x_mdot(2)*5],Color='r',LineWidth=1)

            %Draw sensors
            %Left
            left = [cos(obj.pose(3)+obj.fov/2); sin(obj.pose(3)+obj.fov/2)];
            plot([obj.pose(1,1), obj.pose(1,1)+left(1)*obj.range],[obj.pose(2,1), obj.pose(2,1)+left(2)*obj.range],Color='g',LineWidth=1,LineStyle='--')
            %Right
            right = [cos(obj.pose(3)-obj.fov/2); sin(obj.pose(3)-obj.fov/2)];
            plot([obj.pose(1,1), obj.pose(1,1)+right(1)*obj.range],[obj.pose(2,1), obj.pose(2,1)+right(2)*obj.range],Color='g',LineWidth=1,LineStyle='--')
            
            
        end
        %% set position class of each player
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
        % Input  {obj: nao object, ball_pose: [2x2 array]}
        %
        % Return {foundBall:bool % whether the ball is founded}      
        function foundBall = searchBall(obj, ball_pose)
            
            center_x = obj.pose(1);                                         % Get robot x position
            center_y = obj.pose(2);                                         % Get robot y position
            orientation = obj.pose(3);                                      % Get robot orientation
            ball_x = ball_pose(1,1);                                        % Get ball x position
            ball_y = ball_pose(2,1);                                        % Get ball y position
            theta = [obj.pose(3) - obj.fov/2; obj.pose(3) + obj.fov/2];     % Min and max angle of the sector
%             center = [center_x center_y]
%             ball = [ball_x ball_y]

            %polar_angle = atan2(ball_y,ball_x);
            distance = (center_x-ball_x)^2 + (center_y-ball_y)^2;           % Distance between the ball and the robot
            %angle = atan2(center_y-ball_y,center_x-ball_x)
            
            check_min_angle = (orientation >= theta(1));                    % Check if the orientation of the robot is larger than the min angle of sector
            check_max_angle = (orientation <= theta(2));                    % Check if the orientation of the robot is smaller than the max angle of sector
            check_radius = (distance <= obj.range^2);                       % Check if distance is smaller than the sensor radius
            foundBall = check_min_angle && check_max_angle && check_radius; % Reture true if all cases are true
            %foundBall = (polar_angle >= angle()            
            
        end
        function Svel=Shotting(goalpose,obj)
            d=goalpose-obj.pose;
            Svel=sqrt((2*obj.Fmax*d)/0.45);
        end
        function Pvel=Pass(Dpose,obj)
            d=Dpose-obj.pose;
            %F=d*0.0565; %for c=0.5
            %F=d*0.0365; %for c=0.4
            F=d*0.012; %for c=0.2
            %F=d*0.006; %for c=0.1
            Pvel=sqrt((2*obj.F.*d)/0.45);
        end


    end
end