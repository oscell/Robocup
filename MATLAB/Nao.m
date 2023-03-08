classdef Nao
    properties
        team
        inipose
        pose
        radius


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

            obj.vel = [obj.V*cos(obj.pose(3,1)); obj.V*sin(obj.pose(3,1))]';
            obj.acc = [0,0]';

            obj.fov = 0.873;
            obj.range = range;

            obj.R = 0.5;
            obj.L = 0.5;
            obj.dd = DifferentialDrive(obj.R,obj.L);

            obj.detector = obj.MakeDetector(env,num);

            obj.waypoint = obj.makeWaypoints();


            %             obj.waypoint = [0,0; 2,2; 4,2; 2,4; 0.5,3];%[rand(1)*11,rand(1)*8]';

            obj.controller = obj.MakeController();
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

        function detector = MakeDetector(obj,env,num)
            detector = RobotDetector(env);
            detector.maxDetections = num;
            detector.maxRange = 10;
            detector.fieldOfView = pi/2;
        end

        function controller = MakeController(obj)
            controller = controllerPurePursuit;
            controller.Waypoints = [obj.waypoint];
            controller.LookaheadDistance = 1;
            controller.DesiredLinearVelocity = 0.75;
            controller.MaxAngularVelocity = 3;
        end

        function obj = update_pursuit(obj,idx)

            [vRef,wRef] = obj.controller(obj.pose);
            [wL,wR] = inverseKinematics(obj.dd,vRef,wRef);

            [v,w] = forwardKinematics(obj.dd,wL,wR);

            velB = [v;0;w]; % Body velocities [vx;vy;w]
            velocity = bodyToWorld(velB,obj.pose);  % Convert from body to world

            obj.pose = obj.pose + velocity*obj.dt;

            obj.poses(idx,1) = obj.pose(1);
            obj.poses(idx,2) = obj.pose(2);

            %             waitfor(obj.r);
        end

        function obj = DroneMode(obj,idx,ballPose,ballorientation,ballV)
            phi_mdot = 0.2;
            phi_m = obj.pose(3,1);
            obj.pose(3,1) = phi_m + phi_mdot;

            obj.poses(idx,1) = obj.pose(1);
            obj.poses(idx,2) = obj.pose(2);
        end

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
            %             if OR
            x_mdot = [V_m*cos(phi_m); V_m*sin(phi_m)];
            x_tdot = [V_t*cos(phi_t); V_t*sin(phi_t)];


            %
            %             disp('Check')
            %             disp(x_mdot)
            %             disp(V_m)
            %             disp(phi_m)
            %             disp(V_t)
            %             disp(phi_t)
            %             disp(x_tdot)

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


            obj.pose(3,1) = phi_m + phi_mdot;


            obj.pose(1,1) = x_m(1,1);
            obj.pose(2,1) = x_m(2,1);

            obj.poses(idx,1) = obj.pose(1);
            obj.poses(idx,2) = obj.pose(2);


            obj.vels(idx,1) = x_m(1,1);
            obj.vels(idx,2) = x_m(1,1);


            %             waitfor(obj.r);
        end

        function obj = update(obj,idx)
            %                 velocity = obj.pursuit();
            %                 disp(velocity)
            %                 obj.pose = obj.pose + velocity*obj.dt;
            %                 Perform forward discrete integration step
            %

        end

        function colour = makecolour(obj)

            if obj.team == 1
                colour = 'blue';
            else
                colour = 'red';
            end


        end

        function waypoints = makeWaypoints(obj)
            if obj.team == 1
                waypoints = [0,0; 2,2; 4,2; 2,4; 0.5,3];
            else
                waypoints = [6,0; 8,2; 6,2; 7,4; 6,3];
            end
        end

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
    end

end