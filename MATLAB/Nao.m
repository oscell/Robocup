classdef Nao
    properties

      inipose
      pose


      vel
      acc
      class
      team
     
      detector
      
      dt
      tVec
      poses 

      %% Define as differential drive for now
      % Define Vehicle
      R                 % Wheel radius [m]
      L                 % Wheelbase [m]
      dd

      colour

      waypoint
      
      controller

      r 


      
    end
    methods
        function obj = Nao(env,num,dt,totaltime,team)
            obj.inipose = obj.getpose();
            obj.pose = obj.getpose();%[0;0;0];%[rand(1)*11,rand(1)*8,rand(1)]';
            obj.vel = [0,0]';
            obj.acc = [0,0]';
            
             


            obj.R = 0.5;
            obj.L = 0.5;
            obj.dd = DifferentialDrive(obj.R,obj.L);

            obj.detector = obj.MakeDetector(env,num);

            obj.waypoint = obj.makeWaypoints();


%             obj.waypoint = [0,0; 2,2; 4,2; 2,4; 0.5,3];%[rand(1)*11,rand(1)*8]';

            obj.controller = obj.MakeController();
            obj.poses = zeros(numel(0:dt:totaltime),2);
            obj.r = rateControl(1/dt);

            obj.dt = dt;

            
            

            
            obj.team = team;

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

        function obj = update(obj,idx)
                [vRef,wRef] = obj.controller(obj.pose);
                [wL,wR] = inverseKinematics(obj.dd,vRef,wRef);

                [v,w] = forwardKinematics(obj.dd,wL,wR);

                velB = [v;0;w]; % Body velocities [vx;vy;w]
                velocity = bodyToWorld(velB,obj.pose);  % Convert from body to world
                
                % Perform forward discrete integration step
                
                obj.pose = obj.pose + velocity*obj.dt;
                    
                obj.poses(idx,1) = obj.pose(1);
                obj.poses(idx,2) = obj.pose(2);
                
                
                waitfor(obj.r);
        end

        function colour = makecolour(obj)
            
            if obj.team == 0 
                colour = 'blue';
            else
                colour = 'red';
            end
            

        end
        function pose = getpose(obj)
            disp(obj.team)
            if obj.team == 1
                pose = [0;0;0];
            else
                pose = [10;0;0];
            end
        
        end

        function waypoints = makeWaypoints(obj)
            if obj.team == 0
                disp(obj.team)
                waypoints = [0,0; 2,2; 4,2; 2,4; 0.5,3];
            else
                waypoints = [11,0; 8,2; 6,2; 7,4; 6,3];
            end
        end

        function show(obj)
            plot(obj.poses(:,1),obj.poses(:,2),"Color",obj.colour) % draw trajectory
            
            %Draw waypoints
            waypoints = obj.waypoint;
            plot(waypoints(:,1),waypoints(:,2),'x','MarkerEdgeColor','red','MarkerSize',10,LineWidth=1);
            
            text(waypoints(1)+0.2,waypoints(2)+0.2,string(1));

            

            %Draw robot
            obj.circle(obj.pose(1),obj.pose(2),obj.R)
        end
        
        function h = circle(obj,x,y,r)
            
            th = 0:pi/50:2*pi;
            xunit = r * cos(th) + x;
            yunit = r * sin(th) + y;
            h = plot(xunit, yunit,'Color',obj.colour,'LineWidth',1);
            
        end



    end

end