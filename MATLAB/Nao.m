classdef Nao
    properties

      pose 
      vel
      acc
     
      detector
      
      dt
      tVec
      poses 

      %% Define as differential drive for now
      % Define Vehicle
      R                 % Wheel radius [m]
      L                 % Wheelbase [m]
      dd

      waypoint

      controller

      r 


      
    end
    methods
        function obj = Nao(env,num,dt)
            obj.pose = [rand(1)*11,rand(1)*8,rand(1)]';
            obj.vel = [0,0]';
            obj.acc = [0,0]';

            obj.R = 0.1;
            obj.L = 0.5;
            obj.dd = DifferentialDrive(obj.R,obj.L);

            obj.detector = obj.MakeDetector(env,num);


            obj.waypoint = [rand(1)*11,rand(1)*8]';

            obj.controller = obj.MakeController();


            obj.dt = dt;


            obj.r = rateControl(1/dt);

        end

        function detector = MakeDetector(obj,env,num)
            detector = RobotDetector(env);
            detector.maxDetections = num;
            detector.maxRange = 10;
            detector.fieldOfView = pi/2;
        end

        function controller = MakeController(obj)
            controller = controllerPurePursuit;
            controller.Waypoints = [obj.pose(1:2),obj.waypoint];
            controller.LookaheadDistance = 1;
            controller.DesiredLinearVelocity = 0.75;
            controller.MaxAngularVelocity = 1.5;
        end

        function obj = update(obj)
                [vRef,wRef] = obj.controller(obj.pose);
                [wL,wR] = inverseKinematics(obj.dd,vRef,wRef);

                [v,w] = forwardKinematics(obj.dd,wL,wR);

                velB = [v;0;w]; % Body velocities [vx;vy;w]
                velocity = bodyToWorld(velB,obj.pose);  % Convert from body to world
                
                % Perform forward discrete integration step
                
                obj.pose = obj.pose + velocity*obj.dt;
                waitfor(obj.r);
        end




    end

end