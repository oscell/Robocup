classdef simulation
    properties
        numRobots    {mustBeNumeric}
        sampletime

        env

        robots

        tVec % Time array
    end
    methods
        function obj = simulation(num_Robots,dt,totalTime)
            obj.numRobots = num_Robots;
            obj.sampletime = dt;

            obj.env = obj.MakeEnv(0.15,true);



            obj.tVec = 0:obj.sampletime:totalTime;
            obj.robots = obj.MakeRobots();
        end

        function run(obj)
            for idx = 2:numel(obj.tVec)

                % Update the environment
                obj = obj.update(idx);
                obj.show();
                xlim([0 11]);   % Without this, axis resizing can slow things down
                ylim([0 9]);


            end
        end

        function show(obj)
            obj.env(1:obj.numRobots,[obj.robots.pose]);

            hold on
            %% plot waypoints
            waypoints = [obj.robots.waypoint];
            plot(waypoints(1,:),waypoints(2,:),'x','MarkerEdgeColor','red','MarkerSize',10,LineWidth=1)
            for i = 1:obj.numRobots
                text(waypoints(1,i)+0.2,waypoints(2,i)+0.2,string(i))
            end
            
            
            hold off
        end


        function obj = update(obj,idx)
            for i = 1:obj.numRobots
                obj.robots(i) = obj.robots(i).update();
            end
        end



        function env = MakeEnv(obj,robotRadius,showTrajectory)
            env = MultiRobotEnv(obj.numRobots);
            env.robotRadius = robotRadius;
            env.showTrajectory = showTrajectory;

        end

        function robots = MakeRobots(obj)
            robots = [];

            for i = 1:obj.numRobots
                robots = [robots,Nao(obj.env,obj.numRobots,obj.sampletime)];
            end
        end

    end
end