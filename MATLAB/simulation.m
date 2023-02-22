classdef simulation
    properties
        numRobots    {mustBeNumeric}
        sampletime

        env

        robots
        gamestate

        tVec % Time array
    end
    methods
        function obj = simulation(num_Robots,dt,totalTime)
            obj.numRobots = num_Robots;
            obj.sampletime = dt;

            obj.env = obj.MakeEnv(0.15,true);



            obj.tVec = 0:obj.sampletime:totalTime;
            obj.robots = obj.MakeRobots();
            obj.gamestate = gamestate();
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

            obj.drawpitch()
            
            
            hold off
            set(gca,'visible','off')
        end


        function obj = update(obj,idx)
            for i = 1:obj.numRobots
                obj.robots(i) = obj.robots(i).update();
            end
        end

        function drawpitch(obj)
            rectangle('Position',[0 0 11 8]); % Outer pitch
            rectangle('Position',[1 1 9 6]); % inner pitch
            rectangle('Position',[1 2.5 1 3]); % left goal area
            rectangle('Position',[9 2.5 1 3]); % right goal area
            
            rectangle('Position',[1 1.5 2 5]); % left goal area
            rectangle('Position',[8 1.5 2 5]); % right goal area
            
            rectangle('Position',[0.4 2.7 0.6 2.6]); % left goal
            rectangle('Position',[10 2.7 0.6 2.6]); % right goal

            plot(2.5,4,'x','MarkerEdgeColor','black') %Left penalty mark
            plot(8.5,4,'x','MarkerEdgeColor','black') %right penalty mark

            plot([5.5,5.5],[1,7],'-k'); % Center line
            
            
            rectangle('Position',[4.75 3.25 1.5 1.5],'Curvature',[1 1]) % center circle

        
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