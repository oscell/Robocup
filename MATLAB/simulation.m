classdef simulation
    properties
        numRobots    {mustBeNumeric}
        sampletime
        num_teams
        teams

        robotradius 

        env
        
        robots
        gamestate

        tVec % Time array

        totaltime

        ShowEnv
    end
    methods
        function obj = simulation(num_Robots,dt,totalTime,num_teams,robot_radius,show_env)
            obj.numRobots = num_Robots;
            obj.sampletime = dt;

            obj.num_teams = num_teams;
            obj.teams = obj.make_teams();
            obj.robotradius = 0.15;

            obj.env = obj.MakeEnv(0.15,true);

            obj.totaltime = totalTime;

            obj.tVec = 0:obj.sampletime:totalTime;
            obj.robots = obj.MakeRobots();
            obj.robotradius = robot_radius;
            obj.gamestate = gamestate();
            obj.ShowEnv = show_env;
        end

        function obj = run(obj)
            for idx = 2:numel(obj.tVec)

                % Update the environment
                obj = obj.update(idx);
                if obj.ShowEnv
                    obj.show();
                end
                
                xlim([0 11]);   % Without this, axis resizing can slow things down
                ylim([0 9]);


            end
        end

        function show(obj)
%             clf(fig)
%             cla reset;
             
            hold on
            

             obj.env(1:obj.numRobots,[obj.robots.pose]);

            
            
            hold on

            obj.plot_waypoints()
            obj.drawpitch()
           
            
            hold off
            set(gca,'visible','off')
        end

        function summary(obj)
            figure
            hold on
            obj.drawpitch()

            for i = 1:obj.numRobots
                obj.robots(i).show()
                
%                 obj.plot_waypoints();
                
            end
            hold off
            xlim([0 11]);   % Without this, axis resizing can slow things down
            ylim([0 9]);
            set(gca,'visible','off')
        end

        function plot_waypoints(obj)
            waypoints = [obj.robots.waypoint];
            plot(waypoints(:,1),waypoints(:,2),'x','MarkerEdgeColor','red','MarkerSize',10,LineWidth=1);
            for i = 1:obj.numRobots
                text(waypoints(1,i)+0.2,waypoints(2,i)+0.2,string(i));
            end
            
        
        end


        function obj = update(obj,idx)
            for i = 1:obj.numRobots
                obj.robots(i) = obj.robots(i).update(idx);
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
                robots = [robots,Nao(obj.env,obj.numRobots,obj.sampletime,obj.totaltime,obj.teams(i))];
            end
        end

        function teams = make_teams(obj)
            blue = ones(1);
            red = zeros(1);

            teams = [blue,red];
        
        end

        function h = circle(obj,x,y,r)
            
            th = 0:pi/50:2*pi;
            xunit = r * cos(th) + x;
            yunit = r * sin(th) + y;
            h = plot(xunit, yunit);
            
        end

    end
end