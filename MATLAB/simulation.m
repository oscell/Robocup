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
        ball
        ShowEnv

        positions
    end
    methods
        function obj = simulation(num_Robots,dt,totalTime,num_teams,robot_radius,show_env,Positions)
            obj.numRobots = num_Robots;
            obj.sampletime = dt;

            obj.num_teams = num_teams;
            obj.teams = obj.make_teams();
            obj.robotradius = 0.15;

            obj.env = obj.MakeEnv(0.15,true);
            obj.ball= obj.MakeBall();

            obj.totaltime = totalTime;
            obj.positions = Positions;


            obj.tVec = 0:obj.sampletime:totalTime;
            obj.robots = obj.MakeRobots();
            obj.robotradius = robot_radius;
            obj.gamestate = gamestate();
            obj.ShowEnv = show_env;


        end

        function obj = run(obj)
            figure(2)
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
            figure(1);

            hold on
            
%             obj.env(1:obj.numRobots,[obj.robots.pose]);
            
            hold off
            hold on
            
            obj.ball.show();
            for i = 1:obj.numRobots
                disp(i)
                obj.robots(i).show();

            end
            hold off
            hold on
            obj.plot_waypoints();
            obj.drawpitch();
            

            
%             set(gca,'visible','off')
            hold off
        end

        function summary(obj)
            figure(3)
            hold on
            obj.drawpitch();
%             obj.ball.show();

            for i = 1:obj.numRobots
                obj.robots(i).show();


            end
            xlim([0 11]);   % Without this, axis resizing can slow things down
            ylim([0 9]);
            set(gca,'visible','off')
            hold off
        end

        function plot_waypoints(obj)
            waypoints = [obj.robots.waypoint];
            
            for ii = 1:2:2*obj.numRobots
                plot(waypoints(:,ii),waypoints(:,ii+1),'x','MarkerEdgeColor','red','MarkerSize',10,LineWidth=1);
                text(waypoints(end,ii)+0.2,waypoints(end,ii+1)+0.2,string(round(ii/2)));
            end


        end


        function obj = update(obj,idx)
            obj.ball = obj.ball.update(idx);
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

            plot(2.5,4,'x','MarkerEdgeColor','black'); %Left penalty mark
            plot(8.5,4,'x','MarkerEdgeColor','black'); %right penalty mark

            plot([5.5,5.5],[1,7],'-k'); % Center line


            rectangle('Position',[4.75 3.25 1.5 1.5],'Curvature',[1 1]); % center circle


        end



        function env = MakeEnv(obj,robotRadius,showTrajectory)
            env = MultiRobotEnv(obj.numRobots);
            env.robotRadius = robotRadius;
            env.showTrajectory = showTrajectory;

        end

        function ball=MakeBall(obj)
            pose=[5.5,4];
            velocity=[0.5,0.5];
            kvelocity=[5,5];
            c=0.25;
            ball=BallDynamics(pose,velocity,kvelocity,c,obj.sampletime,obj.totaltime);
        end



        function robots = MakeRobots(obj)
            robots = [];
            if obj.num_teams == 2
                counter = 1;
                for i = 0.5:0.5:obj.numRobots/2
                    robots = [robots,Nao(obj.env,obj.numRobots,obj.sampletime,obj.totaltime,obj.teams(counter),obj.positions(round(i)))];
                    counter = counter+1;
                end
            else
                for i = 1:obj.numRobots
                    if i > numel(obj.positions)
                        i = numel(obj.positions);
                    end
                    robots = [robots,Nao(obj.env,obj.numRobots,obj.sampletime,obj.totaltime,obj.teams(i),obj.positions(i))];
                end
            end

        end

        function teams = make_teams(obj)
            num = obj.numRobots;

            if obj.num_teams == 2 && num ~=1
                if mod(num,2) == 1
                    blue = ones(1,num/2+0.5);
                    red = zeros(1,floor(num/2));
                else
                    blue = ones(1,num/2);
                    red = zeros(1,num/2);
                end
                teams = [blue,red];

            else
                blue = ones(1,num);
                teams = blue;
            end
            
        end

        function h = circle(obj,x,y,r)

            th = 0:pi/50:2*pi;
            xunit = r * cos(th) + x;
            yunit = r * sin(th) + y;
            h = plot(xunit, yunit);

        end

        function unique()
            strings = {'some' 'strings' 'with' 'with' 'duplicate' 'strings' 'strings'};
            [~, uniqueIdx] =unique(strings); % Find the indices of the unique strings
            duplicates = strings; % Copy the original into a duplicate array
            duplicates(uniqueIdx) = []; % remove the unique strings, anything left is a duplicate
            
            duplicates = unique(duplicates); % find the unique duplicates
        end

        function N = num_unique()
        % This returns the number of unique elements
            [UniqueC,~,k] = unique(Positions); 
            N = histc(k,1:numel(UniqueC));
        end

    end
end