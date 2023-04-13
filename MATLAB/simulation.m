classdef simulation
    properties
        numRobots    {mustBeNumeric}
        sampletime
        num_teams
        teams

        robotradius
        sensorRange

        env

        robots
        nrobothb
        gamestate

        tVec % Time array

        totaltime
        ball
        ShowEnv

        passrobot

        positions
    end
    methods
        function obj = simulation(dt,totalTime,num_teams,robot_radius,show_env,Positions,SensorRange)

            if num_teams > 2
                disp('Max number of teams is 2')
                obj.num_teams = 2;
            else
                obj.num_teams = num_teams;
            end

            obj.numRobots = numel(Positions)*obj.num_teams;
            obj.sampletime = dt;
            
            obj.teams = obj.make_teams();


            obj.env = obj.MakeEnv(0.15,true);
            obj.ball= obj.MakeBall();

            obj.totaltime = totalTime;
            obj.positions = Positions;

            obj.tVec = 0:obj.sampletime:totalTime;
            obj.robotradius = robot_radius;
            obj.sensorRange = SensorRange;
            obj.robots = obj.MakeRobots();
            
            obj.gamestate = gamestate();
            obj.ShowEnv = show_env;

            


        end

        function show(obj,idx)

            hold on
            if obj.ShowEnv
                
                obj.env(1:obj.numRobots,[obj.robots.pose]);
                
            end

            hold off
            hold on
            
            obj.ball.show();
            for i = 1:obj.numRobots
                obj.robots(i).show(idx);

            end
            hold off
            hold on
%             obj.plot_waypoints();
            obj.drawpitch();
            

            
            
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
            set(gca,'visible','off');
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
%             obj.ball = obj.ball.update(idx);
            obj.ball = obj.ball.update_kick(idx,obj.ball.V,obj.ball.orientation);

            for i = 1:obj.numRobots
                %controller targeting
%                 obj.robots(i) = obj.robots(i).update_pursuit(idx);

                %update positions
                obj.robots(i) = obj.robots(i).update_target(idx,obj.ball.Pose,obj.ball.orientation,obj.ball.V);
            end
        end
        function obj = robothold(obj)
            for i = 1:obj.numRobots
                if obj.robots(i).holdball == 1
                    obj.nrobothb=i;
                end
            end
        end
        function obj = robottopass(obj)
            for i = 1:obj.numRobots
                if obj.robots(i).team==1
                    dx(i)=[1]-obj.robots(i).pose(1);
                else
                   dx(i)=[10]-obj.robots(i).pose(1);
                end
            end
            for i = 1:obj.numRobots
                lowest_dx = min([dx(i)]);
            end
            for i = 1:obj.numRobots
                if lowest_dx==dx(i)
                    obj.passrobot=i;
                end
            end
        end
        function obj = passing(obj)
            for i = 1:obj.numRobots
                 obj.robots(i).needpass(obj.robots(i).searchRobot(i,obj.numRobots,obj.robots))
            end
            obj.robots(obj.passrobot).desiredpose()
            obj.robots(obj.passrobot).Pass()
        end
        function obj=holdingball(obj)
            for i = 1:obj.numRobots
                if obj.ball.dribblingRobotID==i
                    obj.robots(i).holdball==1;
                else
                   obj.robots(i).holdball==0;
                end
            end
        end
        function ball=MakeBall(obj)
            pose=[5.5,4];
%             pose=[9;0];

            velocity=[0,0];
            kvelocity=[5,5];
            
            ball=BallDynamics(pose,velocity,kvelocity,obj.sampletime,obj.totaltime);
        end

        function env = MakeEnv(obj,robotRadius,showTrajectory)
            env = MultiRobotEnv(obj.numRobots);
            env.robotRadius = robotRadius;
            env.showTrajectory = showTrajectory;

        end

        function robots = MakeRobots(obj)
            robots = [];
            prev_pos = '';
            if obj.num_teams == 2
                position = [obj.positions,obj.positions];

                for i = 1:obj.numRobots
                    pos = position(i);
    
                    is_repeated = false;
                    if strcmp(prev_pos,pos)
                        is_repeated = true;
                    end
                    prev_pos = position(i);
                    robots = [robots,Nao(obj.env,obj.numRobots,obj.sampletime,obj.totaltime,obj.teams(i),position(i),is_repeated,obj.robotradius,obj.sensorRange,i)];
                end
            
            else
                for i = 1:obj.numRobots
                    if i > numel(obj.positions)
                        i = numel(obj.positions);
                    end
                    pos = obj.positions(i);
                    is_repeated = false;
                    if strcmp(prev_pos,pos)
                        is_repeated = true;
                    end
                    prev_pos = obj.positions(i);
                    robots = [robots,Nao(obj.env,obj.numRobots,obj.sampletime,obj.totaltime,obj.teams(i),obj.positions(i),is_repeated,obj.robotradius,obj.sensorRange,i)];

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

        function h = circle(obj,x,y,r)

            th = 0:pi/50:2*pi;
            xunit = r * cos(th) + x;
            yunit = r * sin(th) + y;
            h = plot(xunit, yunit);

        end

    end
end