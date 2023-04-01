classdef BallDynamics
    properties
        Pose=[5.5;4];
        Velocity=[0.1;0.1];
        KVelocity=[];
        C

        orientation
        V

        r

        poses
        dt
        totaltime
    end
    methods
        function obj=BallDynamics(pose,velocity,kvelocity,c,dt,totalTime)
            if nargin==6
                obj.dt = dt;
                obj.totaltime = totalTime;
                obj.poses = zeros(numel(0:dt:totalTime),2);
                
                obj.r = rateControl(1/dt);

                obj.Velocity = [0;0];
                obj.KVelocity = kvelocity;
                obj.C = c;

                obj.orientation = (5/3)*pi - pi;
%                 obj.V = sqrt(velocity(1,1)^2 + velocity(1,1)^2);
                obj.V = 0.01;
            end 
        end
        
        function obj= update(obj,idx)

            obj.Pose=obj.Pose+obj.Velocity*obj.dt;

            obj.Velocity=obj.Velocity-(obj.C*obj.Velocity);

%             disp(obj.Pose)
%             disp(obj.Velocity)
                        
            
            
            if obj.Velocity<=0
                obj.Velocity=[0;0];
                obj.orientation = 0;
            else
                obj.orientation = asin(obj.Velocity(1,2)/sqrt(obj.Velocity(1,2)^2+obj.Velocity(1,2)^2));
            end
%             disp(obj.orientation);
%             disp(['vely = ',obj.Velocity])


%             waitfor(obj.r);

            
            


        end

        function obj= update_kick(obj,idx,V,phi)
%             disp(phi)
            x_tdot = [V*cos(phi); V*sin(phi)];
%             disp(x_tdot)
            obj.Pose=obj.Pose+x_tdot*obj.dt;
%             waitfor(obj.r);
        end

        function show(obj)
            plot(obj.Pose(1), obj.Pose(2),'o', 'Color', 'k', "MarkerFaceColor", 'k', 'MarkerSize', 10)

            x_tdot = [obj.V*cos(obj.orientation); obj.V*sin(obj.orientation)];
%             disp(x_tdot)
            plot([obj.Pose(1,1), obj.Pose(1,1)+x_tdot(1)*0.5],[obj.Pose(2,1), obj.Pose(2,1)+x_tdot(2)*0.5],Color='r',LineWidth=1)
        end

        function obj=Drible(obj)
            obj.Pose=obj.Pose+obj.Velocity;
            waitfor(obj.r);
        end
        function obj=Kick(obj)
            obj.Pose=obj.Pose+obj.KVelocity;
            obj.KVelocity=obj.KVelocity-(obj.C*obj.KVelocity);
            if obj.Velocity<=0
                obj.Velocity=0;
            end
            pause(0.15);
        end    
    end
end