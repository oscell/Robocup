classdef BallDynamics
    properties
        Pose=[];
        Velocity=[];
        KVelocity=[];
        C

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

                obj.Pose = pose;
                obj.Velocity = velocity;
                obj.KVelocity = kvelocity;
                obj.C = c;
            end
        end    
        function obj= update(obj,idx)
            obj.Pose=obj.Pose+obj.Velocity;
            obj.Velocity=obj.Velocity-(obj.C*obj.Velocity);
            if obj.Velocity<=0
                obj.Velocity=0;
            end
            pause(0.15);
        end
        function show(obj)
            plot(obj.Pose(1,1), obj.Pose(1,2),'o', 'Color', 'k', "MarkerFaceColor", 'k', 'MarkerSize', 10)
        end
        function obj=Drible(obj)
            obj.Pose=obj.Pose+obj.Velocity;
            pause(0.15)
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