classdef Attacker
    properties
        is_repeated

        decisions
        decision
        name = "Attacker"       
      
    end
    methods
        function obj = Attacker(is_repeated)
            obj.is_repeated = is_repeated;



        end

        function pose = get_pose(obj,team)
            if team == 1
                if obj.is_repeated
                    pose = [4.5;3;0];
                else
                    pose = [4.5;4;0];
                end
            else

                if obj.is_repeated
                    pose = [11-4.5;3;-pi];
                else
                    pose = [11-4.5;4;pi];
                end
                
            end
        
        end
        function pose = getGoalpose(obj,ball,team)
            pose = zeros(1,3);
            pose(1) = ball.Pose(1);
            pose(2) = ball.Pose(2);
            if team == 1
                pose(3) = 1;
            else
               pose(3) = 0;
            end
        end

        function boundary = get_boundary(obj,team)            
            boundary = [1 7.0; 10.0 1.0];
        end


    end

end