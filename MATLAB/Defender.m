classdef Defender
    properties
        is_repeated
        name = "Defender"
        boundary = 5.5;
      
    end
    methods
        function obj = Defender(is_repeated)
            obj.is_repeated = is_repeated;
        end

        function pose = get_pose(obj,team)
             if team == 1
                if obj.is_repeated
                    pose = [2.5;2;0];
                else
                    pose = [2.5;8-2;0];
                end
            else

                if obj.is_repeated
                    pose = [11 - 2.5;2;pi];
                else
                    pose = [11 - 2.5;8-2;pi];
                end
                
            end           
        end 
        function pose = getGoalpose(obj,ball,team)
            pose = zeros(1,3);
            pose(1) = 0;
            pose(2) = 0;
            if team == 1
                pose(3) = pi;
            else
               pose(3) = -pi;
            end
        end
        function boundary = get_boundary(obj,team)
            if team == 1
                boundary = [1.0 7.0; 5.5 1.0];
            else
                boundary = [5.5 7.0; 10.0 1.0];
            end
        end


    end

end