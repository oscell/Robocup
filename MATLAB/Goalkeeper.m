classdef Goalkeeper
    properties
        is_repeated

      
    end
    methods
        function obj = Goalkeeper(is_repeated)
            obj.is_repeated = is_repeated;

        end

        function pose = get_pose(obj,team)
            if team == 1
                pose = [1.2;4;0];
            else
                pose = [9.8;4;0];
            end
        
        end 



    end

end