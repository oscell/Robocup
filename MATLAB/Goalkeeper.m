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
%                 pose = [0;0;(1/3)*pi];
            else
                pose = [9.8;4;pi];
            end
        
        end 



    end

end