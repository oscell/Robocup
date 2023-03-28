classdef Goalkeeper
    properties
        is_repeated
        name = "Goalkeeper"
        
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

        function boundary = get_boundary(obj,team)            
            if team == 1
                boundary = [1.0 6.5; 3 1.5];
            else
                boundary = [8.0 6.5; 10.0 1.5];
            end
        end




    end

end