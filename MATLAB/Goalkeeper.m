classdef Goalkeeper
    properties

      
    end
    methods
        function obj = Goalkeeper()

        end

        function pose = get_pose(obj,team)
            if team == 1
                pose = [1.2,4,0];
            else
                pose = [9.8,4,0];
            end
        
        end 



    end

end