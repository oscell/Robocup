classdef Attacker
    properties

      
    end
    methods
        function obj = Attacker()

        end

        function pose = get_pose(obj,team)
            if team == 1

                pose = [4.5,2,0];
            else
                pose = [6.5,2,0];
            end
        
        end 



    end

end