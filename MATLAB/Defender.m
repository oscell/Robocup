classdef Defender
    properties
        number
      
    end
    methods
        function obj = Defender()

        end

        function pose = get_pose(obj,team)
            if team == 1

                pose = [2.5,2,0];
            else
                pose = [9.5,2,0];
            end
        end 



    end

end