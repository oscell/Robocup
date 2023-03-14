classdef Defender
    properties
        is_repeated
        name = "Defender"
      
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



    end

end