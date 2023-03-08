classdef Attacker
    properties
        is_repeated

        decisions
        decision

      
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
                    pose = [4.5;8-3;0];
                end
            else

                if obj.is_repeated
                    pose = [11-4.5;3;0];
                else
                    pose = [11-4.5;8-3;0];
                end
                
            end
        
        end
        function Decision = makeDecision(obj)
        end



    end

end