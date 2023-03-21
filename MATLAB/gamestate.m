classdef gamestate
    properties
        isout
        isgoal
    end
    methods
        function obj = gamestate()
            isout = false;
            isgoal = false;
        
        end


        function check_out()

        end

        function check_goal()

        end

        function state_to_csv(game_state, filename)
        csvwrite(file_name, game_state);
        
        end

    end
end
