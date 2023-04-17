classdef BallTracker
    properties
        fieldPos % [left, bottom, right, top]
        goalRight %[left, bottom, right, top]
        goalLeft %[left, bottom, right, top]
        ballPos % [x, y]
        scoreLeft
        scoreRight
        isInside
        
        score = 0
        timer = 0
    end

    methods
        function obj = BallTracker(fieldPos, ballPos, goalLeft, goalRight, scoreLeft, scoreRight)
            obj.fieldPos = fieldPos;
            obj.ballPos = ballPos;
            obj.goalLeft = goalLeft;
            obj.goalRight = goalRight;
            obj.scoreLeft = scoreLeft;
            obj.scoreRight = scoreRight;
        end

        function isOut = isBallOutOfBounds(obj)
            % Determine if the ball is out of bounds
            isOut = false;
            if obj.ballPos(1) < obj.fieldPos(1) || obj.ballPos(1) > obj.fieldPos(3)
                isOut = true;
            elseif obj.ballPos(2) < obj.fieldPos(2) || obj.ballPos(2) > obj.fieldPos(4)
                isOut = true;
            end
        end

        function isInLG = isBallInLeftGoal(obj)
            isInLG = false;
            if obj.ballPos(1) > obj.goalLeft(1) && obj.ballPos(1) < obj.goalLeft(3)...
                    && obj.ballPos(2) < obj.goalLeft(2) && obj.ballPos(2) > obj.goalLeft(4)
                isInLG = true;
            end
        end

        function isInRG = isBallInRightGoal(obj)
            isInRG = false;
            if obj.ballPos(1) < obj.goalRight(1) && obj.ballPos(1) > obj.goalRight(3)...
                    &&obj.ballPos(2) < obj.goalRight(2) && obj.ballPos(2) > obj.goalRight(4)
                isInRG = true;
            end
        end

        function [obj, ball] = updateBallPos(obj, ballPos, scoreLeft, scoreRight,ball)
            % Update the coordinates of the ball
            obj.ballPos = ballPos;
            obj.scoreLeft = scoreLeft;
            obj.scoreRight = scoreRight;
            
            if ball.Pose(1) > 10
                obj.scoreLeft = obj.scoreLeft + 1;
                obj.score = 1;
            end
            if obj.isBallInLeftGoal()
                text(2, 7.5, 'Left goal!', 'Color', 'r', 'FontSize', 20);
                obj.scoreLeft = obj.scoreLeft + 1;
            end
            if obj.isBallInRightGoal()
                text(2, 7.5, 'Right goal!', 'Color', 'r', 'FontSize', 20);
                obj.scoreRight = obj.scoreRight + 1;
            end

            % Check if the ball is out of bounds
            if obj.isBallOutOfBounds()
                text(2, 7.5, 'Goal for blue team!', 'Color', 'r', 'FontSize', 20);
                obj.timer = obj.timer + 1;
                if obj.timer > 20
                    ball.Pose=[5.5;4];
                    ball.dribblingRobotID = [];
                    ball.V = 0;

                end
            end

        end
        function showScores(obj)
            if obj.score == 1
                     text(3, 0.5, num2str(obj.scoreRight), 'FontSize', 24, 'HorizontalAlignment', 'right');
            text(8, 0.5, num2str(1), 'FontSize', 24, 'HorizontalAlignment', 'left');
            else
                text(3, 0.5, num2str(obj.scoreRight), 'FontSize', 24, 'HorizontalAlignment', 'right');
            text(8, 0.5, num2str(obj.scoreLeft), 'FontSize', 24, 'HorizontalAlignment', 'left');
            end
            

        end

        function obj = isPointInRectangle(x, y, x1, y1, x2, y2)
        % isPointInRectangle Check if a point is within a rectangle
        %   INPUTS: 
        %       x, y : coordinates of the point to check
        %       x1, y1 : coordinates of the bottom-left corner of the rectangle
        %       x2, y2 : coordinates of the top-right corner of the rectangle
        %
        %   OUTPUTS:
        %       isInside : returns true if the point is inside the rectangle, false otherwise
        
            % Check if the point is within the horizontal boundaries
            isInsideX = x >= x1 && x <= x2;
        
            % Check if the point is within the vertical boundaries
            isInsideY = y >= y1 && y <= y2;
        
            % Check if the point is within both the horizontal and vertical boundaries
            obj.isInside = isInsideX && isInsideY;
        
        end

    end
end

