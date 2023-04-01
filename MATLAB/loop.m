% Set the number of times to run the main.m file
numRuns = 5;
goal1 = zeros(1,numRuns);
goal2 = zeros(1,numRuns);
out = zeros(1,numRuns);
goal1Total = 0;
goal2Total = 0;
outTotal = 0;
match = [1,2,3,4,5];
% Loop through the number of times to run main.m
for i = 1:numRuns
    % Call the main.m file
    % main; % uncomment when running main 5 times
    % Load the CSV file into a table
    T = readtable('gamestate.csv');

    % Get the last row of the table
    lastRow = table2array(T(end, :));
    goal1(i) = lastRow(2);
    goal2(i) = lastRow(3);
    out(i) = lastRow(4);
    goal1Total = goal1Total + lastRow(2);
    goal2Total = goal2Total + lastRow(3);
    outTotal = outTotal + lastRow(4);
end

figure(1);
subplot(1,2,1);
avgGoal1 = mean(goal1);
hold on
line([0, 6], [avgGoal1, avgGoal1], 'Color', 'r', 'LineWidth', 2)
bar(match, goal1)
xlabel('Match Number')
ylabel('Goal 1 Scored')
title('Goal 1 Scores by Match')
hold off

subplot(1,2,2);
avgGoal2 = mean(goal2);
hold on
line([0, 6], [avgGoal2, avgGoal2], 'Color', 'r', 'LineWidth', 2)
bar(match, goal2)
xlabel('Match Number')
ylabel('Goal 2 Scored')
title('Goal 2 Scores by Match')
hold off

figure(2);
avgOut = mean(out);
hold on
line([0, 6], [avgOut, avgOut], 'Color', 'r', 'LineWidth', 2)
bar(match, out)
xlabel('Match Number')
ylabel('Out Number')
title('Out Number by Match')
hold off


