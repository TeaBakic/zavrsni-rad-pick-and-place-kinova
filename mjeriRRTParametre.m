%% mjeriRRTParametre.m
%
% Skripta mjeri utjecaj tri RRT parametra (MaxConnectionDistance,
% ValidationDistance, EnableConnectHeuristic) na cetiri velicine:
% vrijeme planiranja, duljinu puta, broj tocaka puta i postotak
% uspjesnosti planiranja.
%
ik = inverseKinematics('RigidBodyTree', robot);
rng(1);
goalConfig = ik(coordinator.RobotEndEffector, coordinator.GraspPose, ...
    ones(1,6), startConfig);

numTrials = 15;   % broj ponavljanja po vrijednosti parametra (za postotak uspjesnosti)
shortenIters = 40; % isto kao u izvornoj implementaciji

%% Pomocna funkcija za testiranje jednog parametra
function resultsTable = testParameter(robot, world, startConfig, goalConfig, ...
        paramName, paramValues, numTrials, shortenIters)

    resultsTable = table();

    for k = 1:numel(paramValues)
        v = paramValues(k);

        times   = [];
        lengths = [];
        npoints = [];
        successes = 0;

        for trial = 1:numTrials
            rng(trial);  % razlicito sjeme svaki pokusaj -> mjeri se pouzdanost

            planner = manipulatorRRT(robot, world);
            planner.MaxConnectionDistance = 0.2;   % polazna (default) vrijednost
            planner.ValidationDistance    = 0.2;   % polazna (default) vrijednost
            planner.EnableConnectHeuristic = true; % polazna (default) vrijednost
            planner.SkippedSelfCollisions = "adjacent";

            switch paramName
                case "MaxConnectionDistance"
                    planner.MaxConnectionDistance = v;
                case "ValidationDistance"
                    planner.ValidationDistance = v;
                case "EnableConnectHeuristic"
                    planner.EnableConnectHeuristic = logical(v);
            end

            try
                ticVal = tic;
                path = planner.plan(startConfig, goalConfig);
                elapsed = toc(ticVal);

                path = planner.shorten(path, shortenIters);

                pathLength = sum(vecnorm(diff(path), 2, 2));

                times(end+1)   = elapsed;      %#ok<AGROW>
                lengths(end+1) = pathLength;    %#ok<AGROW>
                npoints(end+1) = size(path, 1); %#ok<AGROW>
                successes = successes + 1;
            catch
            end
        end

        if isempty(times)
            meanTime = NaN; meanLen = NaN; meanPts = NaN;
        else
            meanTime = mean(times);
            meanLen  = mean(lengths);
            meanPts  = mean(npoints);
        end
        successRate = 100 * successes / numTrials;

        newRow = table(v, meanTime, meanLen, meanPts, successRate, ...
            'VariableNames', {char(paramName), 'VrijemePlaniranja_s', ...
            'DuljinaPuta_m', 'BrojTocakaPuta', 'Uspjesnost_pct'});

        resultsTable = [resultsTable; newRow]; %#ok<AGROW>
    end
end

%% Test 1: MaxConnectionDistance
mcdValues = [0.1, 0.2, 0.3, 0.4];
resultsMCD = testParameter(robot, world, startConfig, goalConfig, ...
    "MaxConnectionDistance", mcdValues, numTrials, shortenIters);
disp('--- MaxConnectionDistance ---');
disp(resultsMCD);
writetable(resultsMCD, 'rezultati_MaxConnectionDistance.csv');

%% Test 2: ValidationDistance
vdValues = [0.05, 0.1, 0.2, 0.4];
resultsVD = testParameter(robot, world, startConfig, goalConfig, ...
    "ValidationDistance", vdValues, numTrials, shortenIters);
disp('--- ValidationDistance ---');
disp(resultsVD);
writetable(resultsVD, 'rezultati_ValidationDistance.csv');

%% Test 3: EnableConnectHeuristic
echValues = [0, 1]; % false, true
resultsECH = testParameter(robot, world, startConfig, goalConfig, ...
    "EnableConnectHeuristic", echValues, numTrials, shortenIters);
disp('--- EnableConnectHeuristic ---');
disp(resultsECH);
writetable(resultsECH, 'rezultati_EnableConnectHeuristic.csv');

%% Gotovo
disp('Sve tri tablice spremljene su kao .csv datoteke u trenutnom MATLAB direktoriju.');
