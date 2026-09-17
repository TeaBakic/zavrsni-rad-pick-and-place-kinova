%% mjeriRRTParametre.m
%
% Skripta mjeri utjecaj tri RRT parametra (MaxConnectionDistance,
% ValidationDistance, EnableConnectHeuristic) na cetiri velicine:
% vrijeme planiranja, duljinu puta, broj tocaka puta i postotak
% uspjesnosti planiranja.
%
% PRETPOSTAVKA: 'coordinator' objekt vec postoji u MATLAB workspaceu
% (dakle, vec si pokrenula pripremu i Build Environment korak, pa
% coordinator.Robot i coordinator.World postoje i sadrze stvarni
% model robota i stvarne prepreke iz tvoje scene).
%
% Skripta NE treba aktivnu ROS/Gazebo vezu za samo planiranje (planer
% radi iskljucivo nad MATLAB modelom), pa je mozes pokrenuti i nakon
% sto si zavrsila skeniranje, cak i bez daljnje potrebe za Gazebom.
%
% Rezultat: tri tablice (u Command Windowu) + tri .csv datoteke
% spremne za izravno kopiranje u Word tablicu (Insert -> Table ->
% Insert from file, ili samo copy-paste iz Excela nakon otvaranja csv-a).

%% Postavke koje prilagodi svom scenariju
robot = coordinator.Robot;
world = coordinator.World;

% Pocetna konfiguracija - npr. Home pozicija
startConfig = coordinator.CurrentRobotJConfig;

% Ciljna konfiguracija - npr. konfiguracija za hvatanje prvog predmeta.
% Ako GraspPose jos nije izracunata, izracunaj je prije pokretanja ove
% skripte (exampleCommandComputeGraspPoseROSGazeboScene(coordinator)),
% ili ovdje rucno postavi neku drugu validnu ciljnu pozu.
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

            % Prepisi bas onaj parametar koji se u ovoj petlji testira
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
                % Planiranje nije uspjelo (nije pronaden put) - ne
                % ubraja se u prosjek vremena/duljine, ali utjece na
                % postotak uspjesnosti ispod.
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
disp('Otvori ih u Excelu, kopiraj sadrzaj, i zalijepi kao tablicu u Word (Paste Special -> HTML ili kao obicnu tablicu).');
