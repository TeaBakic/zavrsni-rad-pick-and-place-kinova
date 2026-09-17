%% mjeriTrajektorijuParametre.m
%
% Mjeri utjecaj osnovnog vremenskog koraka h (koristenog u generiranju
% trajektorije putem trapveltraj) na:
%   - ukupno vrijeme gibanja
%   - profil polozaja, brzine i ubrzanja jednog reprezentativnog zgloba
%
% PRETPOSTAVKA: 'coordinator' objekt vec postoji u workspaceu (nakon
% Build Environment, Detect Parts i Compute Grasp Pose koraka - isto
% kao za mjeriRRTParametre.m).
%
% Za razliku od mjeriRRTParametre.m, ovdje se RRT put isplanira SAMO
% JEDNOM (uz polazne, fiksne RRT postavke), kako bi sve razlike u
% rezultatima dosljedno dolazile iskljucivo od promjene parametra h,
% a ne od razlicitih putova u razlicitim pokusajima.
%
% Rezultat: tablica (Command Window + .csv) i graf sa tri panela
% (polozaj / brzina / ubrzanje) koji uspoređuje tri razlicite
% vrijednosti h na istom putu, spremljen kao .png spreman za umetanje
% u rad.

%% Priprema - isplaniraj JEDAN put koji ce se koristiti za sve testove
robot = coordinator.Robot;
world = coordinator.World;
startConfig = coordinator.CurrentRobotJConfig;

ik = inverseKinematics('RigidBodyTree', robot);
rng(1);
goalConfig = ik(coordinator.RobotEndEffector, coordinator.GraspPose, ...
    ones(1,6), startConfig);

rng(1);
planner = manipulatorRRT(robot, world);
planner.MaxConnectionDistance = 0.2;
planner.ValidationDistance = 0.2;
planner.EnableConnectHeuristic = true;
planner.SkippedSelfCollisions = "adjacent";

path = planner.plan(startConfig, goalConfig);
path = planner.shorten(path, 40);
planner.ValidationDistance = 0.02;
positions = interpolate(planner, path);
positions = [startConfig; positions];
robotPos = positions';   % numJoints x N

numJoints = coordinator.NumJoints;
jointToPlot = 1;   % promijeni ako zelis prikazati neki drugi zglob (1-7)

%% Testiraj nekoliko vrijednosti h
hValues = [0.02, 0.03, 0.05];
resultsTable = table();

figure('Name', 'Usporedba profila trajektorije za razlicite h', 'Position', [100 100 800 700]);
colors = lines(numel(hValues));

for k = 1:numel(hValues)
    h = hValues(k);

    timeInterval = [0; h*size(positions,1) - h];
    [~, sd, ~, ~, ~] = trapveltraj(timeInterval', size(positions,1));
    timeSteps = ((-sd + max(sd)) / max(sd)) * 3 * h + h;
    trajTimes = [0, cumsum(timeSteps(1:end-1))];

    robotVelTemp = (diff(robotPos')) ./ diff(trajTimes)';
    robotVel = [zeros(1, numJoints); robotVelTemp];

    robotAccTemp = diff(robotVel) ./ diff(trajTimes)';
    robotAcc = [zeros(2, numJoints); robotAccTemp];

    % Obrezivanje na zajednicku duljinu (izbjegava gresku zbog
    % dodatnog retka koji nastaje u izvornom nacinu racunanja robotAcc)
    N = numel(trajTimes);
    robotAcc = robotAcc(1:N, :);

    totalTime = trajTimes(end);
    maxVel = max(abs(robotVel(:, jointToPlot)));
    maxAcc = max(abs(robotAcc(1:end-1, jointToPlot)));

    newRow = table(h, totalTime, maxVel, maxAcc, ...
        'VariableNames', {'h_s', 'UkupnoVrijeme_s', 'MaxBrzina_rad_s', 'MaxUbrzanje_rad_s2'});
    resultsTable = [resultsTable; newRow]; %#ok<AGROW>

    subplot(3,1,1); hold on; grid on;
    plot(trajTimes, robotPos(jointToPlot,:), 'Color', colors(k,:), ...
        'LineWidth', 1.4, 'DisplayName', sprintf('h = %.2f s', h));
    ylabel('Polozaj (rad)');
    title(sprintf('Zglob %d - profil polozaja', jointToPlot));
    legend('Location', 'best');

    subplot(3,1,2); hold on; grid on;
    plot(trajTimes, robotVel(:,jointToPlot), 'Color', colors(k,:), 'LineWidth', 1.4);
    ylabel('Brzina (rad/s)');
    title('Profil brzine');

    subplot(3,1,3); hold on; grid on;
    plot(trajTimes(1:end-1), robotAcc(1:end-1,jointToPlot), 'Color', colors(k,:), 'LineWidth', 1.4);
    ylabel('Ubrzanje (rad/s^2)');
    xlabel('Vrijeme (s)');
    title('Profil ubrzanja');
end

disp(resultsTable);
writetable(resultsTable, 'rezultati_h_trajektorija.csv');
saveas(gcf, 'profili_trajektorije_usporedba.png');

disp('Gotovo.');
disp('Tablica spremljena kao rezultati_h_trajektorija.csv');
disp('Graf spremljen kao profili_trajektorije_usporedba.png');
disp('Obje datoteke su u trenutnom MATLAB direktoriju (provjeri naredbom: pwd)');
