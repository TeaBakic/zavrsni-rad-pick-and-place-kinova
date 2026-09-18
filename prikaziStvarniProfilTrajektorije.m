%% prikaziStvarniProfilTrajektorije.m
%
% Generira i crta STVARNI profil polozaja, brzine i ubrzanja jednog
% zgloba, izracunat na isti nacin kao u izvornoj implementaciji
% (exampleCommandMoveToTaskConfigROSGazeboScene.m), za stvarnu RRT putanju.
%
% PRETPOSTAVKA: 'coordinator' objekt vec postoji u workspaceu (nakon
% Build Environment, Detect Parts, Picking Logic i Compute Grasp Pose
% koraka - isti redoslijed kao za mjeriRRTParametre.m).
%
robot = coordinator.Robot;
world = coordinator.World;
startConfig = coordinator.CurrentRobotJConfig;

ik = inverseKinematics('RigidBodyTree', robot);
rng(2);   % isto sjeme kao u izvornom kodu (poglavlje 3.2)
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
robotPos = positions';              % numJoints x N

numJoints = coordinator.NumJoints;
jointToPlot = 1;                    % promijeni ovdje ako zelis drugi zglob (1-7)

%% Generiranje trajektorije - isti postupak kao u izvornom kodu, h = 0.03
h = 0.03;
timeInterval = [0; h*size(positions,1) - h];
[~, sd, ~, ~, ~] = trapveltraj(timeInterval', size(positions,1));
timeSteps = ((-sd + max(sd)) / max(sd)) * 3 * h + h;
trajTimes = [0, cumsum(timeSteps(1:end-1))];

robotVelTemp = (diff(robotPos')) ./ diff(trajTimes)';
robotVel = [zeros(1, numJoints); robotVelTemp];

robotAccTemp = diff(robotVel) ./ diff(trajTimes)';
robotAcc = [zeros(2, numJoints); robotAccTemp];
N = numel(trajTimes);
robotAcc = robotAcc(1:N, :);

%% Crtanje
figure('Name', 'Stvarni profil trajektorije iz simulacije', 'Position', [100 100 700 650]);

subplot(3,1,1);
plot(trajTimes, robotPos(jointToPlot,:), 'LineWidth', 1.5);
ylabel('Polo\v{z}aj (rad)', 'Interpreter', 'latex');
title(sprintf('Zglob %d - stvarni profil iz simulacije (h = %.2f s)', jointToPlot, h));
grid on;

subplot(3,1,2);
plot(trajTimes, robotVel(:,jointToPlot), 'LineWidth', 1.5);
ylabel('Brzina (rad/s)');
grid on;

subplot(3,1,3);
plot(trajTimes(1:end-1), robotAcc(1:end-1,jointToPlot), 'LineWidth', 1.5);
ylabel('Ubrzanje (rad/s^2)');
xlabel('Vrijeme (s)');
grid on;

saveas(gcf, 'profil_stvarni_zglob.png');
disp('Graf spremljen kao profil_stvarni_zglob.png u trenutnom direktoriju (pwd).');
