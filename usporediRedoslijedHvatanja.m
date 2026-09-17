%% usporediRedoslijedHvatanja.m  (PRECIZNA, LANCANA VERZIJA)
%
% Uspoređuje ukupnu duljinu puta robota za dva moguća redoslijeda
% hvatanja predmeta, prateci STVARNI lanac pokreta:
%   Redoslijed A (fiksni): Home -> Grasp1 -> Place1 -> Home -> Grasp2 -> Place2 -> Home
%   Redoslijed B:           Home -> Grasp2 -> Place2 -> Home -> Grasp1 -> Place1 -> Home
%
% Za razliku od prve verzije, svaka etapa POCINJE od stvarne konfiguracije
% u kojoj je prethodna etapa ZAVRSILA, ne uvijek od Home-a - pa zbroj
% duljina puta stvarno ovisi o redoslijedu.
%
% PRETPOSTAVKA: 'coordinator' već postoji, Build Environment i
% Detect Parts su već pokrenuti.

robot = coordinator.Robot;
world = coordinator.World;
homeConfig = coordinator.CurrentRobotJConfig;
ik = inverseKinematics('RigidBodyTree', robot);

%% Postavi fiksne PlacingPose (isto kao u glavnoj skripti primjera)
coordinator.PlacingPose{1} = trvec2tform([0.2 0.55 0.26]) * axang2tform([0 0 1 pi/2]) * axang2tform([0 1 0 pi]);
coordinator.PlacingPose{2} = trvec2tform([0.2 -0.55 0.26]) * axang2tform([0 0 1 pi/2]) * axang2tform([0 1 0 pi]);

%% Izračunaj GraspPose za oba predmeta
coordinator.NextPart = 1;
exampleCommandComputeGraspPoseROSGazeboScene(coordinator);
grasp1 = coordinator.GraspPose;
place1 = coordinator.PlacingPose{1};

coordinator.NextPart = 2;
exampleCommandComputeGraspPoseROSGazeboScene(coordinator);
grasp2 = coordinator.GraspPose;
place2 = coordinator.PlacingPose{2};

%% Pomocna funkcija: planiraj put, vrati duljinu I zavrsnu konfiguraciju
function [len, endConfig] = duljinaIKraj(robot, world, ik, startConfig, targetPose, endEffector)
    rng(2);
    targetConfig = ik(endEffector, targetPose, ones(1,6), startConfig);

    rng(1);
    planner = manipulatorRRT(robot, world);
    planner.MaxConnectionDistance = 0.2;
    planner.ValidationDistance = 0.2;
    planner.EnableConnectHeuristic = true;
    planner.SkippedSelfCollisions = "adjacent";

    path = planner.plan(startConfig, targetConfig);
    path = planner.shorten(path, 40);
    len = sum(vecnorm(diff(path), 2, 2));
    endConfig = path(end, :);
end

%% Pomocna funkcija: izvrsi cijeli ciklus za JEDAN predmet (grasp->place->home)
%  Vraca ukupnu duljinu tog ciklusa, pocevsi od zadane pocetne konfiguracije.
function [ukupno, konfigNaKraju] = ciklusZaPredmet(robot, world, ik, startConfig, homePose, graspPose, placePose, endEffector)
    [len1, cfgAfterGrasp] = duljinaIKraj(robot, world, ik, startConfig, graspPose, endEffector);
    [len2, cfgAfterPlace] = duljinaIKraj(robot, world, ik, cfgAfterGrasp, placePose, endEffector);
    [len3, cfgAfterHome]  = duljinaIKraj(robot, world, ik, cfgAfterPlace, homePose, endEffector);
    ukupno = len1 + len2 + len3;
    konfigNaKraju = cfgAfterHome;
end

%% Pripremi pocetnu (Home) pozu kao tform, i naziv end-effectora
endEffector = coordinator.RobotEndEffector;
homePose = getTransform(robot, homeConfig, endEffector);

%% Redoslijed A: boca prvo, zatim limenka (fiksni redoslijed)
[lenA1, cfgA1] = ciklusZaPredmet(robot, world, ik, homeConfig, homePose, grasp1, place1, endEffector);
[lenA2, ~]     = ciklusZaPredmet(robot, world, ik, cfgA1, homePose, grasp2, place2, endEffector);
lenA = lenA1 + lenA2;

%% Redoslijed B: limenka prvo, zatim boca
[lenB1, cfgB1] = ciklusZaPredmet(robot, world, ik, homeConfig, homePose, grasp2, place2, endEffector);
[lenB2, ~]     = ciklusZaPredmet(robot, world, ik, cfgB1, homePose, grasp1, place1, endEffector);
lenB = lenB1 + lenB2;

%% Rezultat
resultsTable = table(["Boca, zatim limenka (fiksni)"; "Limenka, zatim boca"], ...
    [lenA; lenB], 'VariableNames', {'Redoslijed', 'UkupnaDuljinaPuta_m'});
disp(resultsTable);

if lenA <= lenB
    fprintf('\nKra\xE6i (bolji) redoslijed: BOCA prvo, zatim LIMENKA — razlika: %.4f m\n', lenB - lenA);
else
    fprintf('\nKra\xE6i (bolji) redoslijed: LIMENKA prvo, zatim BOCA — razlika: %.4f m\n', lenA - lenB);
end

writetable(resultsTable, 'usporedba_redoslijeda.csv');
disp('Tablica spremljena kao usporedba_redoslijeda.csv');
