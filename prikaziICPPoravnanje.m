%% prikaziICPPoravnanje.m
%
% Generira STVARNU usporedbu ICP poravnanja "prije" i "poslije" za jedan
% segment iz tvoje scene, koristeci isti postupak kao u izvornom kodu
% (exampleCommandDetectPartsROSGazeboScene.m), s metrikom planeToPlane.
%
% PRETPOSTAVKA: 'coordinator' vec postoji i Build Environment je vec
% pokrenut (dakle coordinator.PointCloudSegments postoji i sadrzi
% segmentirane klastere iz scene).
%
%% Ucitavanje CAD modela (isto kao u izvornom kodu)
bottle = stlread('exampleHelperBottlePoints.stl');
can = stlread('exampleHelperCanPoints.stl');
pcBottle = pointCloud(bottle.Points);
pcCan = pointCloud(can.Points);

segmentIndex = 5;
referenceModel = pcBottle;   % promijeni u pcCan za limenku
referenceName = 'model boce';

segment = coordinator.PointCloudSegments{segmentIndex};

%% Prikaz PRIJE poravnanja
figure('Name', 'ICP poravnanje - prije i poslije', 'Position', [100 100 1000 450]);

subplot(1,2,1);
pcshowpair(segment, referenceModel);
title(sprintf('Prije poravnanja (segment %d naspram %s)', segmentIndex, referenceName));

%% Registracija (isti poziv kao u izvornom kodu, planeToPlane metrika)
[tform, segmentAligned, rmse] = pcregistericp(segment, referenceModel, 'Metric', 'planeToPlane');

fprintf('RMSE nakon poravnanja: %.4f\n', rmse);

%% Prikaz POSLIJE poravnanja
subplot(1,2,2);
pcshowpair(segmentAligned, referenceModel);
title(sprintf('Poslije poravnanja (RMSE = %.4f)', rmse));

saveas(gcf, 'icp_poravnanje.png');
disp('Graf spremljen kao icp_poravnanje.png u trenutnom direktoriju (pwd).');
