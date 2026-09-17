%% snimiPointCloudIRGB.m
%
% Snima JEDAN sirovi point cloud i JEDNU RGB sliku iz trenutne Gazebo
% scene, koristeci vec postojece pretplate unutar coordinator objekta
% (obj.ROSinfo.pointCloudSub i obj.ROSinfo.rgbImgSub).
%
% PRETPOSTAVKA: 'coordinator' vec postoji u workspaceu (nakon rosinit
% i inicijalizacije coordinatora - ne treba ni Build Environment ni
% ostale korake, dovoljno je da je ROS veza uspostavljena i da robot
% stoji u nekoj razumnoj pozi s koje kamera gleda prema stolu).
%
% Rezultat: dvije odvojene slike (point_cloud_sirovi.png,
% rgb_scena.png) i jedna kombinirana slika sa obje jedna pored druge
% (point_cloud_i_rgb.png), spremne za umetanje u rad.

%% Snimi RGB sliku
rgbMsg = receive(coordinator.ROSinfo.rgbImgSub, 10);   % timeout 10 s
rgbImage = readImage(rgbMsg);

%% Snimi sirovi point cloud
pcMsg = receive(coordinator.ROSinfo.pointCloudSub, 10);
xyz = readXYZ(pcMsg);
xyz = double(rmmissing(xyz));      % ukloni NaN vrijednosti (isto kao u izvornom kodu)
ptCloudSirovi = pointCloud(xyz);

%% Prikazi i spremi odvojeno
figure('Name', 'RGB slika scene');
imshow(rgbImage);
title('RGB slika scene');
saveas(gcf, 'rgb_scena.png');

figure('Name', 'Sirovi point cloud scene');
pcshow(ptCloudSirovi);
title('Sirovi point cloud (prije obrade)');
xlabel('X'); ylabel('Y'); zlabel('Z');
saveas(gcf, 'point_cloud_sirovi.png');

%% Kombinirana slika - RGB i point cloud jedno pored drugog
figure('Name', 'RGB i point cloud usporedba', 'Position', [100 100 1000 450]);

subplot(1,2,1);
imshow(rgbImage);
title('RGB slika scene');

subplot(1,2,2);
pcshow(ptCloudSirovi);
title('Sirovi point cloud (prije obrade)');
xlabel('X'); ylabel('Y'); zlabel('Z');

saveas(gcf, 'point_cloud_i_rgb.png');

disp('Gotovo. Spremljene tri datoteke u trenutnom direktoriju (provjeri: pwd):');
disp('  - rgb_scena.png');
disp('  - point_cloud_sirovi.png');
disp('  - point_cloud_i_rgb.png  (obje slike jedna pored druge)');
