function provjeriRobusnost()
% PROVJERIROBUSNOST  Za trenutno stanje scene, provjerava detekciju,
% izracunava RMSE i vizualizira centroide oba predmeta na spojenom
% point cloudu.
%
% Koristi se za eksperimente robusnosti opisane u poglavlju 5.4
% zavrsnog rada, NAKON sto su vec pozvani:
%   postavljanjeScene(neki_scenarij)
%   exampleCommandBuildWorldROSGazeboScene(coordinator)
%   exampleCommandDetectPartsROSGazeboScene(coordinator)
%
% PRETPOSTAVKA: varijabla 'coordinator' postoji u pozivnom
% (base/workspace) kontekstu, te da su exampleHelperBottlePoints.stl i
% exampleHelperCanPoints.stl (dio izvornog MathWorks primjera)
% dostupni na MATLAB putanji.
%
% PRIMJER:
%   postavljanjeScene('lezece');
%   exampleCommandBuildWorldROSGazeboScene(coordinator);
%   exampleCommandDetectPartsROSGazeboScene(coordinator);
%   provjeriRobusnost();

    coordinator = evalin('base', 'coordinator');

    % --- provjeri je li detekcija uspjela ---
    try
        n = numel(coordinator.Parts);
    catch err
        fprintf('DETEKCIJA NIJE USPJELA: %s\n', err.message);
        return;
    end
    fprintf('Broj prepoznatih predmeta: %d\n', n);
    if n ~= 2
        warning('Ocekivana su 2 predmeta, prepoznato je %d. Provjeri scenu.', n);
    end

    % --- ucitaj referentne CAD modele ---
    bottle = stlread('exampleHelperBottlePoints.stl');
    can = stlread('exampleHelperCanPoints.stl');
    pcBottle = pointCloud(bottle.Points);
    pcCan = pointCloud(can.Points);

    % --- prikazi spojeni point cloud kao pozadinu ---
    figure('Name', 'Provjera robusnosti');
    pcshow(coordinator.MergedPointCloud);
    hold on;

    boje = {'r*', 'g*'};

    for i = 1:n
        part = coordinator.Parts{i};
        seg = coordinator.PointCloudSegments{part.index};

        if part.type == 1
            ref = pcBottle;
            naziv = 'boca';
        else
            ref = pcCan;
            naziv = 'limenka';
        end

        [~, ~, rmseVal] = pcregistericp(seg, ref, 'Metric', 'planeToPlane');
        fprintf('Predmet %d (%s): RMSE = %.4f\n', i, naziv, rmseVal);

        cp = part.centerPoint;
        plot3(cp(1), cp(2), cp(3), boje{i}, 'MarkerSize', 20, 'LineWidth', 3);
    end

    title('Provjera centroida - crvena = predmet 1, zelena = predmet 2');
end
