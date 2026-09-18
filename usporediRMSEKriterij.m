%% usporediRMSEKriterij.m  
%
% Uspoređuje redoslijed hvatanja predmeta prema kriteriju "najbolji RMSE prvi". 
% PRETPOSTAVKA: 'coordinator' već postoji, Build Environment i Detect Parts su već pokrenuti.

%% Učitavanje CAD modela (isto kao u izvornom kodu)
bottle = stlread('exampleHelperBottlePoints.stl');
can = stlread('exampleHelperCanPoints.stl');
pcBottle = pointCloud(bottle.Points);
pcCan = pointCloud(can.Points);

%% Uzmi oba predmeta
part1 = coordinator.Parts{1};
part2 = coordinator.Parts{2};

function rmseVal = izracunajRMSE(part, pcBottle, pcCan, coordinator)
    segment = coordinator.PointCloudSegments{part.index};
    if part.type == 1
        referenca = pcBottle;
    else
        referenca = pcCan;
    end
    [~, ~, rmseVal] = pcregistericp(segment, referenca, 'Metric', 'planeToPlane');
end

rmse1 = izracunajRMSE(part1, pcBottle, pcCan, coordinator);
rmse2 = izracunajRMSE(part2, pcBottle, pcCan, coordinator);

nazivi = ["Boca", "Limenka"];
naziv1 = nazivi(part1.type);
naziv2 = nazivi(part2.type);

fprintf('RMSE za %s: %.4f\n', naziv1, rmse1);
fprintf('RMSE za %s: %.4f\n', naziv2, rmse2);

%% Odredi bolji redoslijed prema kriteriju
if rmse1 <= rmse2
    preporuceniRedoslijed = sprintf('%s, zatim %s', naziv1, naziv2);
    razlika = rmse2 - rmse1;
else
    preporuceniRedoslijed = sprintf('%s, zatim %s', naziv2, naziv1);
    razlika = rmse1 - rmse2;
end

fprintf('\nPrema RMSE kriteriju, preporučeni redoslijed: %s\n', preporuceniRedoslijed);
fprintf('Razlika u RMSE vrijednostima: %.4f\n', razlika);

resultsTable = table([naziv1; naziv2], [rmse1; rmse2], ...
    'VariableNames', {'Predmet', 'RMSE'});
disp(resultsTable);

writetable(resultsTable, 'usporedba_rmse_kriterija.csv');
disp('Tablica spremljena kao usporedba_rmse_kriterija.csv');
