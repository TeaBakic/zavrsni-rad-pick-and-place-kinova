%% usporediRMSEKriterij.m  (ISPRAVLJENA VERZIJA)
%
% Uspoređuje redoslijed hvatanja predmeta prema kriteriju "najbolji RMSE
% prvi". Budući da coordinator.Parts NE sprema RMSE vrijednost (koristi
% se samo privremeno tijekom detekcije, pa se odbacuje), ova skripta
% RMSE ponovno izračunava izravno, istim postupkom kao u
% prikaziICPPoravnanje.m: uzima pripadajući segment iz
% coordinator.PointCloudSegments (prema polju .index) i poravnava ga s
% odgovarajućim CAD modelom (prema polju .type).
%
% PRETPOSTAVKA: 'coordinator' već postoji, Build Environment i
% Detect Parts su već pokrenuti.

%% Učitaj CAD modele (isto kao u izvornom kodu)
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
% dodijeli citljiv naziv prema type polju (1 = boca, 2 = limenka), ne
% prema pretpostavci da je Parts{1} uvijek boca
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

%% Spremi rezultat kao tablicu
resultsTable = table([naziv1; naziv2], [rmse1; rmse2], ...
    'VariableNames', {'Predmet', 'RMSE'});
disp(resultsTable);

writetable(resultsTable, 'usporedba_rmse_kriterija.csv');
disp('Tablica spremljena kao usporedba_rmse_kriterija.csv');
