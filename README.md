# zavrsni-rad-pick-and-place-kinova

Ovaj repozitorij sadrži dodatne MATLAB skripte razvijene za potrebe završnog rada
"Autonomno pretraživanje okoline primjenom 3D vizijskog sustava i robotske ruke"
(FSB Zagreb, 2026.).

## Preduvjeti

Skripte su izgrađene kao dodatak MathWorks-ovom primjeru:

**Pick-and-Place Workflow in Gazebo Using Point-Cloud Processing and RRT Path Planning**
https://www.mathworks.com/help/robotics/ug/pick-and-place-gazebo-with-point-clouds-and-rrt.html

Za pokretanje je potrebno:
1. MATLAB R2026a (ili noviji) s Robotics System Toolboxom, Computer Vision Toolboxom i ROS Toolboxom
2. Preuzeti i pokrenuti izvorni MathWorks primjer naredbom:
```matlab
   openExample('robotics/PickandPlaceWorkflowInGazeboUsingPointCloudProcessingExample')
```
3. Gazebo virtualnu mašinu (dostupnu putem istog MathWorks primjera)

## Datoteke u ovom repozitoriju

| Datoteka | Opis | Poglavlje u radu |

| `mjeriRRTParametre.m` | Mjeri utjecaj RRT parametara na planiranje | 5.1 |
| `mjeriTrajektorijuParametre.m` | Mjeri utjecaj parametra h na trajektoriju | 5.2 |
| `prikaziStvarniProfilTrajektorije.m` | Crta stvarni profil položaja/brzine/ubrzanja | 5.2 |
| `snimiPointCloudIRGB.m` | Snima RGB i sirovi point cloud iz scene | 4.1 |
| `prikaziICPPoravnanje.m` | Prikazuje ICP poravnanje prije/poslije | 4.2 |
| `usporediRedoslijedHvatanja.m` | Uspoređuje redoslijed hvatanja (najbliži prvi) | 5.3 |
| `usporediRMSEKriterij.m` | Uspoređuje redoslijed prema RMSE kriteriju | 5.3 |
| postavljanjeScene.m | Programski postavlja predmete za 4 testne scene | 5.4 |
| provjeriRobusnost.m | Provjerava detekciju, RMSE i vizualizira centroide | 5.4 |

## Kako pokrenuti

1. Spremi sve datoteke u isti folder kao izvorni MathWorks primjer
2. Pokreni pripremu (rosinit, coordinator, Build Environment, Detect Parts — vidi poglavlje 3-4 rada)
3. Pokreni željenu skriptu iz tablice iznad

## Reproduciranje eksperimenta iz poglavlja 5.4

```matlab
% ... rosinit, coordinator ...

postavljanjeScene('lezece')   % ili: 'boca_na_limenci', 'boca_ispred_limenke', 'limenka_ispred_boce'

exampleCommandBuildWorldROSGazeboScene(coordinator)
exampleCommandDetectPartsROSGazeboScene(coordinator)

provjeriRobusnost()
```

## Autor

Tea Bakić, Fakultet strojarstva i brodogradnje, Sveučilište u Zagrebu
