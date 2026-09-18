function postavljanjeScene(scenarij)
% POSTAVLJANJESCENE  Programski postavlja bocu i limenku na zadanu
% poziciju/orijentaciju u Gazebu, koristeci ROS servis
% /gazebo/set_model_state.
%
% Koristi se za eksperimente robusnosti opisane u poglavlju 5.4
% zavrsnog rada "Autonomno pretrazivanje okoline primjenom 3D vizijskog
% sustava i robotske ruke".
%
% ULAZ:
%   scenarij - string, jedan od:
%       'standardno'            - boca i limenka uspravne, izvorne pozicije
%       'lezece'                - oba predmeta polozena na bok (Scena 1)
%       'boca_na_limenci'       - boca postavljena na vrh limenke (Scena 2)
%       'boca_ispred_limenke'   - boca blize robotu, u liniji (Scena 3)
%       'limenka_ispred_boce'   - limenka blize robotu, u liniji (Scena 4)
%
% PRETPOSTAVKA: aktivna ROS veza s Gazebo simulacijom (rosinit vec
% pozvan), te da su modeli u sceni imenovani 'Red Bottle' i 'Green Can'
% (provjeri stvarna imena u svojoj sceni naredbom:
%   resp = call(rossvcclient('/gazebo/get_world_properties'));
%   disp(resp.ModelNames)
%
% PRIMJER:
%   postavljanjeScene('lezece')

    setStateClient = rossvcclient('/gazebo/set_model_state');

    switch scenarij
        case 'standardno'
            bocaPos   = [0.3583, -0.0760, 0.6339];
            bocaQuat  = [1 0 0 0];
            limenkaPos  = [0.2911, -0.2032, 0.5856];
            limenkaQuat = [1 0 0 0];

        case 'lezece'
            bocaPos   = [0.3583, -0.0760, 0.6339];
            bocaQuat  = eul2quat([0 0 pi/2], 'ZYX');
            limenkaPos  = [0.2911, -0.2032, 0.5856];
            limenkaQuat = eul2quat([0 0 pi/2], 'ZYX');

        case 'boca_na_limenci'
            limenkaPos  = [0.2911, -0.2032, 0.5856];
            limenkaQuat = [1 0 0 0];
            bocaPos   = [0.2911, -0.2032, 0.5856 + 0.15];
            bocaQuat  = [1 0 0 0];

        case 'boca_ispred_limenke'
            bocaPos   = [0.28, -0.14, 0.6339];
            bocaQuat  = [1 0 0 0];
            limenkaPos  = [0.38, -0.14, 0.5856];
            limenkaQuat = [1 0 0 0];

        case 'limenka_ispred_boce'
            limenkaPos  = [0.28, -0.14, 0.5856];
            limenkaQuat = [1 0 0 0];
            bocaPos   = [0.38, -0.14, 0.6339];
            bocaQuat  = [1 0 0 0];

        otherwise
            error('postavljanjeScene:nepoznatScenarij', ...
                'Nepoznat scenarij "%s". Dostupno: standardno, lezece, boca_na_limenci, boca_ispred_limenke, limenka_ispred_boce.', scenarij);
    end

    postaviModel(setStateClient, 'Red Bottle', bocaPos, bocaQuat);
    postaviModel(setStateClient, 'Green Can', limenkaPos, limenkaQuat);

    pause(3);  % pricekaj da se fizika (eventualni pad/slijeganje) smiri
end


function postaviModel(client, ime, pozicija, quat)
% POSTAVIMODEL  Pomocna funkcija - postavlja jedan Gazebo model na
% zadanu poziciju (x,y,z) i orijentaciju (quaternion [w x y z]).

    req = rosmessage(client);
    req.ModelState.ModelName = ime;
    req.ModelState.Pose.Position.X = pozicija(1);
    req.ModelState.Pose.Position.Y = pozicija(2);
    req.ModelState.Pose.Position.Z = pozicija(3);
    req.ModelState.Pose.Orientation.W = quat(1);
    req.ModelState.Pose.Orientation.X = quat(2);
    req.ModelState.Pose.Orientation.Y = quat(3);
    req.ModelState.Pose.Orientation.Z = quat(4);
    call(client, req);
end
