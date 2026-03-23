
%% This script aim to transform the raw log into a matrix that is readable from the EKF

% Bridge: CLOVES Log to EKF Input
% Load the map to convert the hex address into coordinates
map = importfile("DEPT_evb1000_map.csv");

% Load the log (put the name of the file that is inside the folder log)
rawLog = parse_cloves_log('logs/job_calibration.log');

% Create a structor for the EKF
% The EKF needs : [ID_anchor, distance, X_anchor, Y_anchor]
ekf_input = [];

    for i = 1:size(rawLog, 1)
        % 1. Estrai l'indirizzo come stringa semplice
        addrMatch = char(rawLog.respIDs(i)); 
    
        % 2. Prendi gli ultimi 5 caratteri (es. '19:15')
        % Usiamo min per evitare errori se la stringa fosse più corta di 5
        searchStr = addrMatch(max(1, end-4):end);
    
        % 3. Cerca nel database della mappa
        % contains() restituisce un vettore logico, find() ci da l'indice
        idx = find(contains(string(map.evb1000), searchStr));
    
        if ~isempty(idx)
            % Prendi solo il primo match se ce ne sono multipli
            targetIdx = idx(1);
        
            % Estrazione coordinate (gestendo i vari formati di stringa [x, y])
            coordStr = char(map.Coordinates(targetIdx));
            nodeCoord = split(regexprep(coordStr, '[\[\] ]', ''), ',');
        
            ax = str2double(nodeCoord{1});
            ay = str2double(nodeCoord{2});
        
            % Aggiungi alla matrice per l'EKF
            ekf_input = [ekf_input; map.NodeId(targetIdx), rawLog.distances(i), ax, ay];
        end
    end

preprocessed_data = ekf_input;
fprintf('Data are ready for the EKF: %d Valid values.\n', size(ekf_input, 1));