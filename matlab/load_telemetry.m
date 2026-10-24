function tel = load_telemetry(file)
%LOAD_TELEMETRY Read a log of the car's telemetry (the "T 10" command).
%   Works with a picocom --logfile and with a phone Bluetooth terminal log.
%   Only data rows are kept:  ms,cnt,dist_cm,spd_cms,sp_cms,mot_pct,ref_cm,sonar_cm
%   Command replies ("OK open-loop"), the header line and timestamps that a
%   phone app adds in front of each line are skipped.

txt   = fileread(file);
lines = regexp(txt, '\r?\n|\r', 'split');
num   = '-?\d+(\.\d+)?';
row   = [num repmat([',' num], 1, 7)];   % 8 comma-separated numbers

rows = zeros(numel(lines), 8);
n = 0;
for i = 1:numel(lines)
    m = regexp(lines{i}, row, 'match', 'once');
    if ~isempty(m)
        n = n + 1;
        rows(n, :) = str2double(strsplit(m, ','));
    end
end
rows = rows(1:n, :);
if n == 0
    error('load_telemetry:empty', 'No telemetry rows in %s. Was telemetry on (T 10)?', file);
end

% HAL_GetTick() restarts at 0 after a reset: keep only the part after the last reset
k = find(diff(rows(:, 1)) < 0, 1, 'last');
if ~isempty(k), rows = rows(k+1:end, :); end

% the same row can arrive twice when USB and Bluetooth are both logged
[~, ia] = unique(rows(:, 1));
rows = rows(ia, :);

tel = array2table(rows, 'VariableNames', ...
    {'ms', 'cnt', 'dist_cm', 'spd_cms', 'sp_cms', 'mot_pct', 'ref_cm', 'sonar_cm'});
tel.t = (tel.ms - tel.ms(1)) / 1000;     % time in s from the first row
end
