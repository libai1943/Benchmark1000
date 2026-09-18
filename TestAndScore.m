function results = TestAndScore(root_dir)
if nargin < 1 || isempty(root_dir)
    root_dir = fileparts(mfilename('fullpath'));
end
root_dir = char(root_dir);
if ~isfolder(root_dir)
    error('TestAndScore:MissingRoot', 'Directory does not exist: %s', root_dir);
end
n_cases = 1000;
folders = {'OBCA14', 'sdOBCA14', 'BOMP15', 'Lutz16', 'Fan18', 'Liu20', 'Triangle21', 'Fan22'};
tags = {'obca', 'sdobca', 'bomp', '16', '18', '20', '21', '22'};
model_names = {'basic OBCA [14]'; 'sd-OBCA [14]'; 'BOMP [15]'; 'Ref. [16]'; 'Ref. [18]'; 'Ref. [20]'; 'Ref. [21]'; 'Ref. [22]'};
mode_names = {'Static Initial Guess', 'Colliding Initial Guess', 'Completely Feasible Initial Guess'};
variable_names = {'Static_Loss_pct', 'Static_Success_pct', 'Static_Runtime_s', 'Colliding_Loss_pct', 'Colliding_Success_pct', 'Colliding_Runtime_s', 'Feasible_Loss_pct', 'Feasible_Success_pct', 'Feasible_Runtime_s'};
n_models = numel(folders);
n_modes = numel(mode_names);
input_files = cell(n_models, n_modes);
missing_files = {};
for ii = 1:n_models
    for jj = 1:n_modes
        input_files{ii, jj} = fullfile(root_dir, folders{ii}, sprintf('result_%s_mode_%d.mat', tags{ii}, jj - 1));
        if ~isfile(input_files{ii, jj})
            missing_files{end + 1, 1} = input_files{ii, jj};
        end
    end
end
if ~isempty(missing_files)
    error('TestAndScore:MissingResults', 'Complete all 24 result files before scoring. Missing files:\n%s', strjoin(missing_files, newline));
end
raw_data = cell(n_models, n_modes);
reference_time = inf(n_cases, 1);
reference_model = repmat({''}, n_cases, 1);
reference_mode = nan(n_cases, 1);
for ii = 1:n_models
    for jj = 1:n_modes
        data = ReadResult(input_files{ii, jj}, n_cases);
        raw_data{ii, jj} = data;
        valid = data(:, 1) == 1;
        better = valid & data(:, 3) < reference_time;
        reference_time(better) = data(better, 3);
        reference_model(better) = repmat(model_names(ii), nnz(better), 1);
        reference_mode(better) = jj - 1;
    end
end
no_reference = isinf(reference_time);
reference_time(no_reference) = NaN;
if any(no_reference)
    warning('TestAndScore:NoReference', '%d cases have no successful solution across all methods and modes. Their reference times are NaN.', nnz(no_reference));
end
loss_rate = nan(n_cases, n_models, n_modes);
summary_values = nan(n_models, 3 * n_modes);
success_count = zeros(n_models, n_modes);
for ii = 1:n_models
    for jj = 1:n_modes
        data = raw_data{ii, jj};
        valid = data(:, 1) == 1;
        success_count(ii, jj) = nnz(valid);
        col = 3 * (jj - 1);
        if any(valid)
            losses = 100 * (data(valid, 3) - reference_time(valid)) ./ reference_time(valid);
            loss_rate(valid, ii, jj) = losses;
            summary_values(ii, col + 1) = mean(losses);
        end
        summary_values(ii, col + 2) = 100 * nnz(valid) / n_cases;
        summary_values(ii, col + 3) = mean(data(:, 2));
    end
end
TableVIII = [table(model_names, 'VariableNames', {'Model'}), array2table(summary_values, 'VariableNames', variable_names)];
reference_table = table((1:n_cases)', reference_time, reference_model, reference_mode, 'VariableNames', {'CaseID', 'BestKnownTime_s', 'FirstSourceModel', 'FirstSourceMode'});
output_dir = fullfile(root_dir, 'Scores');
if ~isfolder(output_dir)
    [ok, message] = mkdir(output_dir);
    if ~ok
        error('TestAndScore:OutputDirectory', 'Cannot create output directory: %s', message);
    end
end
results = struct;
results.TableVIII = TableVIII;
results.SummaryValues = summary_values;
results.ReferenceTable = reference_table;
results.ReferenceTime = reference_time;
results.LossRate = loss_rate;
results.SuccessCount = success_count;
results.RawData = raw_data;
results.InputFiles = input_files;
results.ModelNames = model_names;
results.ModeNames = mode_names;
results.OutputDirectory = output_dir;
save(fullfile(output_dir, 'BenchmarkScores.mat'), 'results');
writetable(TableVIII, fullfile(output_dir, 'TableVIII.csv'));
writetable(reference_table, fullfile(output_dir, 'BestKnownSolutions.csv'));
WriteTableHTML(fullfile(output_dir, 'TableVIII.html'), model_names, mode_names, summary_values);
fprintf('\nTABLE VIII. SIMULATION RESULTS OF EIGHT MODELS UNDER THREE INITIAL GUESS CONDITIONS\n');
fprintf('%-18s | %-31s | %-31s | %-31s\n', 'Model', mode_names{1}, mode_names{2}, mode_names{3});
fprintf('%-18s | %10s %8s %10s | %10s %8s %10s | %10s %8s %10s\n', '', 'Loss (%)', 'Succ (%)', 'Time (s)', 'Loss (%)', 'Succ (%)', 'Time (s)', 'Loss (%)', 'Succ (%)', 'Time (s)');
for ii = 1:n_models
    fprintf('%-18s | %10.4f %8.1f %10.4f | %10.4f %8.1f %10.4f | %10.4f %8.1f %10.4f\n', model_names{ii}, summary_values(ii, :));
end
fprintf('\nBest-known reference available for %d / %d cases.\n', nnz(~no_reference), n_cases);
fprintf('Reports saved to: %s\n', output_dir);
end
function data = ReadResult(filename, n_cases)
info = whos('-file', filename);
if ~any(strcmp({info.name}, 'data_collection'))
    error('TestAndScore:MissingVariable', 'Missing data_collection: %s', filename);
end
loaded = load(filename, 'data_collection');
data = loaded.data_collection;
if ~isnumeric(data) || ~isreal(data) || ~isequal(size(data), [n_cases, 3])
    error('TestAndScore:InvalidShape', 'data_collection must be a real %d-by-3 numeric matrix: %s', n_cases, filename);
end
data = double(data);
if any(~ismember(data(:, 1), [0, 1]))
    error('TestAndScore:InvalidFlag', 'Column 1 must contain only 0 or 1: %s', filename);
end
bad_runtime = isnan(data(:, 2)) | isinf(data(:, 2)) | data(:, 2) <= 0;
if any(bad_runtime)
    error('TestAndScore:InvalidRuntime', 'Case %d has a nonfinite or nonpositive runtime. Check for an unfinished batch: %s', find(bad_runtime, 1), filename);
end
valid = data(:, 1) == 1;
bad_cost = valid & (isnan(data(:, 3)) | isinf(data(:, 3)) | data(:, 3) <= 0);
if any(bad_cost)
    error('TestAndScore:InvalidCost', 'Successful case %d must have a finite positive completion time: %s', find(bad_cost, 1), filename);
end
if any(strcmp({info.name}, 'case_id'))
    saved = load(filename, 'case_id');
    if ~isnumeric(saved.case_id) || ~isreal(saved.case_id) || ~isscalar(saved.case_id) || saved.case_id ~= n_cases
        error('TestAndScore:IncompleteBatch', 'Saved case_id does not equal %d. Finish the batch before scoring: %s', n_cases, filename);
    end
end
end
function WriteTableHTML(filename, model_names, mode_names, values)
[fid, message] = fopen(filename, 'w', 'n', 'UTF-8');
if fid < 0
    error('TestAndScore:OutputFile', 'Cannot write %s: %s', filename, message);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', '<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8"><title>Benchmark1000 - Table VIII</title>');
fprintf(fid, '%s\n', '<style>body{font-family:"Times New Roman",serif;margin:28px}h1{font-size:18px;text-align:center;font-weight:normal}table{border-collapse:collapse;width:100%;font-size:14px}th,td{border:1px solid #555;padding:7px 9px}th{background:#86d7ed;text-align:center}td{text-align:right;white-space:nowrap}td:first-child{text-align:left}.scroll{overflow-x:auto}p{font-size:14px;line-height:1.5}</style></head><body>');
fprintf(fid, '%s\n', '<h1>TABLE VIII. SIMULATION RESULTS OF EIGHT MODELS UNDER THREE INITIAL GUESS CONDITIONS</h1><div class="scroll"><table><thead><tr><th rowspan="2">Model<br>Name</th>');
for jj = 1:numel(mode_names)
    fprintf(fid, '<th colspan="3">%s</th>', mode_names{jj});
end
fprintf(fid, '%s\n', '</tr><tr>');
for jj = 1:numel(mode_names)
    fprintf(fid, '%s', '<th>Average Optimality<br>Loss Rate (%)</th><th>Success<br>Rate (%)</th><th>Average<br>Runtime (s)</th>');
end
fprintf(fid, '%s\n', '</tr></thead><tbody>');
for ii = 1:numel(model_names)
    fprintf(fid, '<tr><td>%s</td>', model_names{ii});
    for jj = 1:size(values, 2)
        if isnan(values(ii, jj))
            fprintf(fid, '<td>--</td>');
        elseif mod(jj, 3) == 2
            fprintf(fid, '<td>%.1f</td>', values(ii, jj));
        else
            fprintf(fid, '<td>%.4f</td>', values(ii, jj));
        end
    end
    fprintf(fid, '%s\n', '</tr>');
end
fprintf(fid, '%s\n', '</tbody></table></div>');
fprintf(fid, '%s\n', '<p>The reference completion time for each case is the minimum among successful runs across all eight methods and all three initialization modes. It is a best-known value, with no guarantee of global optimality.</p>');
fprintf(fid, '%s\n', '<p>Optimality loss is 100 times the excess completion time divided by the case reference, averaged over successful runs only. Success rate and average runtime use all 1,000 cases, including failures. A dash indicates that no successful run is available for computing the average loss.</p>');
fprintf(fid, '%s\n', '<p>Success flags, runtimes, and completion times are read from columns 1, 2, and 3 of data_collection. Runtime is the elapsed time recorded by RunBatch.m. All table entries are calculated from these input data; numerical formatting affects display only.</p></body></html>');
end
