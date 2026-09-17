close all; clc; clear all;
global params_
data_collection = zeros(1000, 3);

for case_id = 1 : 1000
    InitializeParams();
    LoadCaseBatch(case_id);
    [x, y, theta, v, a, phi, w, tf] = GenerateInitialGuess(0);
    WriteBasicParameterFile();
    WriteObs();
    WriteInitialGuess(x, y, theta, v, a, phi, w, tf);

    tic; SolveNLP(); timer = toc;
    is_valid = IsCurSolValid();

    data_collection(case_id, 1) = is_valid;
    data_collection(case_id, 2) = timer;
    data_collection(case_id, 3) = params_.opti.terminal_time * is_valid;
end
save result_22_mode_0


close all; clc; clear all;
global params_
data_collection = zeros(1000, 3);

for case_id = 1 : 1000
    InitializeParams();
    LoadCaseBatch(case_id);
    [x, y, theta, v, a, phi, w, tf] = GenerateInitialGuess(1);
    WriteBasicParameterFile();
    WriteObs();
    WriteInitialGuess(x, y, theta, v, a, phi, w, tf);

    tic; SolveNLP(); timer = toc;
    is_valid = IsCurSolValid();

    data_collection(case_id, 1) = is_valid;
    data_collection(case_id, 2) = timer;
    data_collection(case_id, 3) = params_.opti.terminal_time * is_valid;
end
save result_22_mode_1



close all; clc; clear all;
global params_
data_collection = zeros(1000, 3);

for case_id = 1 : 1000
    InitializeParams();
    LoadCaseBatch(case_id);
    [x, y, theta, v, a, phi, w, tf] = GenerateInitialGuess(2);
    WriteBasicParameterFile();
    WriteObs();
    WriteInitialGuess(x, y, theta, v, a, phi, w, tf);

    tic; SolveNLP(); timer = toc;
    is_valid = IsCurSolValid();

    data_collection(case_id, 1) = is_valid;
    data_collection(case_id, 2) = timer;
    data_collection(case_id, 3) = params_.opti.terminal_time * is_valid;
end
save result_22_mode_2