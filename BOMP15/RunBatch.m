close all; clc; clear all;
global params_
data_collection = zeros(1000, 3);

for case_id = 1 : 1000
    InitializeParams();
    LoadCaseBatch(case_id);
    [x, y, theta, v, a, phi, w, tf] = GenerateInitialGuess(0);
    params_.solver.epsilon = 1.0;
    tf_history = +Inf;
    WriteInitialGuess(x, y, theta, v, a, phi, w, tf);
    WriteObs();
    iter = 0;
    tic;
    while (iter < 8)
        iter = iter + 1;
        params_.solver.epsilon = params_.solver.epsilon * 0.1;
        WriteBasicParameterFile();
        SolveNLP();
        if (abs(params_.opti.terminal_time - tf_history) <= 0.001)
            break;
        end
        tf_history = params_.opti.terminal_time;
        WriteNewInitialGuess();
    end
    timer = toc;

    is_valid = IsCurSolValid();

    data_collection(case_id, 1) = is_valid;
    data_collection(case_id, 2) = timer;
    data_collection(case_id, 3) = params_.opti.terminal_time * is_valid;
end
save result_bomp_mode_0

close all; clc; clear all;
global params_
data_collection = zeros(1000, 3);

for case_id = 1 : 1000
    InitializeParams();
    LoadCaseBatch(case_id);
    [x, y, theta, v, a, phi, w, tf] = GenerateInitialGuess(1);
    params_.solver.epsilon = 1.0;
    tf_history = +Inf;
    WriteInitialGuess(x, y, theta, v, a, phi, w, tf);
    WriteObs();
    iter = 0;
    tic;
    while (iter < 8)
        iter = iter + 1;
        params_.solver.epsilon = params_.solver.epsilon * 0.1;
        WriteBasicParameterFile();
        SolveNLP();
        if (abs(params_.opti.terminal_time - tf_history) <= 0.001)
            break;
        end
        tf_history = params_.opti.terminal_time;
        WriteNewInitialGuess();
    end
    timer = toc;

    is_valid = IsCurSolValid();

    data_collection(case_id, 1) = is_valid;
    data_collection(case_id, 2) = timer;
    data_collection(case_id, 3) = params_.opti.terminal_time * is_valid;
end
save result_bomp_mode_1

close all; clc; clear all;
global params_
data_collection = zeros(1000, 3);

for case_id = 1 : 1000
    InitializeParams();
    LoadCaseBatch(case_id);
    [x, y, theta, v, a, phi, w, tf] = GenerateInitialGuess(2);
    params_.solver.epsilon = 1.0;
    tf_history = +Inf;
    WriteInitialGuess(x, y, theta, v, a, phi, w, tf);
    WriteObs();
    iter = 0;
    tic;
    while (iter < 8)
        iter = iter + 1;
        params_.solver.epsilon = params_.solver.epsilon * 0.1;
        WriteBasicParameterFile();
        SolveNLP();
        if (abs(params_.opti.terminal_time - tf_history) <= 0.001)
            break;
        end
        tf_history = params_.opti.terminal_time;
        WriteNewInitialGuess();
    end
    timer = toc;

    is_valid = IsCurSolValid();

    data_collection(case_id, 1) = is_valid;
    data_collection(case_id, 2) = timer;
    data_collection(case_id, 3) = params_.opti.terminal_time * is_valid;
end
save result_bomp_mode_2