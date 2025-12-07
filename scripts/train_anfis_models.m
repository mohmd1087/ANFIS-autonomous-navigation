function train_anfis_models()
    clc; clear; close all;

    % === 1. Load prepared data ===
    S = load('all_training_data.mat');
    trainData = double(S.trainData);   % ensure double
    valData   = double(S.valData);

    % Inputs are S1 S2 S3 heading_error
    inTrain = trainData(:,1:4);
    inVal   = valData(:,1:4);

    % === 2. Training options ===
    % [Epochs ErrorGoal InitialStep StepDecrease StepIncrease]
    opt = [50 0 0.01 0.9 1.1];

    % Common settings
    mf_n  = [3 3 3 3];   % 3 MFs per input
    limit = pi/2;        % heading range

    % =====================================================================
    %                           LEFT WHEEL
    % =====================================================================
    outTrain_left = trainData(:,5);     % vL
    outVal_left   = valData(:,5);

    train_left = [inTrain outTrain_left];   % (N x 5) double
    chk_left   = [inVal   outVal_left];

    fprintf('\n=== Building initial FIS for LEFT wheel ===\n');
    initFis_left = genfis1(train_left, mf_n, 'gaussmf'); % warning is OK

    % Set input ranges (physical ranges)
    for i = 1:3
        initFis_left.input(i).range = [0 1];    % 0..1 m sonar
    end
    initFis_left.input(4).range = [-limit limit];  % heading error
    initFis_left.output(1).range = [-2 2];         % wheel speed

    fprintf('=== Training LEFT wheel ANFIS ===\n');
    [fis_left, trainErrL, ~, chkFis_left, chkErrL] = ...
        anfis(train_left, initFis_left, opt, [], chk_left);

    fprintf('Final training error (left):     %g\n', trainErrL(end));
    fprintf('Final checking/val error (left): %g\n', chkErrL(end));

    writefis(chkFis_left, 'fis_left');
    fprintf('Saved fis_left.fis\n');

    % =====================================================================
    %                           RIGHT WHEEL
    % =====================================================================
    outTrain_right = trainData(:,6);    % vR
    outVal_right   = valData(:,6);

    train_right = [inTrain outTrain_right];
    chk_right   = [inVal   outVal_right];

    fprintf('\n=== Building initial FIS for RIGHT wheel ===\n');
    initFis_right = genfis1(train_right, mf_n, 'gaussmf');

    for i = 1:3
        initFis_right.input(i).range = [0 1];
    end
    initFis_right.input(4).range = [-limit limit];
    initFis_right.output(1).range = [-2 2];

    fprintf('=== Training RIGHT wheel ANFIS ===\n');
    [fis_right, trainErrR, ~, chkFis_right, chkErrR] = ...
        anfis(train_right, initFis_right, opt, [], chk_right);

    fprintf('Final training error (right):     %g\n', trainErrR(end));
    fprintf('Final checking/val error (right): %g\n', chkErrR(end));

    writefis(chkFis_right, 'fis_right');
    fprintf('Saved fis_right.fis\n');

    fprintf('\nTraining finished.\n');
end
