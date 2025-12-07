function prepare_training_data()
    clc; clear; close all;

    % === 1. Load all your raw training_data*.mat files ===
    allData = [];

    for k = 1:23
        S = load(sprintf('../data/training_data%d.mat', k));  % training_data1..4.mat
        % Make sure each chunk is double
        thisData = double(S.training_data);          % [S1 S2 S3 heading vL vR]
        allData  = [allData; thisData];
    end

    % Convert whole matrix to double (just in case)
    allData = double(allData);

    fprintf('Loaded %d samples\n', size(allData,1));

    % === 2. Apply same heading_error clamp used in Autonomous_Run ===
    limit = pi/2;   % you use ±pi/2 in computeHeadingError
    allData(:,4) = max(min(allData(:,4),  limit), -limit);

    % === 3. (Optional) shuffle data ===
    idx = randperm(size(allData,1));
    allData = allData(idx,:);

    % === 4. Split into train / validation (80% / 20%) ===
    N      = size(allData,1);
    Ntrain = round(0.8 * N);

    trainData = allData(1:Ntrain, :);
    valData   = allData(Ntrain+1:end, :);

    save('all_training_data.mat','trainData','valData');

    fprintf('Saved all_training_data.mat with %d train and %d val samples.\n',...
             size(trainData,1), size(valData,1));
end
