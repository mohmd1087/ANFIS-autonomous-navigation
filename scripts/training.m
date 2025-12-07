% Load data
load('../data/training_data1.mat'); D1 = training_data;
load('../data/training_data2.mat'); D2 = training_data;
load('../data/training_data3.mat'); D3 = training_data;
load('../data/training_data4.mat'); D4 = training_data;

training_data = [D1; D2; D3; D4];


% Make sure data is double (anfis/neuroFuzzyDesigner requires double)
training_data = double(training_data);

% Split into inputs and outputs
X       = training_data(:,1:4);        % [S1 S2 S3 heading_error]
Y_left  = training_data(:,5);          % left motor speeds
Y_right = training_data(:,6);          % right motor speeds

% Shuffle the samples (same random order for all)
idx     = randperm(size(X,1));
X       = X(idx,:);
Y_left  = Y_left(idx,:);
Y_right = Y_right(idx,:);

% Build ANFIS training matrices
trainingDataLeft  = [X Y_left];        % for left wheel ANFIS
trainingDataRight = [X Y_right];       % for right wheel ANFIS
