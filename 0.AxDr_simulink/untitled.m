Input = importdata('57_rev2.0_17.csv');
% Input = importdata('运放供电4V分压一半.csv');
% Input = importdata('test.csv');
% FULL  SCALE

FR = 65535;

% data = Input.data(: , 2);
% dataMean = mean(data);
% sigma = std(data);
% sinSign = log2(FR/(6*sigma));
% 
% data = Input.data(: , 3);
% sigma = std(data);
% cosSign = log2(FR/(6*sigma));
% 
% data = Input.data(: , 4);
% sigma = std(data);
% u0Res = log2(FR/(6*sigma));
% 
% data = Input.data(: , 5);
% sigma = std(data);
% v0Res = log2(FR/(6*sigma));   
% 
% data = Input.data(:, 6);
% sigma = std(data);
% radRes = log2(2*pi/(6*sigma));

data = Input.data(: , 2);
dataMean = mean(data);
sigma = std(data);
Sign = log2(FR/(6*sigma));