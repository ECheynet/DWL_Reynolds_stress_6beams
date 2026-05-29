clearvars; close all; clc;

nBeams = 6;

% Make sure funOptimizeAngles also returns thetaBest and phiBest

rng(10)
[Rbest, thetaBest, phiBest] = funOptimizeAngles(nBeams);

fprintf('\nCondition number of optimized R : %.6f\n', cond(Rbest));

% Plot optimized lidar beams
plotLidarBeams(thetaBest, phiBest, 'Optimized 6-beam lidar geometry');


%% Comparison with Sathe et al. (2015)
thetaRef = [45 45 45 45 45 90];
phiRef   = [0 72 144 216 288 288];
Rref = buildR(thetaRef, phiRef);
fprintf('\nCondition number in Sathe et al. (2015) : %.6f\n', cond(Rref));
% Plot reference lidar beams
plotLidarBeams(thetaRef, phiRef, 'Sathe et al. (2015) reference geometry');



%% Assume the following Reynolds stresses

sigma2U = 4;                 % m^2/s^2
sigma2V = sigma2U*0.8.^2;
sigma2W = sigma2U*0.5.^2;
sigmaUW = sigma2U/6;
sigmaVW = 0.3*sigmaUW;
sigmaUV = 0.05*sigmaUW;

% Reynolds-stress vector
% Order must match the columns of R:
% [sigma_u^2; sigma_v^2; sigma_w^2; sigma_uv; sigma_uw; sigma_vw]
xTrue = [
    sigma2U
    sigma2V
    sigma2W
    sigmaUV
    sigmaUW
    sigmaVW
];

%% Get the along-beam variances

sigma2Vr = Rbest*xTrue;

fprintf('\nAlong-beam variances from optimized geometry:\n');
fprintf('Beam    sigma_vr^2 [m^2/s^2]\n');
for i = 1:nBeams
    fprintf('%3d     %12.2f\n', i, sigma2Vr(i));
end

%% Retrieve the Reynolds stresses

% Prefer backslash over inv(Rbest)*sigma2Vr
xRetrieved = Rbest\sigma2Vr;
fprintf('\nRetrieved Reynolds stresses:\n');
fprintf('%12s    %12s    %12s    %12s\n', ...
    'Quantity', 'True', 'Retrieved', 'Error');
names = {
    'sigma_u^2'
    'sigma_v^2'
    'sigma_w^2'
    'sigma_uv'
    'sigma_uw'
    'sigma_vw'
};

for i = 1:numel(xTrue)
    fprintf('%12s    %12.2f    %12.2f    %12.3e\n', ...
        names{i}, xTrue(i), xRetrieved(i), xRetrieved(i)-xTrue(i));
end

%% Compare with Sathe et al. (2015) geometry

thetaRef = [45 45 45 45 45 90];
phiRef   = [0 72 144 216 288 288];

Rref = buildR(thetaRef, phiRef);

fprintf('\nCondition number in Sathe et al. (2015): %.6f\n', cond(Rref));

sigma2VrRef = Rref*xTrue;
xRetrievedRef = Rref\sigma2VrRef;

fprintf('\nRetrieved Reynolds stresses using Sathe et al. geometry:\n');
fprintf('%12s    %12s    %12s    %12s\n', ...
    'Quantity', 'True', 'Retrieved', 'Error');

for i = 1:numel(xTrue)
    fprintf('%12s    %12.6f    %12.6f    %12.3e\n', ...
        names{i}, xTrue(i), xRetrievedRef(i), xRetrievedRef(i)-xTrue(i));
end