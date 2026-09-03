% MATLAB code for optimizing a six-beam Doppler wind lidar scanning
% geometry used to retrieve Reynolds-stress statistics from along-beam
% velocity variances.
%
% Script inspired by:
% Sathe, A., Mann, J., Vasiljevic, N., & Lea, G. (2015).
% A six-beam method to measure turbulence statistics using ground-based
% wind lidars. Atmospheric Measurement Techniques, 8(2), 729-740.
clearvars; close all; clc;
addpath('./functions')
%% Beam geometries
nBeams = 6;
rng(10)
[Rbest, thetaBest, phiBest] = funOptimizeAngles(nBeams);

% theta is elevation; theta = 90 degrees is vertical.
% phi is azimuth clockwise from north, as defined in buildR.
thetaRef = [45 45 45 45 45 90];
phiRef   = [0 72 144 216 288 288];
Rref = buildR(thetaRef, phiRef);

fprintf('\nCondition number of optimized R: %.6f\n', cond(Rbest));
fprintf('Condition number of Sathe et al. (2015) R: %.6f\n', ...
    cond(Rref));

plotLidarBeams(thetaBest, phiBest, ...
    'Optimized 6-beam lidar geometry');
plotLidarBeams(thetaRef, phiRef, ...
    'Sathe et al. (2015) reference geometry');

%% Reynolds stresses in fixed east-north-up coordinates

% Column order required by buildR:
% [sigma_x^2; sigma_y^2; sigma_z^2; sigma_xy; sigma_xz; sigma_yz]
xFixedTrue = [
    3.6111    % sigma_x^2 [m^2/s^2]
    2.9489    % sigma_y^2 [m^2/s^2]
    1.0000    % sigma_z^2 [m^2/s^2]
    0.6402    % sigma_xy  [m^2/s^2]
    0.4774    % sigma_xz  [m^2/s^2]
    0.5065    % sigma_yz  [m^2/s^2]
];

%% Rotate from fixed coordinates to wind-aligned coordinates

% Meteorological direction: degrees clockwise from north, indicating the
% direction FROM which the wind comes.
windDir = 240;
s = sind(windDir);
c = cosd(windDir);

% [x; y; z] -> [u; v; w], where u is along wind and v is cross wind.
T = [-s, -c, 0; c, -s, 0; 0, 0, 1];

CfixedTrue = vectorToTensor(xFixedTrue);
CwindTrue = T * CfixedTrue * T.';
xWindTrue = tensorToVector(CwindTrue);

%% Generate radial variances and retrieve stresses

% buildR acts on fixed-coordinate stresses. The retrieved tensor is then
% rotated into wind coordinates inside retrieveStresses.
[sigma2Vr, xFixedRetrieved, xWindRetrieved] = ...
    retrieveStresses(Rbest, xFixedTrue, T);
[~, ~, xWindRetrievedRef] = ...
    retrieveStresses(Rref, xFixedTrue, T);

printBeamVariances(sigma2Vr, 'optimized geometry');

namesFixed = {'sigma_x^2'; 'sigma_y^2'; 'sigma_z^2'; ...
              'sigma_xy';  'sigma_xz';  'sigma_yz'};
namesWind  = {'sigma_u^2'; 'sigma_v^2'; 'sigma_w^2'; ...
              'sigma_uv';  'sigma_uw';  'sigma_vw'};

printComparison('Retrieved stresses in fixed lidar coordinates', ...
    namesFixed, xFixedTrue, xFixedRetrieved);
printComparison('Retrieved stresses in wind-aligned coordinates', ...
    namesWind, xWindTrue, xWindRetrieved);
printComparison('Wind-aligned stresses using Sathe et al. geometry', ...
    namesWind, xWindTrue, xWindRetrievedRef);











