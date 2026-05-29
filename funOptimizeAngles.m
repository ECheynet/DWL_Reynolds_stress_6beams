function [Rbest,thetaBest,phiBest] = funOptimizeAngles(nBeams)

% x = [theta1 phi1 theta2 phi2 ... theta6 phi6]

nVars = 2*nBeams;
lb = zeros(1,nVars);
ub = zeros(1,nVars);

for ii = 1:nBeams
    lb(2*ii-1) = 45;   % theta lower bound
    ub(2*ii-1) = 90;   % theta upper bound

    lb(2*ii) = 0;      % phi lower bound
    ub(2*ii) = 360;    % phi upper bound
end

% All variables are integer-valued, i.e. 1-degree grid
IntCon = 1:nVars;

objFun = @(x) objectiveCondR(x);

options = optimoptions('ga', ...
    'PopulationSize', 400, ...
    'MaxGenerations', 500, ...
    'FunctionTolerance', 1e-8, ...
    'Display', 'off', ...
    'UseParallel', false);

[xBest, fBest] = ga( ...
    objFun, ...
    nVars, ...
    [], [], [], [], ...
    lb, ub, ...
    [], ...
    IntCon, ...
    options);

anglesBest = reshape(xBest,2,nBeams).';

thetaBest = anglesBest(:,1);
phiBest   = mod(anglesBest(:,2),360);
phiBest = phiBest-min(phiBest);
phiBest(thetaBest==90)=0;

Rbest = buildR(thetaBest, phiBest);

fprintf('\nBest angles:\n');
fprintf('Beam    theta [deg]    phi [deg]\n');
for ii = 1:nBeams
    fprintf('%3d     %10.0f    %9.0f\n', ii, thetaBest(ii), phiBest(ii));
end

fprintf('\nCondition number: %.6f\n', cond(Rbest));
fprintf('Rank: %d\n', rank(Rbest));
% 
% fprintf('\nSingular values:\n');
% disp(svd(Rbest).');

% fprintf('\nR matrix:\n');
% disp(Rbest);

    function f = objectiveCondR(x)
        x = round(x(:).');
        angles = reshape(x,2,nBeams).';
        thetaDeg = angles(:,1);
        phiDeg   = mod(angles(:,2),360);
        R = buildR(thetaDeg, phiDeg);
        s = svd(R);
        if s(end) < 1e-12
            f = 1e12;
            return
        end
        f = log(s(1)/s(end));
    end


    function R = buildR(thetaDeg, phiDeg)

        theta = deg2rad(thetaDeg(:));
        phi   = deg2rad(phiDeg(:));

        a = cos(theta).*sin(phi);
        b = cos(theta).*cos(phi);
        c = sin(theta);

        R = [ ...
            a.^2, ...
            b.^2, ...
            c.^2, ...
            2*a.*b, ...
            2*a.*c, ...
            2*b.*c ...
            ];
    end

end