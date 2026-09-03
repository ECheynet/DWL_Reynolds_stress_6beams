function printBeamVariances(sigma2Vr, geometryName)
%PRINTBEAMVARIANCES Print one radial-variance value per beam.

fprintf('\nAlong-beam variances from %s:\n', geometryName);
fprintf('Beam    sigma_vr^2 [m^2/s^2]\n');
for i = 1:numel(sigma2Vr)
    fprintf('%3d     %12.6f\n', i, sigma2Vr(i));
end

end