function [sigma2Vr, xFixedRetrieved, xWindRetrieved] = retrieveStresses( ...
        R, xFixedTrue, T)
%RETRIEVESTRESSES Simulate radial variances, invert them, and rotate.

sigma2Vr = R * xFixedTrue;
xFixedRetrieved = R \ sigma2Vr;
CwindRetrieved = T * vectorToTensor(xFixedRetrieved) * T.';
xWindRetrieved = tensorToVector(CwindRetrieved);

end