function x = tensorToVector(C)
%TENSORTOVECTOR Convert a symmetric tensor to the buildR column order.

% Remove any roundoff-level asymmetry introduced by matrix operations.
C = 0.5 * (C + C.');

x = [
    C(1,1)
    C(2,2)
    C(3,3)
    C(1,2)
    C(1,3)
    C(2,3)
];

end
