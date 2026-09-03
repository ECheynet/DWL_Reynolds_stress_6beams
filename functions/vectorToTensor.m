function C = vectorToTensor(x)
%VECTORTOTENSOR Convert the six unique stresses to a symmetric tensor.

C = [
    x(1), x(4), x(5)
    x(4), x(2), x(6)
    x(5), x(6), x(3)
];
