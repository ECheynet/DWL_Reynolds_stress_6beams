# Six-Beam Wind Lidar Geometry Optimization

MATLAB code for optimizing a six-beam Doppler wind lidar scanning geometry and retrieving Reynolds-stress statistics from along-beam velocity variances.

The implementation distinguishes between two coordinate systems:

- **Fixed lidar coordinates** `(x,y,z)`: east, north, and vertical.
- **Wind-aligned coordinates** `(u,v,w)`: along wind, cross wind, and vertical.

The lidar geometry matrix acts on Reynolds stresses in the fixed coordinate system. After retrieval, the stress tensor is rotated into wind-aligned coordinates.

## Measurement model

The six-beam geometry matrix `R` relates the along-beam velocity variances to the six independent components of the Reynolds-stress tensor:

```text
sigma2Vr = R * xFixed
```

where

```text
xFixed = [sigma_x^2;
          sigma_y^2;
          sigma_z^2;
          sigma_xy;
          sigma_xz;
          sigma_yz]
```

and `sigma2Vr` contains one radial-velocity variance per beam.

The fixed-coordinate stresses are retrieved with MATLAB's backslash operator:

```matlab
xFixedRetrieved = R \ sigma2Vr;
```

The backslash operator is preferred over explicitly calculating `inv(R)`.

## Beam convention

For beam `i`, the elevation angle is `theta_i` and the azimuth angle is `phi_i`.

- `theta_i = 0` degrees is horizontal.
- `theta_i = 90` degrees is vertical.
- `phi_i` is measured clockwise from north.
- Positive radial velocity follows the beam unit-vector direction.

The line-of-sight velocity is

```text
v_r,i = x*cos(theta_i)*sin(phi_i)
      + y*cos(theta_i)*cos(phi_i)
      + z*sin(theta_i)
```

The corresponding beam unit vector is

```text
e_i = [cos(theta_i)*sin(phi_i),
       cos(theta_i)*cos(phi_i),
       sin(theta_i)]
```

Define the direction cosines

```text
a_i = cos(theta_i)*sin(phi_i)
b_i = cos(theta_i)*cos(phi_i)
c_i = sin(theta_i)
```

Then row `i` of `R` is

```text
R_i = [a_i^2,
       b_i^2,
       c_i^2,
       2*a_i*b_i,
       2*a_i*c_i,
       2*b_i*c_i]
```

This ordering must match the ordering of `xFixed`.

## Wind-direction transformation

Let `windDir` be the meteorological wind direction in degrees clockwise from north, indicating the direction **from** which the wind comes. The transformation from fixed east-north-up coordinates to along-wind, cross-wind, and vertical coordinates is

```matlab
s = sind(windDir);
c = cosd(windDir);

T = [-s, -c, 0;
      c, -s, 0;
      0,  0, 1];
```

If the velocity fluctuations satisfy

```text
qWind = T * qFixed
```

their covariance tensors satisfy

```text
Cwind = T * Cfixed * T.'
```

This follows directly from the covariance definition:

```text
Cwind = <qWind*qWind.'>
      = <(T*qFixed)*(T*qFixed).'>
      = T*<qFixed*qFixed.'>*T.'
      = T*Cfixed*T.'
```

Both velocity factors in the covariance must be rotated, which is why `T` appears on the left and `T.'` appears on the right.

Because `T` is orthogonal, the inverse transformation is

```matlab
Cfixed = T.' * Cwind * T;
```

## Fixed and wind-aligned stress vectors

The fixed-coordinate tensor is assembled as

```matlab
Cfixed = [sigma2X, sigmaXY, sigmaXZ;
          sigmaXY, sigma2Y, sigmaYZ;
          sigmaXZ, sigmaYZ, sigma2Z];
```

After applying `Cwind = T*Cfixed*T.'`, the wind-aligned vector is

```text
xWind = [sigma_u^2;
         sigma_v^2;
         sigma_w^2;
         sigma_uv;
         sigma_uw;
         sigma_vw]
```

The fixed and wind-aligned vectors describe the same physical Reynolds-stress tensor in different coordinate systems. The unaligned values should not be selected independently; they are related by the wind-direction rotation.

## Repository contents

| File | Description |
| --- | --- |
| `Example_optimize_angles.m` | Main example: optimizes and plots the geometry, compares it with the reference geometry, generates synthetic radial variances, retrieves fixed-coordinate stresses, and rotates them into wind coordinates. |
| `funOptimizeAngles.m` | Searches for elevation and azimuth angles using MATLAB's genetic algorithm function `ga`. |
| `buildR.m` | Builds the geometry matrix from the beam elevation and azimuth angles. |
| `plotLidarBeams.m` | Plots the beam unit vectors using the same angle convention as `buildR`. |

The MATLAB filenames must match their primary function names. For example, use `buildR.m`, not `buildR(1).m`.

## Requirements

- MATLAB
- Global Optimization Toolbox for `ga`
- Base MATLAB graphics for beam visualization

## Optimization problem

The optimizer searches for six angle pairs:

```text
(theta_i, phi_i), i = 1, ..., 6
```

The implemented objective is

```text
minimize log(cond(R))
```

where `cond(R)` is the 2-norm condition number. A lower condition number generally reduces numerical sensitivity when solving the linear system, although it is not a complete statistical error model.

The current bounds are

```text
45 deg <= theta_i <= 90 deg
0 deg  <= phi_i   <= 360 deg
```

All angles are constrained to integer values on a 1-degree grid.

### Difference from Sathe et al. (2015)

This repository uses a genetic algorithm to minimize `log(cond(R))`. It does **not** reproduce the exact optimization criterion from Sathe et al. (2015), which is based on random-error amplification under assumptions about the radial-variance errors.

Because `ga` is stochastic, it can return a near-optimal geometry whose condition number is slightly higher than that of the reference geometry. Therefore, the reported condition numbers should always be compared explicitly.

## Quick start

Run the main script:

```matlab
Example_optimize_angles
```

The script will:

1. Optimize the six-beam geometry.
2. Print the optimized beam angles, matrix rank, and condition number.
3. Plot the optimized geometry.
4. Build and plot the Sathe et al. reference geometry.
5. Define a Reynolds-stress tensor in fixed coordinates.
6. Rotate the tensor into wind-aligned coordinates.
7. Generate synthetic along-beam variances.
8. Retrieve the fixed-coordinate stresses from those variances.
9. Rotate the retrieved tensor into wind coordinates.
10. Compare the true and retrieved values for both geometries.

## Example workflow

Optimize the geometry and construct the reference geometry:

```matlab
clearvars; close all; clc;

nBeams = 6;
rng(10)
[Rbest, thetaBest, phiBest] = funOptimizeAngles(nBeams);

thetaRef = [45 45 45 45 45 90];
phiRef   = [0 72 144 216 288 288];
Rref = buildR(thetaRef, phiRef);

fprintf('\nCondition number of optimized R: %.6f\n', cond(Rbest));
fprintf('Condition number of reference R: %.6f\n', cond(Rref));
```

Plot the two geometries:

```matlab
plotLidarBeams(thetaBest, phiBest, ...
    'Optimized 6-beam lidar geometry');
plotLidarBeams(thetaRef, phiRef, ...
    'Sathe et al. (2015) reference geometry');
```

Define fixed-coordinate stresses:

```matlab
xFixedTrue = [
    3.6111    % sigma_x^2
    2.9489    % sigma_y^2
    1.0000    % sigma_z^2
    0.6402    % sigma_xy
    0.4774    % sigma_xz
    0.5065    % sigma_yz
];

CfixedTrue = [xFixedTrue(1), xFixedTrue(4), xFixedTrue(5);
              xFixedTrue(4), xFixedTrue(2), xFixedTrue(6);
              xFixedTrue(5), xFixedTrue(6), xFixedTrue(3)];
```

Rotate the true tensor into wind coordinates:

```matlab
windDir = 240;
s = sind(windDir);
c = cosd(windDir);
T = [-s, -c, 0; c, -s, 0; 0, 0, 1];

CwindTrue = T * CfixedTrue * T.';
```

Generate radial variances and retrieve the fixed-coordinate stresses:

```matlab
sigma2Vr = Rbest * xFixedTrue;
xFixedRetrieved = Rbest \ sigma2Vr;
```

Rotate the retrieved tensor into wind coordinates:

```matlab
CfixedRetrieved = [xFixedRetrieved(1), xFixedRetrieved(4), xFixedRetrieved(5);
                   xFixedRetrieved(4), xFixedRetrieved(2), xFixedRetrieved(6);
                   xFixedRetrieved(5), xFixedRetrieved(6), xFixedRetrieved(3)];

CwindRetrieved = T * CfixedRetrieved * T.';
```

For the supplied fixed stresses and `windDir = 240`, the corresponding wind-aligned vector is approximately

```text
[3.999979;
 2.560021;
 1.000000;
 0.033359;
 0.666691;
 0.199942]
```

The small difference from `[4; 2.56; 1; 0.033333; 0.666667; 0.2]` is caused by rounding the supplied fixed-coordinate stresses to four decimal places.

## Interpreting the synthetic retrieval

In the example, the same matrix first generates and then retrieves noise-free radial variances:

```matlab
sigma2Vr = R * xFixedTrue;
xFixedRetrieved = R \ sigma2Vr;
```

Consequently, every full-rank geometry should recover the prescribed stresses to floating-point precision. Errors around `1e-16` are numerical roundoff, not physical retrieval error.

This test verifies the algebra and coordinate transformations. It does not establish robustness to measurement noise, spatial separation between beams, nonstationarity, or violations of horizontal homogeneity. Those effects require noisy simulations, Monte Carlo analysis, or experimental data.

## Reproducibility and equivalent geometries

The optimization uses a genetic algorithm, so its result can vary unless the random seed is fixed. The example uses

```matlab
rng(10)
```

Geometries can also be physically equivalent under beam reordering, a common azimuthal rotation, or an arbitrary azimuth assigned to a vertical beam. Because the six-component stress vector does not use orthonormal tensor weighting, numerically reported condition numbers for rotated geometries can be nearly, rather than exactly, identical.

## Reference

Sathe, A., Mann, J., Vasiljevic, N., & Lea, G. (2015). A six-beam method to measure turbulence statistics using ground-based wind lidars. *Atmospheric Measurement Techniques*, 8(2), 729-740. [https://doi.org/10.5194/amt-8-729-2015](https://doi.org/10.5194/amt-8-729-2015)

If you use this repository, cite Sathe et al. (2015) for the six-beam lidar concept and state that the optimization implementation used here is not identical to the optimization method in that paper.

## License

BSD-3-Clause
