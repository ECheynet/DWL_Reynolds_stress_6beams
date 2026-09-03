# Six-Beam Wind Lidar Geometry Optimization

MATLAB code for optimizing a six-beam Doppler lidar geometry and retrieving Reynolds stresses from along-beam velocity variances.

The main script contains the executable workflow only. All reusable functions are separate `.m` files in `functions/`; there are no local function definitions in the script.

## Measurement model

The implementation distinguishes between:

- fixed lidar coordinates `(x,y,z)`: east, north, and vertical;
- wind-aligned coordinates `(u,v,w)`: along wind, cross wind, and vertical.

The beam matrix acts on stresses in fixed coordinates:

```text
sigma2Vr = R * xFixed

xFixed = [sigma_x^2;
          sigma_y^2;
          sigma_z^2;
          sigma_xy;
          sigma_xz;
          sigma_yz]
```

`sigma2Vr` contains one radial-velocity variance per beam. The fixed stresses are retrieved with

```matlab
xFixedRetrieved = R \ sigma2Vr;
```

MATLAB's backslash operator is preferred over explicitly calculating `inv(R)`.

## Beam convention

For beam `i`, `theta_i` is elevation and `phi_i` is azimuth clockwise from north:

```text
v_r,i = x*cos(theta_i)*sin(phi_i)
      + y*cos(theta_i)*cos(phi_i)
      + z*sin(theta_i)
```

Thus

```text
e_i = [a_i, b_i, c_i]

a_i = cos(theta_i)*sin(phi_i)
b_i = cos(theta_i)*cos(phi_i)
c_i = sin(theta_i)
```

and row `i` of `R` is

```text
R_i = [a_i^2, b_i^2, c_i^2,
       2*a_i*b_i, 2*a_i*c_i, 2*b_i*c_i]
```

Here `theta = 0` degrees is horizontal and `theta = 90` degrees is vertical. A vertical beam's azimuth is arbitrary.

## Wind-coordinate rotation

Let `windDir` be the meteorological direction in degrees clockwise from north, indicating where the wind comes from. The fixed-to-wind transformation is

```matlab
s = sind(windDir);
c = cosd(windDir);
T = [-s, -c, 0;
      c, -s, 0;
      0,  0, 1];
```

If `qWind = T*qFixed`, the covariance tensors obey

```text
Cwind = <qWind*qWind.'>
      = T*<qFixed*qFixed.'>*T.'
      = T*Cfixed*T.'
```

Both velocity factors in the covariance must be rotated, which places `T` on the left and `T.'` on the right. Since `T` is orthogonal, the inverse transformation is

```matlab
Cfixed = T.' * Cwind * T;
```

The corresponding wind-aligned vector is

```text
xWind = [sigma_u^2;
         sigma_v^2;
         sigma_w^2;
         sigma_uv;
         sigma_uw;
         sigma_vw]
```

`xFixed` and `xWind` describe the same Reynolds-stress tensor in different coordinate systems; they are not independent inputs.

## Repository structure

```text
.
|-- Example_optimize_angles.m
|-- README.md
`-- functions/
    |-- buildR.m
    |-- funOptimizeAngles.m
    |-- plotLidarBeams.m
    |-- printBeamVariances.m
    |-- printComparison.m
    |-- retrieveStresses.m
    |-- tensorToVector.m
    `-- vectorToTensor.m
```

| File | Purpose |
| --- | --- |
| `Example_optimize_angles.m` | Function-free main script: optimization, comparison, plots, synthetic measurements, retrieval, rotation, and reporting. |
| `functions/buildR.m` | Builds `R` from elevation and azimuth angles. |
| `functions/funOptimizeAngles.m` | Optimizes the beam angles with `ga`. |
| `functions/plotLidarBeams.m` | Plots the beam unit vectors. |
| `functions/retrieveStresses.m` | Generates radial variances, retrieves fixed stresses, and rotates them. |
| `functions/printBeamVariances.m` | Prints radial variances. |
| `functions/printComparison.m` | Prints true, retrieved, and error values. |
| `functions/vectorToTensor.m` | Converts the six-component vector to a symmetric tensor. |
| `functions/tensorToVector.m` | Converts a symmetric tensor to the six-component vector. |

MATLAB function filenames must match their primary function names.

## Requirements and quick start

- MATLAB
- Global Optimization Toolbox for `ga`
- Base MATLAB graphics

From the repository root, add `functions/` to the path and run the script:

```matlab
addpath('functions')
Example_optimize_angles
```

The script:

1. optimizes and plots the six-beam geometry;
2. constructs and plots the Sathe et al. reference geometry;
3. compares their condition numbers;
4. defines a fixed-coordinate Reynolds-stress tensor;
5. converts it to wind coordinates;
6. generates synthetic radial variances and retrieves the stresses;
7. reports fixed- and wind-coordinate results for both geometries.

## Optimization

The optimizer searches for six integer-valued angle pairs on a 1-degree grid:

```text
45 deg <= theta_i <= 90 deg
0 deg  <= phi_i   <= 360 deg
```

The implemented objective is

```text
minimize log(cond(R))
```

A lower 2-norm condition number generally reduces numerical sensitivity, but it is not a complete statistical error model. This is not the exact Sathe et al. (2015) criterion, which is based on random-error amplification in the retrieved stresses.

Because `ga` is stochastic, it can return a near-optimal geometry whose condition number is slightly higher than the reference value. The example uses `rng(10)` for reproducibility. Equivalent or nearly equivalent geometries can also differ by beam ordering, a common azimuthal rotation, or the azimuth assigned to a vertical beam.

## Retrieval example

The example starts from fixed-coordinate stresses:

```matlab
xFixedTrue = [3.6111; 2.9489; 1.0000; ...
              0.6402; 0.4774; 0.5065];

windDir = 240;
s = sind(windDir);
c = cosd(windDir);
T = [-s, -c, 0; c, -s, 0; 0, 0, 1];

CfixedTrue = vectorToTensor(xFixedTrue);
CwindTrue = T * CfixedTrue * T.';
xWindTrue = tensorToVector(CwindTrue);

sigma2Vr = Rbest * xFixedTrue;
xFixedRetrieved = Rbest \ sigma2Vr;
CwindRetrieved = T * vectorToTensor(xFixedRetrieved) * T.';
xWindRetrieved = tensorToVector(CwindRetrieved);
```

For these values, `xWindTrue` is approximately

```text
[3.999979; 2.560021; 1.000000;
 0.033359; 0.666691; 0.199942]
```

The difference from `[4; 2.56; 1; 0.033333; 0.666667; 0.2]` comes from rounding `xFixedTrue` to four decimal places.

## Interpreting the test

The same matrix generates and retrieves noise-free measurements, so every full-rank geometry should recover the input to floating-point precision. Errors near `1e-16` are numerical roundoff. This verifies the algebra and coordinate rotation, not robustness to measurement noise, spatial separation, nonstationarity, or violations of horizontal homogeneity. Those effects require noisy simulations, Monte Carlo analysis, or experimental data.

## Reference

Sathe, A., Mann, J., Vasiljevic, N., & Lea, G. (2015). A six-beam method to measure turbulence statistics using ground-based wind lidars. *Atmospheric Measurement Techniques*, 8(2), 729-740. [https://doi.org/10.5194/amt-8-729-2015](https://doi.org/10.5194/amt-8-729-2015)

This repository should cite Sathe et al. for the six-beam concept while noting that its optimization implementation is different.

## License

BSD-3-Clause
