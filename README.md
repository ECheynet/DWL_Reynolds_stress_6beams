# Six-Beam Wind Lidar Geometry Optimization

MATLAB code for optimizing a six-beam Doppler wind lidar scanning geometry used to retrieve Reynolds-stress statistics from along-beam velocity variances.

The code builds a geometry matrix `R` such that

```text
sigma_vr_squared = R * x
```

where

```text
x = [sigma_u^2;
     sigma_v^2;
     sigma_w^2;
     sigma_uv;
     sigma_uw;
     sigma_vw]
```

and `sigma_vr_squared` contains the six along-beam velocity variances.

The beam angles are optimized to reduce the condition number of `R`. A lower condition number reduces numerical error amplification when retrieving the Reynolds stresses from the along-beam variances.

## Important note

This repository compares the optimized geometry with the six-beam reference geometry from Sathe et al. (2015), but it does **not** use the exact same optimization method as Sathe et al. (2015).

The present implementation uses MATLAB's genetic algorithm optimizer, `ga`, with integer-constrained elevation and azimuth angles on a 1-degree grid.

## Reference

Sathe, A., Mann, J., Vasiljevic, N., & Lea, G. (2015). A six-beam method to measure turbulence statistics using ground-based wind lidars. *Atmospheric Measurement Techniques*, 8(2), 729-740.

## Beam convention

For beam `i`, the line-of-sight velocity is written as

```text
v_r,i = u*cos(theta_i)*sin(phi_i)
      + v*cos(theta_i)*cos(phi_i)
      + w*sin(theta_i)
```

where:

- `theta_i` is the elevation angle, in degrees.
- `phi_i` is the azimuth angle, in degrees.
- `u`, `v`, and `w` are the three wind-velocity components.

The corresponding beam unit vector is

```text
e_i = [cos(theta_i)*sin(phi_i),
       cos(theta_i)*cos(phi_i),
       sin(theta_i)]
```

The corresponding row of `R` is

```text
R_i = [cos(theta_i)^2*sin(phi_i)^2,
       cos(theta_i)^2*cos(phi_i)^2,
       sin(theta_i)^2,
       2*cos(theta_i)^2*sin(phi_i)*cos(phi_i),
       2*cos(theta_i)*sin(theta_i)*sin(phi_i),
       2*cos(theta_i)*sin(theta_i)*cos(phi_i)]
```

## Repository contents

| File | Description |
| --- | --- |
| `Example_optimize_angles.m` | Main example script. It optimizes the beam geometry, compares it with the Sathe et al. (2015) reference geometry, plots the beams, computes synthetic along-beam variances, and retrieves the Reynolds stresses. |
| `funOptimizeAngles.m` | Optimizes the elevation and azimuth angles using MATLAB's genetic algorithm function `ga`. |
| `buildR.m` | Builds the `6 x 6` matrix `R` from elevation and azimuth angles. |
| `plotLidarBeams.m` | Plots the lidar beams as 3D unit vectors using the same angle convention as the retrieval matrix. |

## Requirements

- MATLAB
- Global Optimization Toolbox, for `ga`
- Base MATLAB graphics, for beam visualization

## Optimization problem

The optimizer searches for six pairs of angles:

```text
(theta_i, phi_i), i = 1, ..., 6
```

The objective function is

```text
minimize log(cond(R))
```

where `cond(R)` is the 2-norm condition number of the matrix `R`.

The current angular bounds are:

```text
45 deg <= theta_i <= 90 deg
0 deg  <= phi_i   <= 360 deg
```

All variables are constrained to integer values, corresponding to a 1-degree angular grid.

## Quick start

Run the main example script in MATLAB:

```matlab
Example_optimize_angles
```

The script will:

1. Optimize the six-beam geometry.
2. Print the optimized elevation and azimuth angles.
3. Print the condition number of the optimized `R` matrix.
4. Plot the optimized beam geometry.
5. Compare the optimized geometry with the Sathe et al. (2015) reference geometry.
6. Generate synthetic along-beam variances from prescribed Reynolds stresses.
7. Retrieve the Reynolds stresses using the optimized `R` matrix.

## Example workflow

Optimize the geometry:

```matlab
clearvars; close all; clc;

nBeams = 6;
rng(10)

[Rbest, thetaBest, phiBest] = funOptimizeAngles(nBeams);

fprintf('\nCondition number of optimized R : %.6f\n', cond(Rbest));
```

Plot the optimized beam directions:

```matlab
plotLidarBeams(thetaBest, phiBest, 'Optimized 6-beam lidar geometry');
```

Compare with the Sathe et al. (2015) reference geometry:

```matlab
thetaRef = [45 45 45 45 45 90];
phiRef   = [0 72 144 216 288 288];

Rref = buildR(thetaRef, phiRef);

fprintf('\nCondition number in Sathe et al. (2015) : %.6f\n', cond(Rref));

plotLidarBeams(thetaRef, phiRef, 'Sathe et al. (2015) reference geometry');
```

## Reynolds-stress retrieval example

Define a synthetic Reynolds-stress vector:

```matlab
sigma2U = 4;                 % m^2/s^2
sigma2V = sigma2U*0.8.^2;
sigma2W = sigma2U*0.5.^2;
sigmaUW = sigma2U/6;
sigmaVW = 0.3*sigmaUW;
sigmaUV = 0.05*sigmaUW;

xTrue = [
    sigma2U
    sigma2V
    sigma2W
    sigmaUV
    sigmaUW
    sigmaVW
];
```

Compute along-beam variances using the optimized geometry:

```matlab
sigma2Vr = Rbest*xTrue;
```

Retrieve the Reynolds stresses:

```matlab
xRetrieved = Rbest\sigma2Vr;
```

The MATLAB backslash operator is preferred over explicitly computing the inverse:

```matlab
% Recommended
xRetrieved = Rbest\sigma2Vr;

% Avoid
xRetrieved = inv(Rbest)*sigma2Vr;
```

## Notes on reproducibility

The optimization uses a genetic algorithm, so the output can vary between runs unless the random seed is fixed. The example script uses:

```matlab
rng(10)
```

The problem also has azimuthal symmetry. Equivalent solutions may differ by a common azimuthal rotation while giving the same, or nearly the same, condition number. For a vertical beam, the azimuth is arbitrary.

## Suggested citation

If you use this repository, cite Sathe et al. (2015) for the six-beam lidar concept:

```text
Sathe, A., Mann, J., Vasiljevic, N., & Lea, G. (2015).
A six-beam method to measure turbulence statistics using ground-based wind lidars.
Atmospheric Measurement Techniques, 8(2), 729-740.
```

Also state that the optimization implementation used here is not the exact same optimization method as in Sathe et al. (2015).

## License

BSD-3-Clause 
