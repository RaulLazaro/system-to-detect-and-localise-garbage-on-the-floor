% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 895.305144970268543 ; 896.487865437919936 ];

%-- Principal point:
cc = [ 640.902040418940601 ; 365.550485592410041 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.068113567133384 ; -0.169313241363402 ; 0.000534926914968 ; 0.001102745750452 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.941413270677502 ; 2.614051601286539 ];

%-- Principal point uncertainty:
cc_error = [ 5.924504608559714 ; 3.310231867583364 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.011288826225390 ; 0.040570379697268 ; 0.001215096402191 ; 0.002468896838624 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 720;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 22;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -5.673325e-02 ; -1.640380e-02 ; 1.570856e+00 ];
Tc_1  = [ 1.920330e+02 ; -1.077500e+02 ; 6.070956e+02 ];
omc_error_1 = [ 1.107427e-02 ; 1.103658e-02 ; 1.101338e-03 ];
Tc_error_1  = [ 4.040177e+00 ; 2.260283e+00 ; 2.577125e+00 ];

%-- Image #2:
omc_2 = [ 2.033047e-01 ; 3.030100e-01 ; 1.565129e+00 ];
Tc_2  = [ 1.750381e+02 ; -1.078679e+02 ; 5.433825e+02 ];
omc_error_2 = [ 8.428956e-03 ; 8.902388e-03 ; 1.433179e-03 ];
Tc_error_2  = [ 3.665206e+00 ; 2.034785e+00 ; 2.470433e+00 ];

%-- Image #3:
omc_3 = [ 5.711864e-01 ; 6.620767e-01 ; 1.500106e+00 ];
Tc_3  = [ 1.154926e+02 ; -1.084889e+02 ; 4.800051e+02 ];
omc_error_3 = [ 6.554505e-03 ; 6.968749e-03 ; 2.436941e-03 ];
Tc_error_3  = [ 3.249764e+00 ; 1.794482e+00 ; 2.273906e+00 ];

%-- Image #4:
omc_4 = [ -3.779050e-01 ; -2.589877e-01 ; 1.533310e+00 ];
Tc_4  = [ 1.852061e+02 ; -1.072814e+02 ; 6.752230e+02 ];
omc_error_4 = [ 8.121856e-03 ; 7.876257e-03 ; 2.183730e-03 ];
Tc_error_4  = [ 4.484269e+00 ; 2.502199e+00 ; 2.358433e+00 ];

%-- Image #5:
omc_5 = [ -7.750849e-01 ; -7.021938e-01 ; 1.420729e+00 ];
Tc_5  = [ 1.433191e+02 ; -1.067683e+02 ; 7.648021e+02 ];
omc_error_5 = [ 6.449187e-03 ; 6.172606e-03 ; 3.275417e-03 ];
Tc_error_5  = [ 5.097584e+00 ; 2.842313e+00 ; 2.189545e+00 ];

%-- Image #6:
omc_6 = [ -9.478508e-01 ; -3.021602e-01 ; 1.270198e+00 ];
Tc_6  = [ 1.744287e+02 ; -8.808484e+01 ; 7.817307e+02 ];
omc_error_6 = [ 5.880704e-03 ; 6.341204e-03 ; 3.756261e-03 ];
Tc_error_6  = [ 5.207744e+00 ; 2.912111e+00 ; 2.372075e+00 ];

%-- Image #7:
omc_7 = [ -7.496993e-01 ; -5.940852e-02 ; 1.386676e+00 ];
Tc_7  = [ 2.051506e+02 ; -8.723653e+01 ; 7.384083e+02 ];
omc_error_7 = [ 6.464423e-03 ; 6.913896e-03 ; 3.235919e-03 ];
Tc_error_7  = [ 4.905700e+00 ; 2.753155e+00 ; 2.459077e+00 ];

%-- Image #8:
omc_8 = [ -3.665945e-01 ; 3.123612e-01 ; 1.539523e+00 ];
Tc_8  = [ 2.205921e+02 ; -8.997585e+01 ; 6.519813e+02 ];
omc_error_8 = [ 7.690401e-03 ; 8.078309e-03 ; 2.182836e-03 ];
Tc_error_8  = [ 4.320004e+00 ; 2.429504e+00 ; 2.640218e+00 ];

%-- Image #9:
omc_9 = [ -6.440440e-02 ; 8.241566e-01 ; 1.630079e+00 ];
Tc_9  = [ 1.999112e+02 ; -7.674696e+01 ; 5.693174e+02 ];
omc_error_9 = [ 7.104481e-03 ; 7.316266e-03 ; 2.069610e-03 ];
Tc_error_9  = [ 3.800579e+00 ; 2.131508e+00 ; 2.629353e+00 ];

%-- Image #10:
omc_10 = [ 7.477948e-01 ; 1.489138e-01 ; 1.376162e+00 ];
Tc_10  = [ 6.039677e+01 ; -8.973039e+01 ; 4.408655e+02 ];
omc_error_10 = [ 6.118139e-03 ; 7.097439e-03 ; 2.801845e-03 ];
Tc_error_10  = [ 2.959556e+00 ; 1.643386e+00 ; 2.034829e+00 ];

%-- Image #11:
omc_11 = [ 3.194334e-01 ; -3.439184e-01 ; 1.546397e+00 ];
Tc_11  = [ 1.510862e+02 ; -8.617098e+01 ; 5.634592e+02 ];
omc_error_11 = [ 8.011934e-03 ; 8.598286e-03 ; 1.720325e-03 ];
Tc_error_11  = [ 3.755934e+00 ; 2.106671e+00 ; 2.433537e+00 ];

%-- Image #12:
omc_12 = [ -4.157960e-02 ; -7.407711e-01 ; 1.620486e+00 ];
Tc_12  = [ 1.752861e+02 ; -8.291305e+01 ; 6.825834e+02 ];
omc_error_12 = [ 7.617470e-03 ; 7.466832e-03 ; 1.743584e-03 ];
Tc_error_12  = [ 4.514539e+00 ; 2.527710e+00 ; 2.399782e+00 ];

%-- Image #13:
omc_13 = [ -2.742580e-01 ; -1.030569e+00 ; 1.648537e+00 ];
Tc_13  = [ 1.589967e+02 ; -7.726265e+01 ; 7.650226e+02 ];
omc_error_13 = [ 7.133294e-03 ; 6.675371e-03 ; 2.411148e-03 ];
Tc_error_13  = [ 5.065429e+00 ; 2.831369e+00 ; 2.436098e+00 ];

%-- Image #14:
omc_14 = [ -5.041333e-02 ; 9.464653e-03 ; 1.571333e+00 ];
Tc_14  = [ 1.947049e+02 ; -1.091134e+02 ; 4.246796e+02 ];
omc_error_14 = [ 7.723209e-03 ; 7.470954e-03 ; 8.373431e-04 ];
Tc_error_14  = [ 2.846916e+00 ; 1.595236e+00 ; 2.071033e+00 ];

%-- Image #15:
omc_15 = [ -4.367468e-01 ; -3.859541e-01 ; 1.525587e+00 ];
Tc_15  = [ 1.612133e+02 ; -1.083967e+02 ; 5.621138e+02 ];
omc_error_15 = [ 7.103733e-03 ; 6.499793e-03 ; 2.138062e-03 ];
Tc_error_15  = [ 3.750228e+00 ; 2.069422e+00 ; 1.868224e+00 ];

%-- Image #16:
omc_16 = [ 3.214241e-01 ; 3.565177e-01 ; 1.547296e+00 ];
Tc_16  = [ 1.836466e+02 ; -1.085808e+02 ; 4.322390e+02 ];
omc_error_16 = [ 7.203038e-03 ; 7.987774e-03 ; 1.559082e-03 ];
Tc_error_16  = [ 2.942432e+00 ; 1.641211e+00 ; 2.084514e+00 ];

%-- Image #17:
omc_17 = [ 3.068058e-01 ; -3.801493e-01 ; 1.551827e+00 ];
Tc_17  = [ 1.670247e+02 ; -8.538165e+01 ; 4.543454e+02 ];
omc_error_17 = [ 6.971876e-03 ; 7.502430e-03 ; 1.519818e-03 ];
Tc_error_17  = [ 3.035322e+00 ; 1.708826e+00 ; 2.028368e+00 ];

%-- Image #18:
omc_18 = [ -4.390880e-01 ; 3.693140e-01 ; 1.523858e+00 ];
Tc_18  = [ 1.663318e+02 ; -8.313210e+01 ; 5.370799e+02 ];
omc_error_18 = [ 6.781485e-03 ; 6.800460e-03 ; 2.323072e-03 ];
Tc_error_18  = [ 3.562352e+00 ; 1.976753e+00 ; 2.069548e+00 ];

%-- Image #19:
omc_19 = [ -7.893908e-01 ; -4.765109e-02 ; 1.365667e+00 ];
Tc_19  = [ 1.274534e+02 ; -8.496068e+01 ; 6.165973e+02 ];
omc_error_19 = [ 6.295921e-03 ; 6.239552e-03 ; 3.294540e-03 ];
Tc_error_19  = [ 4.101918e+00 ; 2.269326e+00 ; 1.764433e+00 ];

%-- Image #20:
omc_20 = [ 7.501096e-02 ; 8.550035e-01 ; 1.632626e+00 ];
Tc_20  = [ 1.401158e+02 ; -8.579843e+01 ; 4.004277e+02 ];
omc_error_20 = [ 6.448995e-03 ; 6.581699e-03 ; 1.991513e-03 ];
Tc_error_20  = [ 2.679712e+00 ; 1.486925e+00 ; 1.768402e+00 ];

%-- Image #21:
omc_21 = [ -2.502821e-01 ; -8.578068e-01 ; 1.620008e+00 ];
Tc_21  = [ 2.245251e+02 ; -8.903912e+01 ; 5.850283e+02 ];
omc_error_21 = [ 6.900114e-03 ; 6.137278e-03 ; 2.057627e-03 ];
Tc_error_21  = [ 3.895158e+00 ; 2.194188e+00 ; 2.144851e+00 ];

%-- Image #22:
omc_22 = [ 8.643885e-01 ; 1.180141e-01 ; 1.302960e+00 ];
Tc_22  = [ 8.203528e+01 ; -7.906921e+01 ; 3.584223e+02 ];
omc_error_22 = [ 5.721352e-03 ; 6.598783e-03 ; 3.061133e-03 ];
Tc_error_22  = [ 2.426133e+00 ; 1.339381e+00 ; 1.667810e+00 ];

