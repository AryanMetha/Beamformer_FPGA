%% ========================================================================
%  24-ELEMENT PHASED ARRAY BEAMFORMING COMPARISON
%  ========================================================================
%  Compares: 1
%  
%  Array: 24-element ULA, back-baffled isotropic, λ/2 spacing
%  Test cases: SNR improvement, beampatterns, DOA accuracy, interference
%  
%  Requires: Phased Array System Toolbox
%  Run in MATLAB R2025a
%  ========================================================================
clear; close all; clc;
rng(42);  % Reproducible results

%% ========================================================================
%  SECTION 1: ARRAY CONFIGURATION
%% ========================================================================
fprintf('\n');
fprintf('%s\n', repmat('=', 1, 60));
fprintf('\n');
fprintf('  24-ELEMENT PHASED ARRAY BEAMFORMING COMPARISON\n');
fprintf('%s\n', repmat('=', 1, 60));
fprintf('\n');

N      = 24;        % Number of elements
lambda = 1;         % Wavelength (normalized to 1)
d      = lambda/2;  % Element spacing = λ/2 (half-wavelength)
fc     = 1;         % Carrier frequency (normalized)

% Create back-baffled isotropic ULA
array = phased.ULA( ...
    'NumElements',    N, ...
    'ElementSpacing', d, ...
    'Element',        phased.IsotropicAntennaElement('BackBaffled', true));

fprintf('ARRAY CONFIGURATION:\n');
fprintf('  Elements:        %d\n', N);
fprintf('  Spacing:         λ/2 = %.2f\n', d);
fprintf('  Type:            Back-baffled isotropic ULA\n');
fprintf('  Aperture size:   %.1f λ\n', (N-1)*d);

%% ========================================================================
%  SECTION 2: TEST CASE DEFINITION
%% ========================================================================
% Main beam direction
scanAngle = 20;              % Degrees
% Interference / jammer directions
jammerAngles = [-50, 45];    % Two jammers

% Signal parameters
snr_in_dB   = -5;            % Input SNR per element (dB) - challenging
jnr_dB      = 15;            % Jammer-to-noise ratio (dB)
numSnapshots = 200;          % Snapshots for covariance estimation
numTrials    = 50;           % Trials for averaging statistics (not looped here, kept for extension)

fprintf('\nTEST CASE PARAMETERS:\n');
fprintf('  Scan angle (desired):  %g°\n', scanAngle);
fprintf('  Jammer angles:         [%s]°\n', sprintf('%g ', jammerAngles));
fprintf('  Input SNR per element: %g dB\n', snr_in_dB);
fprintf('  Jammer-to-noise ratio: %g dB\n', jnr_dB);
fprintf('  Snapshots (cov):       %d\n', numSnapshots);
fprintf('  Trials (averaging):    %d (single run in this script)\n', numTrials);

%% ========================================================================
%  SECTION 3: SIGNAL GENERATION
%% ========================================================================
fprintf('\nGenerating signals...\n');

% Signal angle and range (normalized)
sig_angle = [scanAngle; 0];
jam_angles = [jammerAngles(1:length(jammerAngles)); zeros(1,length(jammerAngles))];

% Complex sinusoidal signal (narrowband model)
t = (0:numSnapshots-1)';

sig_baseband = exp(1j * 2*pi * 0.1 * t);  % Desired signal modulation frequency 0.1 Hz

% Collector for array elements
collector = phased.Collector('Sensor', array, 'OperatingFrequency', fc);

% Desired signal
X_sig = collector(sig_baseband, sig_angle).';   % size: N x L

% Jammers
X_jam = zeros(N, numSnapshots, length(jammerAngles));
for k = 1:length(jammerAngles)
    % Jammer complex baseband (can be same or different frequency)
    jam_baseband = exp(1j * 2*pi * 0.05 * t);  % Jammer modulation frequency 0.05 Hz
    jam_angle    = [jammerAngles(k); 0];
    X_jam(:,:,k) = collector(jam_baseband, jam_angle).';  % N x L
end

% White Gaussian noise (per element)
noisePow = 10^(-snr_in_dB/10);
noise = sqrt(noisePow/2) * (randn(N, numSnapshots) + 1j*randn(N, numSnapshots));

% Combined input (signal + jammers + noise)
X = X_sig;
for k = 1:length(jammerAngles)
    X = X + 10^(jnr_dB/20) * X_jam(:,:,k);  % Add jammers
end
X = X + noise;

% Estimate sample covariance matrix
R = (X * X') / numSnapshots;

fprintf('  ✓ Signal + %d jammers + noise generated\n', length(jammerAngles));
fprintf('  ✓ Sample covariance matrix estimated\n');

%% ========================================================================
%  SECTION 4: BEAMFORMER DESIGN
%% ========================================================================
fprintf('\nDesigning beamformers...\n');

% Steering vector toward desired direction
a_desired = steervec(getElementPosition(array)/lambda, sig_angle); % robust use of positions

% ---- 4.1 DAS (Conventional / Phase Shift Beamformer) ----
w_das = a_desired / norm(a_desired);
fprintf('  ✓ DAS (Delay-and-Sum): wᴴ·a(θ)\n');

% ---- 4.2 MVDR (Minimum Variance Distortionless Response / Capon) ----
% MVDR: w = R⁻¹·a / (aᴴ·R⁻¹·a)
R_inv  = inv(R + 1e-3*eye(N));  % diagonal loading for stability
w_mvdr = (R_inv * a_desired) / (a_desired' * R_inv * a_desired);
fprintf('  ✓ MVDR (Capon): min wᴴRw s.t. wᴴa=1\n');

% ---- 4.3 LCMV (Linearly Constrained Minimum Variance) ----
% Multiple constraints: pass desired, null jammers
% C = [a_desired, a_jam1, a_jam2, ...]
% g = [1, 0, 0, ...] (pass desired, null jammers)
C_lcmv = a_desired;
g_lcmv = 1;
for k = 1:length(jammerAngles)
    a_jam = steervec(getElementPosition(array)/lambda, [jammerAngles(k); 0]);
    C_lcmv = [C_lcmv, a_jam];
    g_lcmv = [g_lcmv; 0];  % Null constraint
end

% LCMV: w = R⁻¹·C·(Cᴴ·R⁻¹·C)⁻¹·g
term1 = R_inv * C_lcmv;
term2 = C_lcmv' * term1;
w_lcmv = term1 * (term2 \ g_lcmv);
fprintf('  ✓ LCMV: min wᴴRw s.t. Cᴴw=[1;0;0;...] (pass + nulls)\n');

% ---- 4.4 GSC (Generalized Sidelobe Canceller) ----
% GSC = DAS beamformer + adaptive interference canceller
% 1. Primary path: scaled DAS weights
% 2. Blocking matrix: orthogonal to steering vector (removes signal)
% 3. Adaptive weights minimize output power
%
% GSC weight: w = w_das - B * w_aux_opt
% where w_aux_opt = (R_aux + δI)^(-1) BᴴR w_das

% Blocking matrix (orthogonal to a_desired)
Q_all = null(a_desired');            % Null space (orthogonal to a_desired)
numAux = min(5, N-1);                % Use up to 5 auxiliary channels
B_gsc = Q_all(:, 1:numAux);         % Blocking matrix

% Auxiliary output
y_aux = B_gsc' * X;
R_aux = (y_aux * y_aux') / numSnapshots;

% Optimal cancellation weights
w_aux_opt = (R_aux + 1e-3*eye(size(R_aux))) \ (B_gsc' * R * w_das);

% GSC weights
w_gsc = w_das - B_gsc * w_aux_opt;
% Normalize to unity gain in look direction
w_gsc = w_gsc / (a_desired' * w_gsc);

fprintf('  ✓ GSC (Generalized Sidelobe Canceller): DAS + blocking matrix\n');
fprintf('\n  All beamformers designed successfully.\n');

%% ========================================================================
%  SECTION 5: BEAMPATTERN GENERATION
%% ========================================================================
fprintf('\nGenerating beampatterns...\n');

angles_scan = -90:0.5:90;  % Dense angle grid for plotting
num_angles  = length(angles_scan);

% Initialize beampatterns (magnitude in dB)
bp_das  = zeros(1, num_angles);
bp_mvdr = zeros(1, num_angles);
bp_lcmv = zeros(1, num_angles);
bp_gsc  = zeros(1, num_angles);

% Precompute element positions for steering
pos = getElementPosition(array)/lambda;

% Compute beampattern at each angle
for idx = 1:num_angles
    angle   = angles_scan(idx);
    a_angle = steervec(pos, [angle; 0]);
    
    % Beamformer outputs (complex)
    y_das  = w_das'  * a_angle;
    y_mvdr = w_mvdr' * a_angle;
    y_lcmv = w_lcmv' * a_angle;
    y_gsc  = w_gsc'  * a_angle;
    
    % Magnitude in dB
    bp_das(idx)  = 20*log10(abs(y_das)  + 1e-10);
    bp_mvdr(idx) = 20*log10(abs(y_mvdr) + 1e-10);
    bp_lcmv(idx) = 20*log10(abs(y_lcmv) + 1e-10);
    bp_gsc(idx)  = 20*log10(abs(y_gsc)  + 1e-10);
end

% Normalize to 0 dB at peak
bp_das  = bp_das  - max(bp_das);
bp_mvdr = bp_mvdr - max(bp_mvdr);
bp_lcmv = bp_lcmv - max(bp_lcmv);
bp_gsc  = bp_gsc  - max(bp_gsc);

fprintf('  ✓ Beampatterns computed over [-90°, 90°]\n');

%% ========================================================================
%  SECTION 6: PERFORMANCE METRICS
%% ========================================================================
fprintf('\nComputing performance metrics...\n');

% Test signal (no jammers for SNR calculation: signal + noise only)
X_test = X_sig + noise;

% Output SNR for each beamformer
output_snr_das  = compute_snr(X_test, w_das);
output_snr_mvdr = compute_snr(X_test, w_mvdr);
output_snr_lcmv = compute_snr(X_test, w_lcmv);
output_snr_gsc  = compute_snr(X_test, w_gsc);

% Array gain (improvement over single element)
single_elem_snr = snr_in_dB;
array_gain_das  = output_snr_das  - single_elem_snr;
array_gain_mvdr = output_snr_mvdr - single_elem_snr;
array_gain_lcmv = output_snr_lcmv - single_elem_snr;
array_gain_gsc  = output_snr_gsc  - single_elem_snr;

% Output with jammers (interference rejection): use full X with jammers
X_jammed = X;
output_snir_das  = compute_snr(X_jammed, w_das);
output_snir_mvdr = compute_snr(X_jammed, w_mvdr);
output_snir_lcmv = compute_snr(X_jammed, w_lcmv);
output_snir_gsc  = compute_snr(X_jammed, w_gsc);

% Directivity (approximate from beampattern)
directivity_das  = compute_directivity(angles_scan, bp_das);
directivity_mvdr = compute_directivity(angles_scan, bp_mvdr);
directivity_lcmv = compute_directivity(angles_scan, bp_lcmv);
directivity_gsc  = compute_directivity(angles_scan, bp_gsc);

% Mainlobe beamwidth (3 dB)
bw_das  = compute_beamwidth(angles_scan, bp_das);
bw_mvdr = compute_beamwidth(angles_scan, bp_mvdr);
bw_lcmv = compute_beamwidth(angles_scan, bp_lcmv);
bw_gsc  = compute_beamwidth(angles_scan, bp_gsc);

% Maximum sidelobe level (exclude ±5° around main lobe)
msl_das  = max(bp_das( abs(angles_scan - scanAngle) > 5 ));
msl_mvdr = max(bp_mvdr(abs(angles_scan - scanAngle) > 5 ));
msl_lcmv = max(bp_lcmv(abs(angles_scan - scanAngle) > 5 ));
msl_gsc  = max(bp_gsc( abs(angles_scan - scanAngle) > 5 ));

fprintf('  ✓ SNR, Array Gain, Directivity, Beamwidth, Sidelobe Level\n');

%% ========================================================================
%  SECTION 7: RESULTS TABLE
%% ========================================================================
fprintf('\n');
fprintf('%s\n', repmat('=', 1, 60));
fprintf('\n');
fprintf('PERFORMANCE METRICS COMPARISON\n');
fprintf('%s\n', repmat('=', 1, 60));
fprintf('\n');

algorithms = {'DAS'; 'MVDR'; 'LCMV'; 'GSC'};

metrics_table = table( ...
    algorithms, ...
    [output_snr_das;  output_snr_mvdr;  output_snr_lcmv;  output_snr_gsc], ...
    [output_snir_das; output_snir_mvdr; output_snir_lcmv; output_snir_gsc], ...
    [array_gain_das;  array_gain_mvdr;  array_gain_lcmv;  array_gain_gsc], ...
    [directivity_das; directivity_mvdr; directivity_lcmv; directivity_gsc], ...
    [bw_das;          bw_mvdr;          bw_lcmv;          bw_gsc], ...
    [msl_das;         msl_mvdr;         msl_lcmv;         msl_gsc], ...
    'VariableNames', {'Algorithm', 'Output_SNR_dB', 'Output_SNIR_dB', ...
                      'Array_Gain_dB', 'Directivity_dBi', ...
                      'Beamwidth_deg', 'MSL_dB'});

disp(metrics_table);

fprintf('\n');
fprintf('KEY OBSERVATIONS:\n');
fprintf('  Output SNR:   Signal + noise only (ideal case)\n');
fprintf('  Output SNIR:  Signal + jammers + noise (realistic case)\n');
fprintf('  Array Gain:   SNR improvement over single element\n');
fprintf('  Directivity:  Concentration of energy in main lobe\n');
fprintf('  Beamwidth:    3 dB mainlobe width\n');
fprintf('  MSL:          Maximum sidelobe level (away from main lobe)\n');
fprintf('\n');

%% ========================================================================
%  SECTION 8: PLOTTING
%% ========================================================================
fprintf('Generating plots...\n');

alg_names = {'DAS','MVDR','LCMV','GSC'};

% Figure 1: Beampatterns comparison
figure('Name', 'Beampatterns', 'NumberTitle', 'off', 'Position', [100 100 1200 600]);
plot(angles_scan, bp_das,  'LineWidth', 2, 'DisplayName', 'DAS');  hold on;
plot(angles_scan, bp_mvdr, 'LineWidth', 2, 'DisplayName', 'MVDR');
plot(angles_scan, bp_lcmv, 'LineWidth', 2, 'DisplayName', 'LCMV');
plot(angles_scan, bp_gsc,  'LineWidth', 2, 'DisplayName', 'GSC');

% Mark scan angle and jammers
xline(scanAngle, 'k--', 'LineWidth', 1.5, 'Alpha', 0.6, 'DisplayName', 'Desired');
for k = 1:length(jammerAngles)
    xline(jammerAngles(k), 'r--', 'LineWidth', 1.5, 'Alpha', 0.6);
end

xlabel('Angle (degrees)', 'FontSize', 11);
ylabel('Magnitude (dB)', 'FontSize', 11);
title('Beampatterns: DAS vs MVDR vs LCMV vs GSC', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
ylim([-60, 5]);
xlim([-90, 90]);

% Figure 2: SNR Performance
figure('Name', 'SNR Performance', 'NumberTitle', 'off', 'Position', [100 700 1200 400]);
output_snr  = [output_snr_das,  output_snr_mvdr,  output_snr_lcmv,  output_snr_gsc];
output_snir = [output_snir_das, output_snir_mvdr, output_snir_lcmv, output_snir_gsc];

subplot(1, 2, 1);
bar(1:4, output_snr, 'FaceColor', [0.2 0.6 0.9], 'EdgeColor', 'k', 'LineWidth', 1.5);
hold on;
yline(snr_in_dB, 'r--', 'LineWidth', 2, 'DisplayName', 'Input SNR');
xlabel('Algorithm', 'FontSize', 11);
ylabel('Output SNR (dB)', 'FontSize', 11);
title('Output SNR (Signal + Noise)', 'FontSize', 11, 'FontWeight', 'bold');
set(gca, 'XTick', 1:4, 'XTickLabel', alg_names);
grid on; grid minor;
legend('FontSize', 9);

subplot(1, 2, 2);
bar(1:4, output_snir, 'FaceColor', [0.9 0.4 0.2], 'EdgeColor', 'k', 'LineWidth', 1.5);
xlabel('Algorithm', 'FontSize', 11);
ylabel('Output SNIR (dB)', 'FontSize', 11);
title('Output SNIR (Signal + Jammers + Noise)', 'FontSize', 11, 'FontWeight', 'bold');
set(gca, 'XTick', 1:4, 'XTickLabel', alg_names);
grid on; grid minor;

% Figure 3: Array Gain & Directivity
figure('Name', 'Array Gain & Directivity', 'NumberTitle', 'off', 'Position', [100 1300 1200 400]);
array_gains   = [array_gain_das,  array_gain_mvdr,  array_gain_lcmv,  array_gain_gsc];
directivities = [directivity_das, directivity_mvdr, directivity_lcmv, directivity_gsc];
theoretical_max = 10*log10(N);  % Theoretical max for N elements

subplot(1, 2, 1);
bar(1:4, array_gains, 'FaceColor', [0.2 0.8 0.4], 'EdgeColor', 'k', 'LineWidth', 1.5);
hold on;
yline(theoretical_max, 'r--', 'LineWidth', 2, ...
    'DisplayName', sprintf('Theoretical Max (10·log_{10}(N) = %.1f dB)', theoretical_max));
xlabel('Algorithm', 'FontSize', 11);
ylabel('Array Gain (dB)', 'FontSize', 11);
title('Array Gain (dB)', 'FontSize', 11, 'FontWeight', 'bold');
set(gca, 'XTick', 1:4, 'XTickLabel', alg_names);
grid on; grid minor;
legend('FontSize', 9);

subplot(1, 2, 2);
bar(1:4, directivities, 'FaceColor', [0.8 0.7 0.2], 'EdgeColor', 'k', 'LineWidth', 1.5);
hold on;
yline(theoretical_max, 'r--', 'LineWidth', 2, ...
    'DisplayName', sprintf('Theoretical (10·log_{10}(N) = %.1f dB)', theoretical_max));
xlabel('Algorithm', 'FontSize', 11);
ylabel('Directivity (dBi)', 'FontSize', 11);
title('Directivity (dBi)', 'FontSize', 11, 'FontWeight', 'bold');
set(gca, 'XTick', 1:4, 'XTickLabel', alg_names);
grid on; grid minor;
legend('FontSize', 9);

% Figure 4: Beamwidth & MSL
figure('Name', 'Beamwidth & MSL', 'NumberTitle', 'off', 'Position', [100 1900 1200 400]);
beamwidths = [bw_das, bw_mvdr, bw_lcmv, bw_gsc];
msls       = [msl_das, msl_mvdr, msl_lcmv, msl_gsc];

subplot(1, 2, 1);
bar(1:4, beamwidths, 'FaceColor', [0.6 0.3 0.8], 'EdgeColor', 'k', 'LineWidth', 1.5);
xlabel('Algorithm', 'FontSize', 11);
ylabel('3-dB Beamwidth (degrees)', 'FontSize', 11);
title('Mainlobe Beamwidth (3 dB)', 'FontSize', 11, 'FontWeight', 'bold');
set(gca, 'XTick', 1:4, 'XTickLabel', alg_names);
grid on; grid minor;

subplot(1, 2, 2);
bar(1:4, msls, 'FaceColor', [0.9 0.6 0.3], 'EdgeColor', 'k', 'LineWidth', 1.5);
xlabel('Algorithm', 'FontSize', 11);
ylabel('Max Sidelobe Level (dB)', 'FontSize', 11);
title('Maximum Sidelobe Level', 'FontSize', 11, 'FontWeight', 'bold');
set(gca, 'XTick', 1:4, 'XTickLabel', alg_names);
grid on; grid minor;

% Figure 5: Zoomed beampatterns around scan angle
figure('Name', 'Beampatterns (Zoomed)', 'NumberTitle', 'off', 'Position', [1350 100 1200 600]);
plot(angles_scan, bp_das,  'LineWidth', 2, 'DisplayName', 'DAS');  hold on;
plot(angles_scan, bp_mvdr, 'LineWidth', 2, 'DisplayName', 'MVDR');
plot(angles_scan, bp_lcmv, 'LineWidth', 2, 'DisplayName', 'LCMV');
plot(angles_scan, bp_gsc,  'LineWidth', 2, 'DisplayName', 'GSC');
xline(scanAngle, 'k--', 'LineWidth', 1.5, 'Alpha', 0.6);
for k = 1:length(jammerAngles)
    xline(jammerAngles(k), 'r--', 'LineWidth', 1.5, 'Alpha', 0.6);
end
xlabel('Angle (degrees)', 'FontSize', 11);
ylabel('Magnitude (dB)', 'FontSize', 11);
title('Beampatterns (Zoomed): [-60°, 80°]', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 10);
grid on;
xlim([scanAngle - 60, scanAngle + 60]);
ylim([-60, 5]);

fprintf('  ✓ All plots generated\n');

%% ========================================================================
%  SECTION 9: SUMMARY & CONCLUSIONS
%% ========================================================================
fprintf('\n');
fprintf('%s\n', repmat('=', 1, 60));
fprintf('\n');
fprintf('ALGORITHM SUMMARY & RECOMMENDATIONS\n');
fprintf('%s\n', repmat('=', 1, 60));
fprintf('\n');

fprintf('1. DAS (Conventional Beamforming)\n');
fprintf('   • Simplest, lowest complexity\n');
fprintf('   • Uses fixed steering vector, no adaptation\n');
fprintf('   • Poor interference suppression\n');
fprintf('   • Use when: Computational budget is tight, interference is low\n\n');

fprintf('2. MVDR (Minimum Variance Distortionless Response / Capon)\n');
fprintf('   • Adaptive, uses covariance matrix\n');
fprintf('   • Excellent interference suppression with deep nulls\n');
fprintf('   • Requires accurate covariance estimation\n');
fprintf('   • Use when: One main signal + interference, low SNR OK\n\n');

fprintf('3. LCMV (Linearly Constrained Minimum Variance)\n');
fprintf('   • Multiple linear constraints (pass desired + null jammers)\n');
fprintf('   • Can protect multiple directions simultaneously\n');
fprintf('   • More robust than MVDR for known interferers\n');
fprintf('   • Use when: Known jammer locations, multi-beam requirements\n\n');

fprintf('4. GSC (Generalized Sidelobe Canceller)\n');
fprintf('   • Hybrid: DAS primary + adaptive interference canceller\n');
fprintf('   • Good balance of stability and interference rejection\n');
fprintf('   • Less sensitive to covariance estimation errors\n');
fprintf('   • Use when: Robustness needed, moderate computational budget\n\n');

fprintf('PERFORMANCE RANKING (this test case):\n');

% Rank by SNIR (interference rejection)
snir_ranks = [output_snir_das, output_snir_mvdr, output_snir_lcmv, output_snir_gsc];
[~, idx] = sort(snir_ranks, 'descend');
fprintf('  Interference Rejection: ');
for i = 1:4
    fprintf('%s (%.1f dB) ', alg_names{idx(i)}, snir_ranks(idx(i)));
end
fprintf('\n');

% Rank by beamwidth (smaller is better)
bw_ranks = [bw_das, bw_mvdr, bw_lcmv, bw_gsc];
[~, idx] = sort(bw_ranks, 'ascend');
fprintf('  Angular Resolution:    ');
for i = 1:4
    fprintf('%s (%.2f°) ', alg_names{idx(i)}, bw_ranks(idx(i)));
end
fprintf('\n');

% Rank by MSL (more negative is better → smaller value)
msl_ranks = [msl_das, msl_mvdr, msl_lcmv, msl_gsc];
[~, idx] = sort(msl_ranks, 'ascend');
fprintf('  Sidelobe Suppression:  ');
for i = 1:4
    fprintf('%s (%.1f dB) ', alg_names{idx(i)}, msl_ranks(idx(i)));
end
fprintf('\n\n');

fprintf('%s\n', repmat('=', 1, 80));
fprintf('Analysis complete. All figures open in MATLAB.\n');
fprintf('Press any key to continue...\n');
pause;

%% ========================================================================
%  HELPER FUNCTIONS
%% ========================================================================
function snr_out = compute_snr(X, w)
    % Compute output SNR-like metric for beamformer weights w
    % X: [N x L] input data matrix
    % w: [N x 1] beamformer weight vector
    y = w' * X;  % Beamformer output
    output_power = mean(abs(y).^2);
    snr_out = 10 * log10(output_power + 1e-10);
end

function d = compute_directivity(angles, beampattern_dB)
    % Approximate directivity from beampattern (1D approximation)
    % D ≈ 41253 / (θ_3dB^2), θ_3dB in degrees
    peak   = max(beampattern_dB);
    bw_idx = find(beampattern_dB >= peak - 3);
    if isempty(bw_idx)
        bw_deg = 90;
    else
        bw_deg = max(angles(bw_idx)) - min(angles(bw_idx));
    end
    d = 10*log10(41253 / (bw_deg^2 + 1e-10));
end

function bw = compute_beamwidth(angles, beampattern_dB)
    % Find 3dB beamwidth in degrees
    peak   = max(beampattern_dB);
    bw_idx = find(beampattern_dB >= peak - 3);
    if isempty(bw_idx)
        bw = 180;
    else
        bw = max(angles(bw_idx)) - min(angles(bw_idx));
    end
end
