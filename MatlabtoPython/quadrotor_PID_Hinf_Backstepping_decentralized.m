clear; 
clc; 
close all;

%% ========================================================================
%  TAHAP 1: Parameter Fisik Wahana & Robustness H-Infinity Backstepping
% ========================================================================
g  = 9.81; 
m  = 1.0;
Ix = 8.1e-3; 
Iy = 8.1e-3; 
Iz = 14.2e-3;

% Batas Ketahanan Gangguan (Semakin kecil semakin kebal, tapi motor makin agresif)
% Jika gamma terlalu kecil, MATLAB akan error (Solusi Riccati Infeasible)
gamma_out = 500.0; 
gamma_in  = 500.0;

% [PERBAIKAN] Pemisahan Parameter Tuning Kalang Luar & Dalam
rho1_out = 5.0;   % Penalti besar agar komando target sudut tidak ekstrem
rho2_out = -1.0;  % Redaman backstepping posisi

rho1_in  = 0.8;   % Penalti sikap agar torsi motor tidak meledak (naik dari 0.01)
rho2_in  = -5.0;  % Redaman backstepping sikap (diperbesar keras agar sudut tidak bergetar)

p.l  = 0.24;             % Panjang lengan untuk inversi gaya motor di plot
p.bd = 4.1e-6 / 54.2e-6; % Rasio drag/lift

%% ========================================================================
%  TAHAP 2: Sintesis Gain Kontroler PID-Hinf Backstepping Desentralisasi
% ========================================================================

% ── 1. SUBSISTEM X (Lateral - Orde 4) ────────────────────────────────────
% 1.1 OUTER LOOP: X-Position ke Target Pitch
A_x_out = [0 1; 0 0];
B_x_out = [0; g]; 
C_x_out = [1 0];
Q_x_out = diag([5 8]); % Bobot kecepatan dinaikkan agar ngerem lebih awal
[Kp_x_out, Ki_x_out, Kd_x_out, Kx_x_out, Ku_x_out] = solve_pid_hinf_backstepping(A_x_out, B_x_out, C_x_out, Q_x_out, gamma_out, rho1_out, rho2_out);

% 1.2 INNER LOOP: Pitch Angle ke Torsi Motor
A_x_in = [0 1; 0 0];
B_x_in = [0; 1/Iy];
C_x_in = [1 0];
Q_x_in = diag([10 5]); % Redaman sudut (dtheta) dinaikkan drastis dari 1 ke 5
[Kp_x_in, Ki_x_in, Kd_x_in, Kx_x_in, Ku_x_in] = solve_pid_hinf_backstepping(A_x_in, B_x_in, C_x_in, Q_x_in, gamma_in, rho1_in, rho2_in);

% ── 2. SUBSISTEM Y (Longitudinal - Orde 4) ───────────────────────────────
% 2.1 OUTER LOOP: Y-Position ke Target Roll
A_y_out = [0 1; 0 0];
B_y_out = [0; -g]; 
C_y_out = [1 0];
Q_y_out = diag([5 8]); 
[Kp_y_out, Ki_y_out, Kd_y_out, Kx_y_out, Ku_y_out] = solve_pid_hinf_backstepping(A_y_out, B_y_out, C_y_out, Q_y_out, gamma_out, rho1_out, rho2_out);

% 2.2 INNER LOOP: Roll Angle ke Torsi Motor
A_y_in = [0 1; 0 0];
B_y_in = [0; 1/Ix];
C_y_in = [1 0];
Q_y_in = diag([10 5]); 
[Kp_y_in, Ki_y_in, Kd_y_in, Kx_y_in, Ku_y_in] = solve_pid_hinf_backstepping(A_y_in, B_y_in, C_y_in, Q_y_in, gamma_in, rho1_in, rho2_in);

% ── 3. SUBSISTEM Z (Heave - Orde 2) ──────────────────────────────────────
Az = [0 1; 0 0];
Bz = [0; 1/m];
Cz = [1 0]; 
[Kp_z, Ki_z, Kd_z, Kx_z, Ku_z] = solve_pid_hinf_backstepping(Az, Bz, Cz, diag([10 15]), gamma_in, rho1_in, rho2_in);

% ── 4. SUBSISTEM YAW (Orde 2) ────────────────────────────────────────────
Ayaw = [0 1; 0 0];
Byaw = [0; 1/Iz];
Cyaw = [1 0]; 
[Kp_yaw, Ki_yaw, Kd_yaw, Kx_yaw, Ku_yaw] = solve_pid_hinf_backstepping(Ayaw, Byaw, Cyaw, diag([10 5]), gamma_in, rho1_in, rho2_in);

%% ========================================================================
%  TAHAP 3: Display Hasil Kalkulasi Gain
% ========================================================================
fprintf('=== GAIN PID DECENTRALIZED (OPTIMAL-ROBAS H-INF BACKSTEPPING) ===\n');
fprintf('OUTER (Pos X)   -> Kp: %.4f, Ki: %.4f, Kd: %.4f\n', Kp_x_out, Ki_x_out, Kd_x_out);
fprintf('                   Kx: %s, Ku: %s\n', mat2str(Kx_x_out, 4), mat2str(Ku_x_out, 4));
fprintf('INNER (Pitch)   -> Kp: %.4f, Ki: %.4f, Kd: %.4f\n', Kp_x_in, Ki_x_in, Kd_x_in);
fprintf('                   Kx: %s, Ku: %s\n', mat2str(Kx_x_in, 4), mat2str(Ku_x_in, 4));
fprintf('OUTER (Pos Y)   -> Kp: %.4f, Ki: %.4f, Kd: %.4f\n', Kp_y_out, Ki_y_out, Kd_y_out);
fprintf('                   Kx: %s, Ku: %s\n', mat2str(Kx_y_out, 4), mat2str(Ku_y_out, 4));
fprintf('INNER (Roll)    -> Kp: %.4f, Ki: %.4f, Kd: %.4f\n', Kp_y_in, Ki_y_in, Kd_y_in);
fprintf('                   Kx: %s, Ku: %s\n', mat2str(Kx_y_in, 4), mat2str(Ku_y_in, 4));
fprintf('Z               -> Kp: %.4f, Ki: %.4f, Kd: %.4f\n', Kp_z, Ki_z, Kd_z);
fprintf('                   Kx: %s, Ku: %s\n', mat2str(Kx_z, 4), mat2str(Ku_z, 4));
fprintf('Yaw             -> Kp: %.4f, Ki: %.4f, Kd: %.4f\n', Kp_yaw, Ki_yaw, Kd_yaw);
fprintf('                   Kx: %s, Ku: %s\n\n', mat2str(Kx_yaw, 4), mat2str(Ku_yaw, 4));

%% ========================================================================
%  TAHAP 4: Eksekusi Simulink & Ekstraksi Struktur Objek 'out' 
% ========================================================================
fprintf('[INFO] Menjalankan simulasi PID-Hinf Backstepping di Simulink...\n');

xinit  = [1; 0; 1; 0; 1; 0; 0; 0; 0; 0; deg2rad(0); 0];
xref   = 1; 
yref   = 1; 
zref   = 1; 
yawref = deg2rad(10);

out = sim('quadrotor_PID_Hinf_backstepping_decentralized_simulink.slx'); 
fprintf('-> Jalur simulasi numerik sukses diselesaikan.\n');

% Ekstraksi variabel utama dari Simulink
Xs   = out.Xs;     
Us   = out.Us;     

% Ambil data referensi ASLI dari Simulink (bukan rekayasa garis lurus)
Xref_asli = out.Xref; 

% Penyelarasan dimensi waktu dan data referensi agar bebas eror dimensi
N_samples = size(Xs, 1); 
t_g       = linspace(0, out.tout(end), N_samples)'; 

% [TRIK AMAN] Selaraskan jumlah baris Xref_asli dengan Xs
if size(Xref_asli, 1) > N_samples
    Xref_asli = Xref_asli(1:N_samples, :); % Potong kelebihannya
elseif size(Xref_asli, 1) < N_samples
    selisih = N_samples - size(Xref_asli, 1);
    Xref_asli = [Xref_asli; repmat(Xref_asli(end, :), selisih, 1)]; % Duplikasi ujungnya
end

% Mapping data referensi asli Simulink
x_ref_total   = Xref_asli(:,1);
y_ref_total   = Xref_asli(:,2);
z_ref_total   = Xref_asli(:,3);
psi_ref_total = Xref_asli(:,4); % Biasanya dari Simulink masih dalam format Radian

% Struktur Xref untuk kebutuhan visualisasi gambar
Xref = [x_ref_total, y_ref_total, z_ref_total, psi_ref_total];

%% ========================================================================
%  TAHAP 5: Perhitungan Kuantitatif Metrik Evaluasi Kinerja
% ========================================================================
% Perhitungan nilai RMSE individual
rmse_x_total   = sqrt(mean((Xs(:,1) - x_ref_total).^2));
rmse_y_total   = sqrt(mean((Xs(:,3) - y_ref_total).^2));
rmse_z_total   = sqrt(mean((Xs(:,5) - z_ref_total).^2));
rmse_psi_total = sqrt(mean((rad2deg(Xs(:,11)) - rad2deg(Xref(:,4))).^2));

% Perhitungan RMSE spasial gabungan 3D (XYZ)
rmse_pos_3d_total = sqrt(mean((Xs(:,1) - x_ref_total).^2 + ...
                              (Xs(:,3) - y_ref_total).^2 + ...
                              (Xs(:,5) - z_ref_total).^2));

% Perhitungan akumulasi daya usaha kontrol integral kuadratik total (Eu)
E_u_val = trapz(t_g, sum(Us.^2, 2));

% Cetak rangkuman evaluasi metrik ke Command Window
fprintf('\n=======================================================\n');
fprintf(' HASIL EVALUASI METRIK KINERJA PID-HINF BACKSTEPPING\n');
fprintf('=======================================================\n');
fprintf('RMSE Sumbu X               = %.6f m\n', rmse_x_total);
fprintf('RMSE Sumbu Y               = %.6f m\n', rmse_y_total);
fprintf('RMSE Ketinggian Z          = %.6f m\n', rmse_z_total);
fprintf('-------------------------------------------------------\n');
fprintf('RMSE Total Posisi 3D (XYZ) = %.6f m\n', rmse_pos_3d_total);
fprintf('RMSE Total Sudut Yaw (psi) = %.6f deg\n', rmse_psi_total);
fprintf('-------------------------------------------------------\n');
fprintf('Energi Kontrol Total (Eu)  = %.4f N^2.s\n', E_u_val);
fprintf('=======================================================\n');

%% ========================================================================
%  TAHAP 6: Fungsi Render Visualisasi Grafik Terstruktur 
% ========================================================================
fprintf('\n[INFO] Merender visualisasi grafik ke layar...\n');
plot_results_pid_hinf_backstepping(t_g, Xs, Us, Xref, p);
fprintf('[SELESAI] Seluruh kurva pelacakan PID-Hinf Backstepping sukses di-plot.\n');


% =========================================================================
%  LOCAL FUNCTIONS (Khusus Penanganan Visualisasi dan Solver Backstepping)
% =========================================================================

function R = Rzyx(phi, theta, psi)
    Rx = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
    Ry = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
    Rz = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
    R  = Rz * Ry * Rx;
end

% -------------------------------------------------------------------------
function plot_results_pid_hinf_backstepping(t_vec, chi_opt, u_opt, Xref, p)

    x_ref_f   = Xref(:,1);
    y_ref_f   = Xref(:,2);
    z_ref_f   = Xref(:,3);
    psi_ref_f = rad2deg(Xref(:,4));

    %% ── Gambar 1: Animasi Mekanis 3D Real-Time dan Penjejakan Jalur ─────────
    figure('Name', 'Gambar 1: Animasi Real-Time Kinerja PID-Hinf Backstepping', ...
           'Position', [40, 40, 1450, 850]);

    subplot(3, 4, [1 2 5 6 9 10]);
    hold on; grid on; view(35, 25);
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Visualisasi Geometri Animasi Real-Time Mekanika Terbang PID-Hinf Backstepping');

    plot3(x_ref_f, y_ref_f, z_ref_f, 'r--', 'LineWidth', 1.2);
    traveled_path = plot3(chi_opt(1,1), chi_opt(1,3), chi_opt(1,5), 'b-', 'LineWidth', 1.8);

    plot3(chi_opt(1,1), chi_opt(1,3), chi_opt(1,5), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot3(x_ref_f(end), y_ref_f(end), z_ref_f(end), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

    axis equal; daspect([1 1 1]);
    axis([min(x_ref_f)-0.4 max(x_ref_f)+0.4 min(y_ref_f)-0.4 max(y_ref_f)+0.4 min(z_ref_f)-0.4 max(z_ref_f)+0.4]);

    long_arm        = plot3(0, 0, 0, '-ro', 'LineWidth', 2.5, 'MarkerSize', 6, 'MarkerFaceColor', 'r'); 
    lat_arm         = plot3(0, 0, 0, '-ko', 'LineWidth', 2.5, 'MarkerSize', 6, 'MarkerFaceColor', 'k'); 
    quadrotor_center    = plot3(0, 0, 0, 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'b');
    heading_pointer = plot3(0, 0, 0, '-g', 'LineWidth', 2.5); 

    subplot(3, 4, 3); plot(t_vec, x_ref_f, 'r--', 'LineWidth', 1.2); hold on; grid on;
    anim_x = plot(t_vec(1), chi_opt(1,1), 'b-', 'LineWidth', 1.8);
    ylabel('x [m]'); title('Posisi Translasi X'); axis([0 t_vec(end) min(x_ref_f)-0.2 max(x_ref_f)+0.2]);

    subplot(3, 4, 7); plot(t_vec, y_ref_f, 'r--', 'LineWidth', 1.2); hold on; grid on;
    anim_y = plot(t_vec(1), chi_opt(1,3), 'b-', 'LineWidth', 1.8);
    ylabel('y [m]'); title('Posisi Translasi Y'); axis([0 t_vec(end) min(y_ref_f)-0.2 max(y_ref_f)+0.2]);

    subplot(3, 4, 11); plot(t_vec, z_ref_f, 'r--', 'LineWidth', 1.2); hold on; grid on;
    anim_z = plot(t_vec(1), chi_opt(1,5), 'b-', 'LineWidth', 1.8);
    ylabel('z [m]'); xlabel('t [s]'); title('Ketinggian Vertikal Z'); axis([0 t_vec(end) min(z_ref_f)-0.2 max(z_ref_f)+0.2]);

    subplot(3, 4, 4); plot(t_vec, zeros(size(t_vec)), 'r--', 'LineWidth', 1.2); hold on; grid on;
    anim_phi = plot(t_vec(1), rad2deg(chi_opt(1,7)), 'b-', 'LineWidth', 1.8);
    ylabel('\phi [deg]'); title('Sudut Rotasi Roll (\phi)'); axis([0 t_vec(end) -5 5]);

    subplot(3, 4, 8); plot(t_vec, zeros(size(t_vec)), 'r--', 'LineWidth', 1.2); hold on; grid on;
    anim_theta = plot(t_vec(1), rad2deg(chi_opt(1,9)), 'b-', 'LineWidth', 1.8);
    ylabel('\theta [deg]'); title('Sudut Rotasi Pitch (\theta)'); axis([0 t_vec(end) -5 5]);

    subplot(3, 4, 12); plot(t_vec, psi_ref_f, 'r--', 'LineWidth', 1.2); hold on; grid on;
    anim_psi = plot(t_vec(1), rad2deg(chi_opt(1,11)), 'b-', 'LineWidth', 1.8);
    ylabel('\psi [deg]'); xlabel('t [s]'); title('Sudut Rotasi Yaw (\psi)'); axis([0 t_vec(end) min(psi_ref_f)-2 max(psi_ref_f)+2]);

    sgtitle('Gambar 1: Visualisasi Animasi Real-Time Penerbangan Kuadrotor PID-Hinf Backstepping (+ Config) dan Penjejakan Profil Status');

    skip = max(1, round(length(t_vec)/250));
    for k = 1:skip:length(t_vec)
        pos = [chi_opt(k,1); chi_opt(k,3); chi_opt(k,5)];
        Rm = Rzyx(chi_opt(k,7), chi_opt(k,9), chi_opt(k,11));
        
        m1 = pos + Rm * [p.l; 0; 0];   m2 = pos + Rm * [-p.l; 0; 0];  
        m3 = pos + Rm * [0; p.l; 0];   m4 = pos + Rm * [0; -p.l; 0];  
        head_p = pos + Rm * [1.6*p.l; 0; 0]; 
        
        set(long_arm, 'XData', [m2(1), pos(1), m1(1)], 'YData', [m2(2), pos(2), m1(2)], 'ZData', [m2(3), pos(3), m1(3)]);
        set(lat_arm, 'XData', [m4(1), pos(1), m3(1)], 'YData', [m4(2), pos(2), m3(2)], 'ZData', [m4(3), pos(3), m3(3)]);
        set(quadrotor_center, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
        set(heading_pointer, 'XData', [pos(1), head_p(1)], 'YData', [pos(2), head_p(2)], 'ZData', [pos(3), head_p(3)]);
        set(traveled_path, 'XData', chi_opt(1:k,1), 'YData', chi_opt(1:k,3), 'ZData', chi_opt(1:k,5));
        
        set(anim_x, 'XData', t_vec(1:k), 'YData', chi_opt(1:k,1));
        set(anim_y, 'XData', t_vec(1:k), 'YData', chi_opt(1:k,3));
        set(anim_z, 'XData', t_vec(1:k), 'YData', chi_opt(1:k,5));
        set(anim_phi, 'XData', t_vec(1:k), 'YData', rad2deg(chi_opt(1:k,7)));
        set(anim_theta, 'XData', t_vec(1:k), 'YData', rad2deg(chi_opt(1:k,9)));
        set(anim_psi, 'XData', t_vec(1:k), 'YData', rad2deg(chi_opt(1:k,11)));
        
        drawnow limitrate;
        pause(0.1);
    end

    %% ── Gambar 2: Analisis Komparatif Pelacakan Lintasan Berfilter ───────────
    figure('Name', 'Gambar 2: Pelacakan Komparatif Berfilter PID-Hinf Backstepping (3x3 Grid)', ...
           'Position', [50, 50, 1350, 820]);

    subplot(3,3,1); plot(t_vec, chi_opt(:,1), 'b-', 'LineWidth', 2); hold on; plot(t_vec, x_ref_f, 'r--', 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel('x [m]'); title('Pelacakan Posisi X'); legend('Aktual (Backstepping)', 'Referensi Filter'); grid on;

    subplot(3,3,2); plot(t_vec, chi_opt(:,3), 'b-', 'LineWidth', 2); hold on; plot(t_vec, y_ref_f, 'r--', 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel('y [m]'); title('Pelacakan Posisi Y'); legend('Aktual (Backstepping)', 'Referensi Filter'); grid on;

    subplot(3,3,3); plot(t_vec, chi_opt(:,5), 'b-', 'LineWidth', 2); hold on; plot(t_vec, z_ref_f, 'r--', 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel('z [m]'); title('Pelacakan Ketinggian Z'); legend('Aktual (Backstepping)', 'Referensi Filter'); grid on;

    subplot(3,3,4); plot(t_vec, rad2deg(chi_opt(:,7)), 'b-', 'LineWidth', 2); hold on; plot(t_vec, zeros(size(t_vec)), 'r--', 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel('\phi [deg]'); title('Sudut Roll (\phi)'); grid on;

    subplot(3,3,5); plot(t_vec, rad2deg(chi_opt(:,9)), 'b-', 'LineWidth', 2); hold on; plot(t_vec, zeros(size(t_vec)), 'r--', 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel('\theta [deg]'); title('Sudut Pitch (\theta)'); grid on;

    subplot(3,3,6); plot(t_vec, rad2deg(chi_opt(:,11)), 'b-', 'LineWidth', 2); hold on; plot(t_vec, psi_ref_f, 'r--', 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel('\psi [deg]'); title('Sudut Yaw (\psi)'); legend('Aktual (Backstepping)', 'Referensi Filter'); grid on;

    subplot(3,3,7); plot(t_vec, chi_opt(:,2), 'r-', 'LineWidth', 1.5); hold on; plot(t_vec, chi_opt(:,4), 'g-', 'LineWidth', 1.5); plot(t_vec, chi_opt(:,6), 'b-', 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel('Kecepatan [m/s]'); title('Profil Kecepatan Linear'); legend('v_x', 'v_y', 'v_z'); grid on;

    subplot(3,3,8); plot(t_vec, chi_opt(:,8), 'r-', 'LineWidth', 1.5); hold on; plot(t_vec, chi_opt(:,10), 'g-', 'LineWidth', 1.5); plot(t_vec, chi_opt(:,12), 'b-', 'LineWidth', 1.5);
    xlabel('t [s]'); ylabel('Laju [rad/s]'); title('Profil Kecepatan Angular'); legend('p', 'q', 'r'); grid on;

    subplot(3,3,9); plot3(chi_opt(:,1), chi_opt(:,3), chi_opt(:,5), 'b-', 'LineWidth', 2); hold on;
    plot3(chi_opt(1,1), chi_opt(1,3), chi_opt(1,5), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot3(x_ref_f(end), y_ref_f(end), z_ref_f(end), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); title('Lintasan Spasial 3D'); grid on; axis equal; view(35, 25);

    %% ── Gambar 3: Sinyal Kontrol Optimal u*(t) ──────────────────────────────
    figure('Name', 'Gambar 3: Sinyal Kontrol Optimal PID-Hinf Backstepping u^*(t)', ...
           'Position', [100, 100, 950, 680]);

    subplot(2,2,1); plot(t_vec, u_opt(:,1), 'b-', 'LineWidth', 2); yline(0, 'k--', 'LineWidth', 0.8);
    xlabel('t [s]'); ylabel('[N]'); title('U_1 — Total Thrust'); grid on;

    subplot(2,2,2); plot(t_vec, u_opt(:,2), 'r-', 'LineWidth', 2); yline(0, 'k--', 'LineWidth', 0.8);
    xlabel('t [s]'); ylabel('[N.m]'); title('U_2 — Torsi Roll'); grid on;

    subplot(2,2,3); plot(t_vec, u_opt(:,3), 'g-', 'LineWidth', 2); yline(0, 'k--', 'LineWidth', 0.8);
    xlabel('t [s]'); ylabel('[N.m]'); title('U_3 — Torsi Pitch'); grid on;

    subplot(2,2,4); plot(t_vec, u_opt(:,4), 'm-', 'LineWidth', 2); yline(0, 'k--', 'LineWidth', 0.8);
    xlabel('t [s]'); ylabel('[N.m]'); title('U_4 — Torsi Yaw'); grid on;

    %% ── Gambar 4: Evolusi Kinerja Kumulatif RMSE(t) ─────────────────────────
    figure('Name', 'Gambar 4: Evolusi Kinerja Kumulatif RMSE(t)', ...
           'Position', [150, 150, 950, 620]);

    e_x = chi_opt(:,1) - x_ref_f; e_y = chi_opt(:,3) - y_ref_f; e_z = chi_opt(:,5) - z_ref_f;
    e_psi = rad2deg(chi_opt(:,11)) - psi_ref_f;

    N_samples = length(t_vec);
    rmse_x_t   = zeros(1, N_samples); rmse_y_t   = zeros(1, N_samples);
    rmse_z_t   = zeros(1, N_samples); rmse_psi_t = zeros(1, N_samples);

    for k = 1:N_samples
        rmse_x_t(k)   = sqrt(mean(e_x(1:k).^2));
        rmse_y_t(k)   = sqrt(mean(e_y(1:k).^2));
        rmse_z_t(k)   = sqrt(mean(e_z(1:k).^2));
        rmse_psi_t(k) = sqrt(mean(e_psi(1:k).^2));
    end

    subplot(2,1,1); plot(t_vec, rmse_x_t, 'r-', 'LineWidth', 2); hold on; plot(t_vec, rmse_y_t, 'g-', 'LineWidth', 2); plot(t_vec, rmse_z_t, 'b-', 'LineWidth', 2);
    xlabel('Waktu t [s]'); ylabel('RMSE Posisi [m]'); title('Evolusi Kumulatif RMSE Posisi terhadap Waktu'); legend('RMSE X', 'RMSE Y', 'RMSE Z'); grid on;

    subplot(2,1,2); plot(t_vec, rmse_psi_t, 'm-', 'LineWidth', 2);
    xlabel('Waktu t [s]'); ylabel('RMSE Sudut Yaw (\psi) terhadap Waktu'); legend('RMSE \psi'); grid on;

end

% -------------------------------------------------------------------------
function [Kp, Ki, Kd, K_x, K_u] = solve_pid_hinf_backstepping(A, B, C, Q_val, gamma, rho1, rho2)
    % Cek syarat dari buku
    if rho2 >= 0
        error('Sesuai buku Persamaan (8.16), nilai rho2 HARUS NEGATIF (rho2 < 0)!');
    end

    n = size(A, 1);
    m = size(B, 2);
    
    % =====================================================================
    % 1. AUGMENTASI BACKSTEPPING (Persamaan 8.17)
    % Memasukkan parameter rho2 ke dalam dinamika Integrator
    % =====================================================================
    A_aug = [A,           B; 
             zeros(m, n), rho2 * eye(m)]; 
             
    Bu_aug = [zeros(n, m); eye(m)];
    Bw_aug = [B; zeros(m, m)]; % Asumsi gangguan masuk lewat jalur aktuator yang sama (B1)
    
    % =====================================================================
    % 2. MATRIKS PEMBOBOT Q DAN PENYELESAIAN RICCATI (Persamaan 8.19)
    % =====================================================================
    % Kita asumsikan D12^T * D12 pada state xi adalah kecil (misal 0.1)
    Q_aug = blkdiag(Q_val, 0.1 * eye(m)); 
    
    % Dalam Persamaan (8.19), rho1^-2 berfungsi persis seperti R^-1 pada LQR biasa.
    % Oleh karena itu ekuivalen energi R = rho1^2
    R_equiv = (rho1^2) * eye(m);
    
    % Trik manipulasi matriks untuk fungsi care()
    B_tilde = [Bu_aug, Bw_aug];
    R_tilde = [R_equiv,      zeros(m, m);
               zeros(m, m), -gamma^2 * eye(m)];
               
    [P_inf, ~, ~] = care(A_aug, B_tilde, Q_aug, R_tilde);
    
    % Gain Augmented (Ka) berdasarkan Persamaan (8.20)
    Ka = (1/rho1^2) * Bu_aug' * P_inf; 
    
    % =====================================================================
    % 3. MATRIKS TRANSFORMASI GAMMA (\Gamma)
    % =====================================================================
    Gamma_mat = [C',             (C*A)',            (C*(A^2))';
                 zeros(m, m),    (C*B)',            (C*A*B)'   ];
    
    % =====================================================================
    % 4. EKSTRAKSI PARAMETER PID TERNORMALISASI (\hat{K})
    % =====================================================================
    K_hat = pinv(Gamma_mat) * Ka'; 
    
    k1_hat_col = K_hat(1:m, :);
    k2_hat_col = K_hat(m+1:2*m, :);
    k3_hat_col = K_hat(2*m+1:3*m, :);
    
    k1_hat = k1_hat_col';
    k2_hat = k2_hat_col';
    k3_hat = k3_hat_col';
    
    % =====================================================================
    % 5. DENORMALISASI KE PID ASLI (k1, k2, k3) -> Kp, Ki, Kd
    % =====================================================================
    I = eye(m); 
    Kd = k3_hat * inv(I + C * B * k3_hat);
    c = I - Kd * C * B;
    Ki = c * k1_hat;
    Kp = c * k2_hat;
    
    % =====================================================================
    % 6. EKSTRAKSI K_x DAN K_u (FULL STATE FEEDBACK)
    % =====================================================================
    K_hat_T = K_hat'; 
    K_x = K_hat_T * [C',  A'*C',  (A')^2*C']'; 
    K_u = K_hat_T * [zeros(m, m),  B'*C',  B'*A'*C']';
end