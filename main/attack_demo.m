%% =======================================================================
%  V2V COMMUNICATION-INTEGRITY DEMO
%  Cooperative highway merging: TRUSTED V2V vs. SPOOFED V2V (BSM/GPS attack)
% -------------------------------------------------------------------------
%  Story:
%   - An ego CAV ("EGO") is cruising in the right lane.
%   - A vehicle ("MERGING") is entering from the on-ramp and needs to
%     merge in front of/alongside it.
%   - The ego vehicle has NO onboard sensor on the merging car until it is
%     almost alongside (realistic: it's outside camera/radar FOV behind
%     the gore). Its only early information comes from the merging car's
%     V2V Basic Safety Message (BSM) broadcast.
%   - SCENARIO A (trusted): the BSM is accurate (plus small, legitimate
%     GPS/V2V sensor noise). The ego vehicle's IDM controller brakes
%     appropriately and a safe gap is maintained.
%   - SCENARIO B (spoofed): an attacker injects a large, structured bias
%     into the broadcast position, making the merging car appear much
%     farther away than it truly is. The ego vehicle under-brakes,
%     trusting the lie, and the TRUE gap collapses -> collision.
%
%  Both scenarios share the exact same ground-truth initial conditions and
%  the same merging-car trajectory; the ONLY thing that differs is whether
%  the broadcast channel the ego vehicle relies on is trustworthy. This is
%  the controlled "before your gated-KF defense" picture for the slides.
%
%  Outputs:
%   1) merging_attack_comparison.mp4  - side-by-side animation
%   2) merging_attack_danger.png      - the "how dangerous is it" figure
% =======================================================================
clear; close all; clc;

%% ---------------- USER SETTINGS ----------------------------------------
outputDir = fullfile(pwd, 'output', 'merging_attack_demo');
if ~exist(outputDir, 'dir'); mkdir(outputDir); end
videoFile  = fullfile(outputDir, 'merging_attack_comparison.mp4');
figureFile = fullfile(outputDir, 'merging_attack_danger.png');

dt      = 0.1;                 % simulation time step (s)
T_total = 27;                  % total simulated duration (s)
N       = round(T_total/dt)+1; % number of samples
rngSeed = 7;                   % fixed seed -> reproducible noise realization

% --- Road geometry (meters) ---
laneWidth = 3.6;            % standard highway lane width
y_main    = laneWidth/2;    % mainline (right lane) centerline
y_ramp0   = -1.5*laneWidth; % on-ramp centerline upstream of the taper
x_taper0  = 150;            % taper (gore) start - merge maneuver begins
x_taper1  = 300;            % taper (gore) end   - fully merged into lane
carLen = 4.7; carWid = 2.0; % vehicle footprint

% --- IDM parameters for the EGO car (the only reactive/controlled agent) ---
a_acc = 1.4;   % max comfortable acceleration (m/s^2)
b_dec = 2.2;   % comfortable deceleration (m/s^2)
s0    = 2.0;   % minimum standstill gap (m)
Th    = 1.2;   % desired time headway (s)
R0    = 5.0;   % effective length + min gap used in the gap term (m)

% --- Initial conditions ---
ic.x1_0 = 0;   ic.v1_0 = 30; ic.vd1 = 30;                       % EGO
ic.x2_0 = 60;  ic.v2_0 = 20; ic.a2_const = 1.0; ic.v2_max = 26; % MERGING (fixed kinematic profile,
                                                                 % independent of ego -> clean A/B test)
% --- Attack model: BSM/GPS position spoofing on the merging vehicle ---
ic.attack_start = 1.0;   % s, spoofing begins as soon as BSM is received
ic.attack_ramp  = 1.5;   % s, ramp-up duration (sudden-onset spoof)
ic.bias_max     = 100;   % m, steady-state spoofed position offset (the "lie").
                          % Try smaller values (e.g. 30-60) to show a
                          % "dangerously tight but not fatal" near-miss
                          % instead of an outright collision.

% --- Legitimate V2V/GPS noise -> matches this project's variance_X/variance_V
%     convention in Car.m, so "noise" and "attack" are on a directly
%     comparable, physically-grounded scale for the committee. ---
ic.noise_std_X = sqrt(14.27141916);
ic.noise_std_V = sqrt(0.01);

collision_dist = 4.0;  % m, true Euclidean gap below this = physical contact

%% ---------------- RUN BOTH SCENARIOS ------------------------------------
fprintf('Simulating TRUSTED V2V scenario...\n');
Trust = simulate_merge(false, dt, N, y_main, y_ramp0, x_taper0, x_taper1, ...
                        a_acc, b_dec, s0, Th, R0, ic, rngSeed, collision_dist);

fprintf('Simulating SPOOFED V2V scenario...\n');
Spoof = simulate_merge(true, dt, N, y_main, y_ramp0, x_taper0, x_taper1, ...
                        a_acc, b_dec, s0, Th, R0, ic, rngSeed, collision_dist);

fprintf('\n--- Summary ---\n');
fprintf('Trusted V2V : min true gap = %5.1f m | collision: %s\n', ...
    min(Trust.truegap), collision_str(Trust.collide_t));
fprintf('Spoofed V2V : min true gap = %5.1f m | collision: %s\n', ...
    min(Spoof.truegap), collision_str(Spoof.collide_t));
fprintf('Attack bias = %.0f m  vs.  legitimate noise std = %.1f m  (%.0fx larger)\n\n', ...
    ic.bias_max, ic.noise_std_X, ic.bias_max/ic.noise_std_X);

%% ---------------- ANIMATION (side-by-side) ------------------------------
fprintf('Rendering animation -> %s\n', videoFile);

vw = VideoWriter(videoFile, 'MPEG-4');   % on Linux/older MATLAB use 'Motion JPEG AVI' instead
vw.FrameRate = 1/dt;                     % 1 video-second == 1 simulated second
vw.Quality   = 95;
open(vw);

fig = figure('Color','w','Position',[50 50 1500 800]);
axT = subplot(2,1,1);
axS = subplot(2,1,2);
camHalf = 55; % meters shown on each side of the focus point

for k = 1:N
    t = (k-1)*dt;

    %% --- TOP PANEL: trusted V2V ---
    cla(axT); hold(axT,'on');
    focusT = (Trust.x1(k) + Trust.x2(k))/2;
    winT = [focusT-camHalf-15, focusT+camHalf+15];
    draw_road_scene(axT, winT, laneWidth, x_taper0, x_taper1, y_ramp0);

    hdgT = taper_heading(Trust.x2(k), x_taper0, x_taper1, y_ramp0, y_main);
    draw_car(axT, Trust.x1(k), y_main, 0, carLen, carWid, [0.15 0.35 0.75], 'EGO (CAV)');
    draw_car(axT, Trust.x2(k), Trust.y2(k), hdgT, carLen, carWid, [0.15 0.65 0.25], 'MERGING');
    plot(axT, [Trust.x1(k) Trust.x2(k)], [y_main Trust.y2(k)], 'g-', 'LineWidth',1.4);

    set(axT,'DataAspectRatio',[1 1 1],'XTick',[],'YTick',[]);
    xlim(axT, [focusT-camHalf, focusT+camHalf]); ylim(axT, [-9 9]);
    title(axT, sprintf('TRUSTED V2V  (t = %4.1f s)', t), 'FontWeight','bold');
    kClamp = min(k, N-1);
    text(axT, focusT-camHalf+3, 7.3, sprintf( ...
        'true gap %5.1f m  |  perceived gap %5.1f m  |  v_{ego} %4.1f m/s', ...
        Trust.truegap(kClamp), Trust.Sperc(kClamp)+R0, Trust.v1(k)), ...
        'FontSize',9,'BackgroundColor','w','Margin',2);

    %% --- BOTTOM PANEL: spoofed V2V (freeze on collision) ---
    kk = k; frozen = false;
    if Spoof.collide_k > 0 && k >= Spoof.collide_k
        kk = Spoof.collide_k; frozen = true;
    end
    cla(axS); hold(axS,'on');
    focusS = (Spoof.x1(kk) + Spoof.x2(kk))/2;
    winS = [focusS-camHalf-15, focusS+camHalf+15];
    draw_road_scene(axS, winS, laneWidth, x_taper0, x_taper1, y_ramp0);

    hdgS = taper_heading(Spoof.x2(kk), x_taper0, x_taper1, y_ramp0, y_main);
    draw_car(axS, Spoof.x1(kk), y_main, 0, carLen, carWid, [0.15 0.35 0.75], 'EGO (CAV)');
    draw_car(axS, Spoof.x2(kk), Spoof.y2(kk), hdgS, carLen, carWid, [0.15 0.65 0.25], 'MERGING');

    if t > ic.attack_start
        ramp = min(1, (t-ic.attack_start)/ic.attack_ramp);
        bx2Ghost = Spoof.x2(kk) + ic.bias_max*ramp;
        patch(axS, bx2Ghost+[carLen/2 -carLen/2 -carLen/2 carLen/2], ...
                   y_main+[carWid/2 carWid/2 -carWid/2 -carWid/2], ...
                   [0.85 0.3 0.3], 'FaceAlpha',0.30, 'LineStyle','--', 'EdgeColor',[0.6 0 0]);
        plot(axS, [Spoof.x1(kk) bx2Ghost], [y_main y_main], 'r--', 'LineWidth',1.5);
        text(axS, bx2Ghost, y_main+2.8, 'SPOOFED BSM', 'Color',[0.7 0 0], ...
            'FontWeight','bold','FontSize',8,'HorizontalAlignment','center');
    else
        plot(axS, [Spoof.x1(kk) Spoof.x2(kk)], [y_main Spoof.y2(kk)], 'g-', 'LineWidth',1.4);
    end

    set(axS,'DataAspectRatio',[1 1 1],'XTick',[],'YTick',[]);
    xlim(axS, [focusS-camHalf, focusS+camHalf]); ylim(axS, [-9 9]);
    if frozen
        title(axS, sprintf('SPOOFED V2V  (t = %4.1f s)  --  COLLISION at t = %.1f s', t, Spoof.collide_t), ...
            'FontWeight','bold','Color',[0.6 0 0]);
        text(axS, Spoof.x1(kk), y_main+4.5, 'COLLISION', 'Color','r', ...
            'FontWeight','bold','FontSize',16,'HorizontalAlignment','center');
    else
        title(axS, sprintf('SPOOFED V2V  (t = %4.1f s)', t), 'FontWeight','bold');
    end
    kkClamp = min(kk, N-1);
    text(axS, focusS-camHalf+3, 7.3, sprintf( ...
        'true gap %5.1f m  |  perceived gap %5.1f m  <- LIE  |  v_{ego} %4.1f m/s', ...
        Spoof.truegap(kkClamp), Spoof.Sperc(kkClamp)+R0, Spoof.v1(kk)), ...
        'FontSize',9,'BackgroundColor',[1 0.92 0.92],'Margin',2);

    drawnow;
    writeVideo(vw, getframe(fig));

    if mod(k,50)==0
        fprintf('  frame %d / %d\n', k, N);
    end
end
close(vw);
fprintf('Video saved.\n\n');

%% ---------------- "HOW DANGEROUS IS IT?" SUMMARY FIGURE -----------------
fprintf('Rendering danger comparison figure -> %s\n', figureFile);

tvec = (0:N-1)*dt;
legitStd = ic.noise_std_X;

figDanger = figure('Color','w','Position',[50 50 1100 750]);

% --- Panel 1: true gap, trusted vs spoofed ---
subplot(2,1,1); hold on;
plot(tvec, Trust.truegap, 'Color',[0.15 0.55 0.15], 'LineWidth',2);
plot(tvec, Spoof.truegap, 'Color',[0.75 0.10 0.10], 'LineWidth',2);
plot([tvec(1) tvec(end)], [collision_dist collision_dist], 'k--', 'LineWidth',1.2);
if Spoof.collide_t > 0
    plot(Spoof.collide_t, collision_dist, 'rx', 'MarkerSize',12, 'LineWidth',2.5);
    text(Spoof.collide_t+0.4, collision_dist+6, sprintf('COLLISION  t = %.1f s', Spoof.collide_t), ...
        'Color',[0.6 0 0], 'FontWeight','bold', 'FontSize',10);
end
xlabel('Time (s)'); ylabel('True inter-vehicle distance (m)');
title('How dangerous is it? True gap under trusted vs. spoofed V2V', 'FontWeight','bold');
legend('Trusted V2V (accurate BSM)', sprintf('Spoofed V2V (attacker bias = %.0f m)', ic.bias_max), ...
       'Physical contact threshold', 'Location','northeast');
grid on; ylim([0, max(Trust.truegap)*1.15]);

% --- Panel 2: the deception itself (spoofed run) ---
subplot(2,1,2); hold on;
perceivedGap = Spoof.Sperc + R0;
fill([tvec fliplr(tvec)], ...
     [Spoof.truegap+3*legitStd, fliplr(max(Spoof.truegap-3*legitStd,0))], ...
     [0.85 0.85 0.85], 'EdgeColor','none', 'FaceAlpha',0.7);
plot(tvec, perceivedGap, 'b-', 'LineWidth',1.8);
plot(tvec, Spoof.truegap, 'Color',[0.75 0.10 0.10], 'LineWidth',1.8);
xlabel('Time (s)'); ylabel('Distance (m)');
title('The deception: what the ego vehicle believed vs. the truth (spoofed run)', 'FontWeight','bold');
legend('\pm3\sigma legitimate GPS/V2V noise band', 'Perceived gap (broadcast-based)', ...
       'True gap', 'Location','northeast');
grid on;

print(figDanger, figureFile, '-dpng', '-r150');
fprintf('Figure saved.\n');

%% =========================================================================
%  LOCAL FUNCTIONS
%% =========================================================================
function out = simulate_merge(use_attack, dt, N, y_main, y_ramp0, xt0, xt1, ...
                                a_acc, b_dec, s0, Th, R0, ic, seed, collision_dist)
    % Simulates one run. The EGO car (1) is the only reactive agent: it
    % runs IDM against whatever state it RECEIVES for the merging car (2)
    % over V2V. The merging car follows a fixed kinematic profile that is
    % IDENTICAL in both the trusted and spoofed runs, so the only
    % experimental variable is the integrity of the broadcast channel.
    rng(seed);
    x1 = zeros(1,N); v1 = zeros(1,N);
    x2 = zeros(1,N); v2 = zeros(1,N); y2 = zeros(1,N);
    truegap = zeros(1,N); Sperc = zeros(1,N); a1log = zeros(1,N);

    x1(1) = ic.x1_0; v1(1) = ic.v1_0; vd1 = ic.vd1;
    x2(1) = ic.x2_0; v2(1) = ic.v2_0;
    collide_t = -1; collide_k = -1;

    for k = 1:N-1
        t = (k-1)*dt;

        if v2(k) < ic.v2_max, a2 = ic.a2_const; else, a2 = 0; end
        y2(k) = taper_y(x2(k), xt0, xt1, y_ramp0, y_main);

        % --- what the ego vehicle RECEIVES over V2V ---
        % legitimate sensor/comm noise is always present...
        bx2 = x2(k) + ic.noise_std_X*randn();
        bv2 = v2(k) + ic.noise_std_V*randn();
        % ...and the attack adds a large structured bias on top of it
        if use_attack && t > ic.attack_start
            rampFrac = min(1, (t-ic.attack_start)/ic.attack_ramp);
            bx2 = bx2 + ic.bias_max*rampFrac;
        end

        % --- EGO's IDM reaction to the (possibly corrupted) leader state ---
        S = bx2 - x1(k) - R0;
        if S < 0.95, S = 0.95; end
        dV = v1(k) - bv2;
        Ss = s0 + v1(k)*Th + v1(k)*dV/(2*sqrt(a_acc*b_dec));
        f  = a_acc*(1 - (v1(k)/vd1)^4 - (Ss/S)^2);
        a1 = 8*tanh(f/8);
        a1log(k) = a1; Sperc(k) = S;

        % --- integrate true physics ---
        x1(k+1) = x1(k) + v1(k)*dt + 0.5*a1*dt^2;
        v1(k+1) = max(0, v1(k) + a1*dt);
        x2(k+1) = x2(k) + v2(k)*dt + 0.5*a2*dt^2;
        v2(k+1) = min(ic.v2_max, max(0, v2(k) + a2*dt));
        y2(k+1) = taper_y(x2(k+1), xt0, xt1, y_ramp0, y_main);

        truegap(k) = sqrt((x1(k)-x2(k))^2 + (y_main-y2(k))^2);
        if truegap(k) < collision_dist && collide_t < 0 && x2(k) > xt0
            collide_t = t; collide_k = k;
        end
    end
    truegap(N) = sqrt((x1(N)-x2(N))^2 + (y_main-y2(N))^2);
    Sperc(N) = Sperc(N-1);

    out.x1 = x1; out.v1 = v1; out.x2 = x2; out.v2 = v2; out.y2 = y2;
    out.truegap = truegap; out.Sperc = Sperc; out.a1 = a1log;
    out.collide_t = collide_t; out.collide_k = collide_k;
end

function y = taper_y(x, x0, x1, y0, y1)
    % Lateral position as a function of longitudinal position: constant
    % y0 before the gore, linear taper through it, constant y1 (merged)
    % after it. Used for both the merging car AND the ramp pavement edge.
    if x <= x0
        y = y0;
    elseif x >= x1
        y = y1;
    else
        y = y0 + (y1-y0)*(x-x0)/(x1-x0);
    end
end

function h = taper_heading(x, x0, x1, y0, y1)
    if x <= x0 || x >= x1
        h = 0;
    else
        h = atan2(y1-y0, x1-x0);
    end
end

function draw_road_scene(ax, xlimRange, laneWidth, x0, x1, y_ramp0)
    % Mainline pavement (2 lanes)
    patch(ax, [xlimRange(1) xlimRange(2) xlimRange(2) xlimRange(1)], ...
              [0 0 2*laneWidth 2*laneWidth], [0.55 0.55 0.58], 'EdgeColor','none');

    % Ramp pavement: constant width upstream, tapers to zero at the gore
    nseg = 80;
    xs2 = linspace(xlimRange(1), min(x1,xlimRange(2)), nseg);
    haveRamp = xs2(end) > xs2(1);
    if haveRamp
        bottomY = arrayfun(@(xx) taper_y(xx, x0, x1, y_ramp0-laneWidth/2, 0), xs2);
        topY = zeros(size(xs2));
        patch(ax, [xs2 fliplr(xs2)], [topY fliplr(bottomY)], [0.55 0.55 0.58], 'EdgeColor','none');
    end

    % Lane markings (drawn on top so they stay visible)
    xs = floor(xlimRange(1)/16)*16 : 8 : ceil(xlimRange(2)/16)*16;
    for i = 1:2:length(xs)-1
        line(ax, [xs(i) min(xs(i)+5,xlimRange(2))], [laneWidth laneWidth], 'Color','w','LineWidth',1.5);
    end
    line(ax, xlimRange, [2*laneWidth 2*laneWidth], 'Color','w', 'LineWidth',2.2);  % left road edge
    if haveRamp
        line(ax, xs2, bottomY, 'Color',[0.95 0.75 0.10], 'LineWidth',2.2);          % outer ramp edge
    end
    xGore = [xlimRange(1), min(x0,xlimRange(2))];
    if xGore(2) > xGore(1)
        line(ax, xGore, [0 0], 'Color','w', 'LineWidth',1.8);                       % gore line
    end

    set(ax,'Color',[0.78 0.86 0.72]); % grass background
end

function draw_car(ax, x, y, headingRad, L, W, faceColor, labelStr)
    corners = [ L/2  W/2; -L/2  W/2; -L/2 -W/2;  L/2 -W/2 ]';
    Rm = [cos(headingRad) -sin(headingRad); sin(headingRad) cos(headingRad)];
    rotated = Rm*corners;
    patch(ax, rotated(1,:)+x, rotated(2,:)+y, faceColor, 'EdgeColor',[0 0 0], 'LineWidth',1);
    if ~isempty(labelStr)
        text(ax, x, y-3.4, labelStr, 'FontSize',8, 'HorizontalAlignment','center', 'FontWeight','bold');
    end
end

function s = collision_str(ct)
    if ct > 0
        s = sprintf('YES, at t = %.1f s', ct);
    else
        s = 'no';
    end
end