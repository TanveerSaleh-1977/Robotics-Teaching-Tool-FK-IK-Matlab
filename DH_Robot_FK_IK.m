%% ========================================================================
%  DH ROBOT - FORWARD vs INVERSE KINEMATICS, SIDE BY SIDE
%  One self-contained MATLAB script. Run it top to bottom (press Run / F5).
% ========================================================================
%
%  WHAT IT DOES
%  ------------
%  The forward-only teaching script (DH_Robot_LiveScript.m, published as a
%  separate project) derives the DH table and the symbolic HTMs and lets you
%  drive a 2-4 DOF arm FORWARD with sliders. This script adds the other
%  direction and puts both on screen at once:
%
%      +---------------------------+   +---------------------------+
%      |  FORWARD kinematics       |   |  INVERSE kinematics       |
%      |  joints  ->  pose         |   |  pose    ->  joints       |
%      |  (sliders)                |   |  (typed x y z + RPY)      |
%      +---------------------------+   +---------------------------+
%
%  The LEFT panel is the arm you already know: drag q1..qN and read the pose
%  of the tool frame {N}. The RIGHT panel is the same arm, but it is placed by
%  the INVERSE solver: you type where you want the tool frame to be, and the
%  solver finds joint values that put it there. The target pose is drawn as a
%  dashed ghost frame, so you can see the gap between "where you asked" and
%  "where the arm can go".
%
%  WHY THIS IS A USEFUL PAIR
%  -------------------------
%  FK is a function: one pose per joint vector. IK is not. A 3-DOF arm asked
%  for a full 6-DOF pose is over-constrained, and even when a pose is
%  reachable there are usually SEVERAL joint vectors that produce it (elbow
%  up / elbow down). The right panel shows one of them, lets you step through
%  the others with < and >, and can draw any of them - or all of them - at
%  once as coloured ghosts, so the postures can be compared side by side.
%  Press "Use FK pose" to copy the left panel's pose into the target: the
%  solver must then return the left panel's joint values, which is the
%  cleanest check that the two panels agree.
%
%  HOW THE INVERSE KINEMATICS IS SOLVED
%  ------------------------------------
%  Numerically, so it works for ANY R/P mix without a closed-form derivation:
%
%     e(q) = [ p_target - p(q) ;  rotvec(R_target * R(q)') ]     (6 x 1)
%
%  and the update is the damped least-squares (Levenberg-Marquardt) step
%
%     dq = ( J' J + lambda^2 I )^-1 J' e
%
%  with J the geometric Jacobian of the chain (built from the DH frames, not
%  by finite differences). lambda is raised when a step does not reduce the
%  error and lowered when it does, which keeps the solver stable at
%  singularities (a fully stretched arm, for instance) instead of throwing a
%  huge joint step. The solver starts from 40 seeds - the current FK pose, the
%  zero pose, the middle of the range and 37 deterministic random poses - keeps
%  every seed that reaches the target, removes duplicates and sorts what is
%  left by distance from the current FK pose. So the solution you see first is
%  the one the arm would take with the least joint motion, and the < > buttons
%  step through the alternative postures.
%
%  Joint limits are respected: revolute joints turn through +-180 deg and a
%  prismatic joint travels +-stroke, exactly as the sliders do. Angles are
%  wrapped to (-180, 180] at the end.
%
%  ORIENTATION - the roll/pitch/yaw convention
%  -------------------------------------------
%  R = Rz(yaw) * Ry(pitch) * Rx(roll)      (the usual ZYX / "yaw-pitch-roll")
%
%  so yaw turns about the base z axis, pitch about y, roll about x. The target
%  and the achieved frame are both reported in this convention, so a mismatch
%  shows up as a difference in the numbers and as a tilt of the solid frame
%  away from the dashed ghost.
%
%  A 2- or 3-DOF arm cannot, in general, satisfy all six pose coordinates. When
%  the target is out of reach or over-constrained the panel says so and shows
%  the CLOSEST pose it could find, with the position and orientation error in
%  mm and degrees. Tick "Ignore orientation" to solve for position only - the
%  right choice for a planar arm whose tool angle is fixed by the position.
%
%  CONTROLS
%  --------
%   LEFT  (forward)   drag a joint slider, or type a value and press Enter.
%                     The a_i / s_i sliders change the geometry of BOTH panels.
%   RIGHT (inverse)   type x y z (m) and roll pitch yaw (deg), press Enter in
%                     any box or click "Solve IK".
%                     "Use FK pose"  copies the left panel's pose to the target.
%                     "Apply to FK"  copies the solution back to the left panel,
%                                    so both arms stand in the same pose.
%                     < >            step through the other solutions.
%                     Show all       draw every posture the solver found.
%                     1 2 3 ...      one chip per posture: tick the ones you
%                                    want on screen (coloured ghosts).
%   TOP               XY / YZ / ZX / 3D set the camera of BOTH panels; they are
%                     linked, so the two arms are always seen from one viewpoint.
%                     The mouse wheel zooms the plot under the pointer (both
%                     plots when "Link views" is ticked) and "Fit" goes back to
%                     the fitted view; drag a plot to orbit.
%                     "New robot" reopens the form to change the configuration.
%
%  EXAMPLES TO TRY
%  ---------------
%   1. Default (planar 3R): press "Use FK pose", then "Solve IK". The solver
%      returns the same angles - FK and IK agree.
%   2. Change z from 0.00 to 0.30 and solve: a planar arm cannot reach it. Read
%      the error, then tick "Ignore orientation" and solve again: the arm now
%      gets as close as it can, and the ghost frame shows the gap.
%   3. Set x = 0.45, y = 0.20, z = 0, yaw = 60 and solve: two solutions
%      (elbow up / elbow down). Step through them with < >, then tick
%      "Show all" to see both postures at once.
%   4. SCARA: joints RRPR, lengths 0.35 0.25 0.20 0, offsets 0.50 0 0 0.
%      Type a pick-and-place pose and watch q3 (the vertical slide) do the
%      work; the arm keeps its tool vertical.
%   5. Spatial 3R: joints RRR, lengths 0.40 0.30 0.20, twists 90 0 0.
%
%  REQUIREMENTS
%  ------------
%   * MATLAB R2020b or newer (developed and tested on R2024b).
%   * Robotics System Toolbox (rigidBodyTree, getTransform).
%  The inverse solver and the DH chain are written here, so no other toolbox
%  (and no Symbolic Math Toolbox) is needed.
%
%  FILE MAP
%  --------
%   section 0  environment check
%   section 1  configuration form (same layout as the forward-only script)
%   section 2  DH table and the numeric FK check
%   section 3  the window: two linked 3-D panels + controls
%   section 4  local functions: form, DH math, FK, IK, graphics, callbacks
%  ========================================================================


%% 0. Environment check
haveRobotics = ~isempty(ver('robotics'));   % Robotics System Toolbox
assert(haveRobotics, ['This script requires the Robotics System Toolbox. ' ...
    'Install it from Add-Ons > Get Add-Ons, then run again.']);
fprintf('Toolboxes OK: Robotics System Toolbox.\n');


%% 1. Configuration - asked for in a pop-up form
% The values below are the DEFAULTS: they pre-fill the form, and they are used
% as-is if you press "Use defaults" (or close the form).
%
%   jointSeq       one letter per joint, 'R' (revolute) or 'P' (prismatic)
%   linkLengths    link length of each joint in METRES (0 is allowed; at
%                  least one must be positive)
%   jointTwist_deg optional DH twist alpha_i of each joint, in DEGREES
%                  ([] = all zero = planar arm; set one to 90 for spatial)
%   baseOffsets    optional d_i offset of each joint, in METRES

defaultJointSeq    = 'RRR';            % try 'RR', 'RP', 'RPR', 'RRR', 'RRPR'
defaultLinkLengths = [0.40 0.30 0.20]; % metres, one per joint (0 is allowed)
defaultTwist_deg   = [];               % e.g. [90 0 0] for a spatial arm
defaultBaseOffsets = [];               % e.g. [0 0 0 0.50] for a raised base

% To skip the form and use the defaults above, replace the next statement with
% the commented struct below.
cfg = dhInputDialog(defaultJointSeq, defaultLinkLengths, ...
                    defaultTwist_deg, defaultBaseOffsets);
% cfg = struct('jointSeq',defaultJointSeq, ...
%              'linkLengths',defaultLinkLengths, ...
%              'twist_deg',defaultTwist_deg, ...
%              'baseOffsets',defaultBaseOffsets);

% The window is built by a function, not by straight-line script code, so the
% "New robot" button can call it again with another configuration and get a
% fresh window in the SAME MATLAB session - no restart, no re-running the file.
[DH, robot, S, fig] = dhfkLaunchApp(cfg);


%% 4. Local functions
% Everything below this line is a local function. They must stay at the very
% end of the file. The form, the IK solver, the slider timer and the callbacks
% all call them.

function [DH, robot, S, fig] = dhfkLaunchApp(cfg)
% Build the whole two-panel window for one robot configuration.
% Called once by the script, and again by dhNewRobot when you define a
% different robot from inside the window.

jointSeq       = upper(strtrim(char(cfg.jointSeq)));
linkLengths    = double(cfg.linkLengths(:))';
jointTwist_deg = double(cfg.twist_deg(:))';
baseOffsets    = double(cfg.baseOffsets(:))';
N              = numel(jointSeq);

% The form already checks this; re-check here for the "no form" path.
cfgErr = dhConfigError(jointSeq, linkLengths, jointTwist_deg, baseOffsets);
assert(isempty(cfgErr), 'Configuration error: %s', cfgErr);

if isempty(jointTwist_deg), jointTwist_deg = zeros(1,N); end
if isempty(baseOffsets),    baseOffsets    = zeros(1,N); end
jointTwist_rad = jointTwist_deg * pi/180;

% Planar means every joint axis is parallel, i.e. every twist is a multiple
% of 180 degrees (sin(alpha) == 0).
isPlanar = all(abs(sin(jointTwist_rad)) < 1e-9);
if isPlanar
    typeStr = '(planar)';
else
    typeStr = '(spatial)';
end

%% 2. DH table and the numeric cross-check
DH    = dhGenerate(jointSeq, linkLengths, jointTwist_rad, baseOffsets);
robot = dhBuildRobot(DH, linkLengths);

fprintf('\n=== DH robot: forward vs inverse kinematics ===\n');
fprintf('Robot: %d DOF, joints %s, lengths [%s] m, twists [%s] deg -> %s\n', ...
    N, jointSeq, strtrim(sprintf('%.3f ', linkLengths)), ...
    strtrim(sprintf('%g ', jointTwist_deg)), typeStr(2:end-1));
disp(table((1:N)', cellstr(jointSeq(:)), DH.alpha(:), DH.a(:), DH.d(:), ...
    DH.theta(:), DH.variable(:), ...
    'VariableNames', {'Joint','Type','alpha_rad','a_m','d_m', ...
                      'theta_rad','MovingVar'}));

% The IK panel is driven by the DH chain written in this file (dhFKchain),
% while the FK panel uses the Robotics System Toolbox. Check that the two
% descriptions of the SAME robot agree before showing them side by side.
qChk = zeros(N,1);
for i = 1:N
    if DH.isRev(i)
        qChk(i) = 0.3*i;
    else
        qChk(i) = 0.4*linkLengths(i);
    end
end
[Tc, ~] = dhFKchain(DH, qChk);
Tr      = getTransform(robot, qChk, sprintf('link%d',N));
fprintf('FK check (DH chain vs toolbox): max |difference| = %.2e\n', ...
    max(abs(Tc(:) - Tr(:))));

%% 3. The window
% Size it to the screen. The compiled application keeps the requested size,
% so it is capped rather than trusting a large monitor.
scr  = get(0,'ScreenSize');                       % [1 1 width height]
winW = min(1560, scr(3) - 60);
winH = min(980,  scr(4) - 90);
winL = max(10, min(30, scr(3) - winW - 10));
winB = max(20, scr(4) - winH - 40);
fig = figure('Name','DH robot - forward vs inverse kinematics', ...
    'NumberTitle','off','Color',[0.94 0.94 0.94],'MenuBar','none', ...
    'ToolBar','figure','Position',[winL winB winW winH]);

% Two 3-D panels. The gap between them is kept small so the two arms can be
% compared at a glance; the titles sit above each box.
axF = axes(fig,'Units','normalized', ...
    'Position',[0.030 0.475 0.455 0.415],'Box','on');
axI = axes(fig,'Units','normalized', ...
    'Position',[0.515 0.475 0.455 0.415],'Box','on');
% Mouse control is handled by this program, not by MATLAB's axes interaction
% layer: drag a plot to orbit it, roll the wheel to zoom it (dhInstallMouse).
% The built-in layer is switched off so that nothing else moves the camera and
% the two views always agree. Everything is done with view() and the axis
% limits, so no camera mode is ever forced and the plane views keep working.
try
    axF.Toolbar.Visible = 'off';
    axI.Toolbar.Visible = 'off';
catch
end
for ax = [axF axI]
    try
        ax.Interactions = [];
    catch
    end
end
dhInstallMouse(fig);

% Control panels under each plot.
pF = uipanel(fig,'Units','normalized', ...
    'Position',[0.030 0.010 0.455 0.445], ...
    'Title','FORWARD kinematics:  joints -> pose', ...
    'FontSize',11,'FontWeight','bold', ...
    'BackgroundColor',[0.96 0.96 0.96],'ForegroundColor',[0.10 0.25 0.55]);
pI = uipanel(fig,'Units','normalized', ...
    'Position',[0.515 0.010 0.455 0.445], ...
    'Title','INVERSE kinematics:  pose -> joints', ...
    'FontSize',11,'FontWeight','bold', ...
    'BackgroundColor',[0.96 0.96 0.96],'ForegroundColor',[0.55 0.28 0.05]);

% --- top strip: view buttons, new robot, view link ------------------------
vbW = 0.056; vbH = 0.042; vbY = 0.920;
viewNames = {'XY','YZ','ZX','3D'};
viewTips  = {'Look straight down the XY plane (plan view)', ...
             'Look straight at the YZ plane', ...
             'Look straight at the ZX plane', ...
             'Back to the 3-D perspective view'};
for k = 1:numel(viewNames)
    uicontrol(fig,'Style','pushbutton','Units','normalized', ...
        'Position',[0.030 + (k-1)*(vbW+0.006), vbY, vbW, vbH], ...
        'String',viewNames{k},'FontSize',9,'FontWeight','bold', ...
        'TooltipString',viewTips{k},'Tag','viewbutton', ...
        'Callback', @(~,~) dhSetPlaneView(fig, viewNames{k}));
end
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.290 vbY 0.085 vbH], ...
    'String','New robot','FontSize',9,'FontWeight','bold', ...
    'TooltipString',['Define a different robot (joint sequence, lengths, ' ...
                     'twists, offsets) without restarting the program'], ...
    'Callback', @(~,~) dhNewRobot(fig));
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[0.383 vbY 0.062 vbH], ...
    'String','Fit','FontSize',9,'FontWeight','bold', ...
    'TooltipString','Back to the fitted view (undoes a mouse-wheel zoom)', ...
    'Callback', @(~,~) dhFitButton(fig));
chkLink = uicontrol(fig,'Style','checkbox','Units','normalized', ...
    'Position',[0.455 vbY 0.095 vbH], ...
    'String','Link views','Value',0,'FontSize',9, ...
    'BackgroundColor',[0.94 0.94 0.94], ...
    'TooltipString','Keep both cameras on the same viewpoint', ...
    'Callback', @(~,~) dhLinkViews(fig));
uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.565 vbY 0.410 vbH], ...
    'String','Drag to orbit, scroll to zoom.  Type a target on the right.', ...
    'FontSize',9,'HorizontalAlignment','left', ...
    'BackgroundColor',[0.94 0.94 0.94],'ForegroundColor',[0.35 0.35 0.35]);

% --- LEFT panel: joint sliders and link-length sliders -------------------
% Same rows as the forward-only script:  [ label ][ slider ][ box ][ range ]
nRows = 2*N + 2;                       % 2 header rows + N joints + N lengths
rowH  = min(0.105, 0.720/nRows);
topY  = 0.935;                         % first row sits under the panel title

uicontrol(pF,'Style','text','Units','normalized', ...
    'Position',[0.020 topY-rowH 0.400 rowH], ...
    'String','Joint variables  (deg / mm)','FontWeight','bold','FontSize',10, ...
    'HorizontalAlignment','left','BackgroundColor',[0.96 0.96 0.96]);

qInit        = zeros(1,N);             % a slightly bent start pose
jointSliders = gobjects(1,N);
jointEdit    = gobjects(1,N);
jointRange   = gobjects(1,N);
for i = 1:N
    y = topY - (i+1)*rowH;
    if DH.isRev(i)
        qInit(i) = 25;                                  % degrees
        lab  = sprintf('q_%d  (R, deg)',i);
        rng  = '-180..180';
        lims = [-180 180];
    else
        st   = max(linkLengths(i)*1000, 0.001);         % millimetres
        lab  = sprintf('q_%d  (P, mm)',i);
        rng  = sprintf('-%.0f..%.0f', -st, st);
        lims = [-st st];
    end
    [jointSliders(i), jointEdit(i), jointRange(i)] = dhSliderRow(pF, y, rowH, ...
        lab, rng, lims(1), lims(2), qInit(i), '%.1f');
end

uicontrol(pF,'Style','text','Units','normalized', ...
    'Position',[0.020 topY-(N+2)*rowH 0.720 rowH], ...
    'String','Link lengths  (mm)   -   shared by both panels', ...
    'FontWeight','bold','FontSize',10, ...
    'HorizontalAlignment','left','BackgroundColor',[0.96 0.96 0.96]);

lengthSliders = gobjects(1,N);
lengthEdit    = gobjects(1,N);
for i = 1:N
    y = topY - (N+2+i)*rowH;
    if DH.isRev(i)
        lab = sprintf('a_%d  (mm)',i);
    else
        lab = sprintf('s_%d  stroke (mm)',i);
    end
    [lengthSliders(i), lengthEdit(i)] = dhSliderRow(pF, y, rowH, ...
        lab, '0 .. 1000', 0, 1000, linkLengths(i)*1000, '%.0f');
end

% Readout under the last row: the FK pose in one small block.
rowBot = topY - nRows*rowH;
fkRead = uicontrol(pF,'Style','text','Units','normalized', ...
    'Position',[0.020 max(0.015, rowBot-0.175) 0.960 ...
                min(0.165, max(0.100, rowBot-0.030))], ...
    'String',{''},'FontName','FixedWidth','FontSize',9, ...
    'HorizontalAlignment','left','BackgroundColor',[1 1 1], ...
    'ForegroundColor',[0.15 0.15 0.15]);

% --- RIGHT panel: the typed target pose ----------------------------------
uicontrol(pI,'Style','text','Units','normalized', ...
    'Position',[0.020 0.868 0.960 0.095], ...
    'String',{sprintf('Target pose of the end frame {%d}, in the base frame {0}',N), ...
              'dashed ghost = target     solid frame = achieved'}, ...
    'FontSize',9.5,'HorizontalAlignment','left', ...
    'BackgroundColor',[0.96 0.96 0.96],'ForegroundColor',[0.30 0.30 0.30]);

edtX     = dhField(pI, 0.020, 0.752, 'x (m)', '0.000');
edtY     = dhField(pI, 0.355, 0.752, 'y (m)', '0.000');
edtZ     = dhField(pI, 0.690, 0.752, 'z (m)', '0.000');
edtRoll  = dhField(pI, 0.020, 0.638, 'roll',  '0.0');
edtPitch = dhField(pI, 0.355, 0.638, 'pitch', '0.0');
edtYaw   = dhField(pI, 0.690, 0.638, 'yaw',   '0.0');
uicontrol(pI,'Style','text','Units','normalized', ...
    'Position',[0.945 0.638 0.045 0.100],'String','deg','FontSize',8.5, ...
    'HorizontalAlignment','left','BackgroundColor',[0.96 0.96 0.96], ...
    'ForegroundColor',[0.45 0.45 0.45]);

uicontrol(pI,'Style','pushbutton','Units','normalized', ...
    'Position',[0.020 0.518 0.240 0.090], ...
    'String','Solve IK','FontSize',10,'FontWeight','bold', ...
    'BackgroundColor',[0.85 0.92 1.00], ...
    'TooltipString','Solve for the typed pose (or just press Enter in a box)', ...
    'Callback', @(~,~) dhIKSolve(fig, false));
uicontrol(pI,'Style','pushbutton','Units','normalized', ...
    'Position',[0.280 0.518 0.300 0.090], ...
    'String','Use FK pose','FontSize',10, ...
    'TooltipString','Copy the left panel pose into the target (always reachable)', ...
    'Callback', @(~,~) dhIKUseFK(fig));
uicontrol(pI,'Style','pushbutton','Units','normalized', ...
    'Position',[0.600 0.518 0.300 0.090], ...
    'String','Apply to FK','FontSize',10, ...
    'TooltipString','Put the IK solution on the left sliders', ...
    'Callback', @(~,~) dhIKApplyFK(fig));

uicontrol(pI,'Style','pushbutton','Units','normalized', ...
    'Position',[0.020 0.410 0.070 0.085], ...
    'String','<','FontSize',11,'FontWeight','bold', ...
    'TooltipString','Previous solution', ...
    'Callback', @(~,~) dhIKStep(fig, -1));
solLabel = uicontrol(pI,'Style','text','Units','normalized', ...
    'Position',[0.100 0.410 0.200 0.085], ...
    'String','no solution yet','FontSize',9.5,'FontWeight','bold', ...
    'HorizontalAlignment','left','BackgroundColor',[0.96 0.96 0.96]);
uicontrol(pI,'Style','pushbutton','Units','normalized', ...
    'Position',[0.310 0.410 0.070 0.085], ...
    'String','>','FontSize',11,'FontWeight','bold', ...
    'TooltipString','Next solution', ...
    'Callback', @(~,~) dhIKStep(fig, +1));
chkOrient = uicontrol(pI,'Style','checkbox','Units','normalized', ...
    'Position',[0.400 0.410 0.540 0.085], ...
    'String','Ignore orientation (position only)','Value',0,'FontSize',9.5, ...
    'BackgroundColor',[0.96 0.96 0.96], ...
    'TooltipString','For arms whose tool angle is fixed by the position', ...
    'Callback', @(~,~) dhIKSolve(fig, false));

% --- the "show several postures" row -------------------------------------
% One chip per posture the solver finds (at most 8). Ticking a chip draws that
% posture as a translucent ghost; the posture selected with < > is always the
% solid arm. "Show all" ticks every chip at once.
chipCol = dhGhostColours();
chkAll = uicontrol(pI,'Style','checkbox','Units','normalized', ...
    'Position',[0.020 0.322 0.150 0.075], ...
    'String','Show all','Value',0,'FontSize',9.5, ...
    'BackgroundColor',[0.96 0.96 0.96], ...
    'TooltipString',['Draw every posture the solver found, not just the ' ...
                     'selected one'], ...
    'Callback', @(~,~) dhShowAll(fig));
solChip = gobjects(1,8);
for k = 1:8
    solChip(k) = uicontrol(pI,'Style','togglebutton','Units','normalized', ...
        'Position',[0.180+(k-1)*0.088 0.322 0.080 0.075], ...
        'String',sprintf('%d',k),'FontSize',9,'FontWeight','bold', ...
        'Value',0,'Visible','off','BackgroundColor',[0.90 0.90 0.90], ...
        'TooltipString',sprintf('Show posture %d of the solutions found',k), ...
        'Callback', @(~,~) dhChipToggle(fig, k));
end

ikHead = uicontrol(pI,'Style','text','Units','normalized', ...
    'Position',[0.020 0.222 0.960 0.090], ...
    'String','','FontSize',10.5,'FontWeight','bold', ...
    'HorizontalAlignment','left','BackgroundColor',[0.96 0.96 0.96]);
ikBody = uicontrol(pI,'Style','text','Units','normalized', ...
    'Position',[0.020 0.012 0.960 0.200], ...
    'String',{''},'FontName','FixedWidth','FontSize',9, ...
    'HorizontalAlignment','left','BackgroundColor',[1 1 1], ...
    'ForegroundColor',[0.15 0.15 0.15]);

% --- the two graphics scenes --------------------------------------------
% Every drawing object is created ONCE and only its coordinates are rewritten
% afterwards (see dhCreateScene / dhUpdateVisuals), which is what keeps the
% sliders smooth.
hold(axF,'on'); set(axF,'NextPlot','add'); view(axF,-37.5,30);
gF = dhCreateScene(axF, N);
camlight(axF,'headlight'); lighting(axF,'gouraud'); material(axF,'dull');
xlabel(axF,'X'); ylabel(axF,'Y'); zlabel(axF,'Z');
title(axF, sprintf('%d-DOF %s %s   |   FORWARD: joints -> pose', ...
    N, jointSeq, typeStr));

hold(axI,'on'); set(axI,'NextPlot','add'); view(axI,-37.5,30);
gI = dhCreateScene(axI, N);
camlight(axI,'headlight'); lighting(axI,'gouraud'); material(axI,'dull');
xlabel(axI,'X'); ylabel(axI,'Y'); zlabel(axI,'Z');
title(axI, sprintf('%d-DOF %s %s   |   INVERSE: pose -> joints', ...
    N, jointSeq, typeStr));
gt = dhCreateTarget(axI);

% --- shared state --------------------------------------------------------
reach0 = dhReach(DH, linkLengths);
S = struct( ...
    'jointSeq',jointSeq, 'jointAlpha',DH.alpha, 'baseOffsets',DH.d, ...
    'N',N, 'typeStr',typeStr, 'isRev',DH.isRev, 'isPlanar',isPlanar, ...
    'jointSliders',jointSliders, 'jointEdit',jointEdit, ...
    'jointRange',jointRange, ...
    'lengthSliders',lengthSliders, 'lengthEdit',lengthEdit, ...
    'axF',axF, 'axI',axI, 'gF',gF, 'gI',gI, 'gt',gt, ...
    'fkRead',fkRead, 'ikHead',ikHead, 'ikBody',ikBody, ...
    'edtX',edtX, 'edtY',edtY, 'edtZ',edtZ, ...
    'edtRoll',edtRoll, 'edtPitch',edtPitch, 'edtYaw',edtYaw, ...
    'chkOrient',chkOrient, 'chkLink',chkLink, 'solLabel',solLabel, ...
    'chkAll',chkAll, 'solChip',solChip, 'chipCol',chipCol, ...
    'ghost',{cell(1,8)}, 'ghostOn',false(1,8), 'showAll',false, ...
    'robot',robot, 'DH',DH, 'Tt',[], 'sols',[], 'solIdx',0, ...
    'useOrient',true, 'reach',reach0, 'reachPlot',reach0, ...
    'lastJoint',qInit, 'lastLen',linkLengths*1000, ...
    'animating',false, 'tLastRebuild',tic, 'tText',tic, ...
    'textStale',false, 'planeView','');
setappdata(fig,'dhstate',S);

for i = 1:N
    set(jointSliders(i), 'Callback', @(~,~) dhPoll(fig, true));
    set(lengthSliders(i),'Callback', @(~,~) dhLengthRelease(fig));
    set(jointEdit(i),    'Callback', @(~,~) dhJointEdit(fig,i));
    set(lengthEdit(i),   'Callback', @(~,~) dhLengthEdit(fig,i));
end
set([edtX edtY edtZ edtRoll edtPitch edtYaw], ...
    'Callback', @(~,~) dhIKSolve(fig, false));

% The timer watches the sliders while you drag and animates continuously.
S.timer = timer('ExecutionMode','fixedSpacing','Period',0.04, ...
    'BusyMode','drop','TimerFcn', @(~,~) dhPoll(fig, false));
setappdata(fig,'dhstate',S);

dhMarkViewButton(fig, '');          % the window opens in the 3-D view
set(fig,'CloseRequestFcn', @(~,~) dhClose(fig));

dhLinkViews(fig);                   % both cameras on one viewpoint
dhSyncEdits(fig);
dhRebuild(fig, false);              % geometry, view fit and the first FK draw
drawnow;                            % paint the window before the first solve
dhIKUseFK(fig);                     % start from the FK pose, then solve
start(S.timer);

fprintf('\nWindow open.  LEFT = forward kinematics, RIGHT = inverse kinematics.\n');
fprintf('IK: type a pose and press Enter, or press "Use FK pose".\n');
end


% ------------------------------------------------------------------ the form
function cfg = dhInputDialog(dSeq, dLen, dTwist, dOff)
% Modal pop-up form that asks for the robot definition. Returns a struct with
% fields jointSeq (char), linkLengths (numeric), twist_deg (numeric),
% baseOffsets (numeric). Closing the form keeps the defaults.
col = [0.96 0.96 0.96];
f = figure('Name','Robot configuration - DH generator', ...
    'NumberTitle','off','MenuBar','none','ToolBar','none','Resize','off', ...
    'WindowStyle','modal','Position',[480 340 680 380],'Color',col);

uicontrol(f,'Style','text','Position',[22 336 640 26], ...
    'String','Define the robot, then press "Build".','FontSize',12, ...
    'FontWeight','bold','HorizontalAlignment','left','BackgroundColor',col);

dhFormRow(f, 282, 'fseq', ...
    'Joint sequence   (R = revolute,  P = prismatic;  2 to 4 letters):', ...
    dSeq, col);
dhFormRow(f, 228, 'flen', ...
    'Link lengths in METRES, one per joint (0 is allowed):', ...
    strtrim(sprintf('%.3f ', dLen)), col);
dhFormRow(f, 174, 'ftwist', ...
    'Joint twists alpha in DEGREES, optional (e.g. 90 0 0):', ...
    dhNumList(dTwist), col);
dhFormRow(f, 120, 'foff', ...
    'Base offsets d in METRES, optional (e.g. 0.10 0 0):', ...
    dhNumList(dOff), col);

uicontrol(f,'Style','checkbox','Position',[24 86 620 22], ...
    'String','Make it spatial  (twist joint 1 by 90 degrees)', ...
    'Value',0,'BackgroundColor',col,'FontSize',10,'Tag','fspatial');

uicontrol(f,'Style','text','Position',[24 54 620 22],'String','', ...
    'HorizontalAlignment','left','BackgroundColor',col, ...
    'ForegroundColor',[0.80 0 0],'FontSize',10,'Tag','fmsg');

uicontrol(f,'Style','pushbutton','Position',[444 14 100 30], ...
    'String','Build','FontSize',10,'FontWeight','bold', ...
    'Callback',@(~,~) dhFormBuild(f));
uicontrol(f,'Style','pushbutton','Position',[554 14 110 30], ...
    'String','Use defaults','FontSize',10, ...
    'Callback',@(~,~) uiresume(f));

% The result lives on the root so it survives the figure being closed. It
% starts as the defaults, so closing the form simply keeps them.
setappdata(0, 'dhFormResult', struct('jointSeq',dSeq, ...
    'linkLengths',dLen(:)','twist_deg',dTwist(:)','baseOffsets',dOff(:)'));

uiwait(f);
cfg = getappdata(0,'dhFormResult');
rmappdata(0,'dhFormResult');
if isgraphics(f), delete(f); end
end


function h = dhFormRow(f, y, tag, label, value, col)
% One labelled edit box of the form; the edit box is returned.
uicontrol(f,'Style','text','Position',[24 y+26 620 20],'String',label, ...
    'HorizontalAlignment','left','BackgroundColor',col,'FontSize',10);
h = uicontrol(f,'Style','edit','Position',[24 y 620 26],'String',value, ...
    'HorizontalAlignment','left','BackgroundColor','w','FontSize',10, ...
    'Tag',tag);
end


function dhFormBuild(f)
% Validate the form and, if it is good, store the result and close the form.
seq = upper(strtrim(get(findobj(f,'Tag','fseq'),'String')));
seq = seq(~isspace(seq));
L   = dhParseList(get(findobj(f,'Tag','flen'),'String'));
tw  = dhParseList(get(findobj(f,'Tag','ftwist'),'String'));
of  = dhParseList(get(findobj(f,'Tag','foff'),'String'));

if get(findobj(f,'Tag','fspatial'),'Value') == 1
    if isempty(tw), tw = zeros(1,numel(seq)); end
    if numel(tw) == numel(seq), tw(1) = 90; end
end

err = dhConfigError(seq, L, tw, of);
if ~isempty(err)
    set(findobj(f,'Tag','fmsg'),'String',err);
    return;
end
setappdata(0,'dhFormResult', struct('jointSeq',seq,'linkLengths',L, ...
    'twist_deg',tw,'baseOffsets',of));
uiresume(f);
end


function err = dhConfigError(seq, L, tw, of)
% Shared validation of a robot definition; '' means the definition is fine.
err = '';
if isempty(seq) || ~all(ismember(seq,'RP'))
    err = 'Joint sequence may contain only R and P (e.g. RRR or RPR).';
    return;
end
N = numel(seq);
if N < 2 || N > 4
    err = sprintf('Use 2 to 4 joints (you gave %d).', N);
    return;
end
if numel(L) ~= N
    err = sprintf('Enter exactly %d link lengths, in metres.', N);
    return;
end
if any(L < 0)
    err = 'Link lengths must be zero or positive.';
    return;
end
if ~any(L > 0)
    err = 'At least one link length must be greater than zero.';
    return;
end
if ~isempty(tw) && numel(tw) ~= N
    err = sprintf('Twists: enter %d numbers, or leave the box empty.', N);
    return;
end
if ~isempty(of) && numel(of) ~= N
    err = sprintf('Base offsets: enter %d numbers, or leave the box empty.', N);
    return;
end
% For a slide, d_i IS the joint variable - there is no room for a separate
% fixed offset, and the toolbox drops one silently. Catch it here instead.
if ~isempty(of)
    badP = find(seq == 'P' & of(:)' ~= 0, 1);
    if ~isempty(badP)
        err = sprintf(['Base offset on joint %d is not used: for a prismatic ' ...
            'joint d_i is the sliding variable itself, and it travels both ' ...
            'ways (the stroke slider covers it).'], badP);
        return;
    end
end
end


function s = dhNumList(v)
% Render a numeric row as text for an edit box ('' for empty).
if isempty(v), s = ''; else, s = strtrim(sprintf('%g ', v(:)')); end
end


function v = dhParseList(str)
% Parse a user-typed list of numbers: spaces and/or commas separate them.
v = sscanf(strrep(strtrim(char(str)), ',', ' '), '%f')';
end


% --------------------------------------------------------------- the DH math
function DH = dhGenerate(jointSeq, linkLengths, twistRad, baseOffsets)
% Generate the DH parameter set from the joint sequence.
%   * Revolute  ('R'): the moving variable is theta_i; a_i is the link length.
%   * Prismatic ('P'): the moving variable is d_i; a_i = 0 and the link length
%                      is the STROKE of the slide (it travels both ways).
jointSeq    = upper(strtrim(char(jointSeq)));
N           = numel(jointSeq);
linkLengths = double(linkLengths(:))';
if isempty(twistRad) || numel(twistRad) ~= N, twistRad = zeros(1,N); end
if isempty(baseOffsets) || numel(baseOffsets) ~= N, baseOffsets = zeros(1,N); end

isRev = false(1,N);
for i = 1:N
    isRev(i) = (jointSeq(i) == 'R');
end

DH.N        = N;
DH.isRev    = isRev;
DH.alpha    = double(twistRad(:))';
DH.d        = double(baseOffsets(:))';
DH.theta    = zeros(1,N);
DH.a        = zeros(1,N);
DH.variable = cell(1,N);
for i = 1:N
    if isRev(i)
        DH.a(i)        = linkLengths(i);
        DH.variable{i} = sprintf('theta%d',i);
    else
        DH.a(i)        = 0;
        DH.variable{i} = sprintf('d%d',i);
    end
end
end


function T = dhTransform(a, alpha, d, theta)
% Standard (distal) DH homogeneous transformation matrix.
ct = cos(theta); st = sin(theta);
ca = cos(alpha); sa = sin(alpha);
T = [ct, -st*ca,  st*sa, a*ct;
     st,  ct*ca, -ct*sa, a*st;
      0,     sa,     ca,    d;
      0,      0,      0,    1];
end


function [T, Tf] = dhFKchain(DH, q)
% Forward kinematics from the DH parameters alone: the product of the N joint
% matrices, plus the frame T(0 -> i) at every joint (Tf{1} is the base frame).
% This is the chain the IK solver differentiates, and the check in section 2
% shows that it agrees with the Robotics System Toolbox.
N  = DH.N;
Tf = cell(1,N+1);
Tf{1} = eye(4);
T = eye(4);
for i = 1:N
    if DH.isRev(i)
        d  = DH.d(i);
        th = q(i);
    else
        d  = q(i);              % the slide IS d_i
        th = DH.theta(i);
    end
    T = T * dhTransform(DH.a(i), DH.alpha(i), d, th);
    Tf{i+1} = T;
end
end


function r = dhReach(DH, L)
% A simple upper bound on how far the tool frame can be from the base: every
% link length, every stroke and every fixed offset, added. Used to scale the
% plots and the damping term (never as a hard reachability test).
r = sum(abs(DH.a)) + sum(abs(L(~DH.isRev))) + sum(abs(DH.d));
if r <= 0, r = 0.1; end
end


% ------------------------------------------------------------ the IK solver
function sols = dhSolveIK(DH, L, q0, Tt, useOrient)
% Multi-start damped least squares for the pose Tt.
%
% Returns a struct array, best first. Every element has
%   q     joint vector (radians for R, metres for P)
%   pe    position error in metres
%   ae    orientation error in radians
%   cost  weighted residual used to rank the failures
%
% If at least one seed reaches the pose, ONLY the seeds that reach it are
% returned (so < > steps through genuine alternative postures). If none does,
% the three closest poses are returned and the caller reports the residual.
N = DH.N;
[lo, hi] = dhJointLimits(DH, L);
reach = dhReach(DH, L);
wc    = max(reach, 0.15);          % orientation weight: 1 rad ~ wc metres

% Deterministic seeds: the same target always gives the same set of solutions,
% which matters when a class is following along.
rs    = RandStream('mt19937ar','Seed',11);
nSeed = 40;
seeds = zeros(N, nSeed);
seeds(:,1) = min(max(q0(:), lo), hi);      % the pose already on screen
seeds(:,2) = zeros(N,1);
seeds(:,3) = (lo + hi)/2;
for k = 4:nSeed
    seeds(:,k) = lo + (hi - lo).*rand(rs, N, 1);
end

cand = struct('q',{},'pe',{},'ae',{},'cost',{});
for k = 1:nSeed
    [q, pe, ae, ce] = dhIKFrom(DH, seeds(:,k), Tt, useOrient, wc, lo, hi);
    cand(end+1) = struct('q',q,'pe',pe,'ae',ae,'cost',ce); %#ok<AGROW>
end

% "Reached" is 0.2 mm and 0.1 deg - tight enough that the readout shows 0.00.
peTol = 2e-4;
aeTol = 0.10*pi/180;
isEx  = arrayfun(@(c) c.pe <= peTol && (~useOrient || c.ae <= aeTol), cand);

idx = find(isEx);
if ~isempty(idx)
    % All of these reach the pose: show the one nearest the current FK pose
    % first (least joint motion), then the others.
    d = zeros(numel(idx),1);
    for k = 1:numel(idx)
        d(k) = dhQdist(cand(idx(k)).q, q0, DH);
    end
    [~, ord] = sort(d);
    idx = idx(ord);
else
    [~, ord] = sort([cand.cost]);
    idx = ord(1:min(3,numel(ord)));
end

% Remove duplicates (the same posture found from two seeds), cap the list.
keep = zeros(1,0);
for k = idx(:)'
    dup = false;
    for m = keep
        if dhQclose(cand(k).q, cand(m).q, DH)
            dup = true;
            break;
        end
    end
    if ~dup
        keep(end+1) = k; %#ok<AGROW>
    end
    if numel(keep) >= 8, break; end
end
sols = cand(keep);
end


function [q, pe, ae, cost] = dhIKFrom(DH, q, Tt, useOrient, wc, lo, hi)
% One damped-least-squares descent from one seed.
q = min(max(q, lo), hi);
lam = 0.05;
[T, Tf] = dhFKchain(DH, q);
[e, pe, ae] = dhTaskErr(T, Tt, useOrient, wc);
cost = norm(e);

maxIt = 100;
dqCap = 0.6*ones(DH.N,1);                       % radians
dqCap(~DH.isRev) = max(0.15*wc, 0.01);          % metres

for it = 1:maxIt
    if pe < 1e-8 && (ae < 1e-8 || ~useOrient), break; end

    J = dhJacobian(DH, Tf, useOrient, wc);
    A = J.'*J + (lam*wc)^2 * eye(DH.N);
    dq = A \ (J.'*e);

    % Limit the size of one step so a singular Jacobian cannot fling a joint
    % across its whole range in a single iteration.
    sc = max(abs(dq)./dqCap);
    if sc > 1, dq = dq/sc; end

    accepted = false;
    for a = [1 0.5 0.25 0.1]
        qn = min(max(q + a*dq, lo), hi);
        [Tn, Tfn] = dhFKchain(DH, qn);
        [en, pen, aen] = dhTaskErr(Tn, Tt, useOrient, wc);
        if norm(en) < cost - 1e-14
            q = qn; Tf = Tfn; e = en;
            pe = pen; ae = aen; cost = norm(en);
            accepted = true;
            lam = max(lam*0.6, 1e-5);          % confident: trust the step more
            break;
        end
    end
    if ~accepted
        lam = lam*3;                           % too greedy: damp harder
        if lam > 1e3, break; end
    end
end

% Revolute angles are periodic; report them inside the slider range.
for i = 1:DH.N
    if DH.isRev(i), q(i) = dhWrap(q(i)); end
end
end


function [e, pe, ae] = dhTaskErr(T, Tt, useOrient, wc)
% Pose error: position difference plus the rotation vector of R_target*R'.
% Both parts are in the base frame, which is also the frame of the Jacobian.
ep = Tt(1:3,4) - T(1:3,4);
pe = norm(ep);
rv = dhRotVec(Tt(1:3,1:3) * T(1:3,1:3).');
ae = norm(rv);
if useOrient
    e = [ep; wc*rv];
else
    e = ep;
end
end


function J = dhJacobian(DH, Tf, useOrient, wc)
% Geometric Jacobian of the chain, in the base frame, built from the DH
% frames: for a revolute joint the axis is z of the PREVIOUS frame and the
% linear part is z x (p_tool - p_joint); for a slide the axis is the linear
% part and the angular part is zero. No finite differences.
N  = DH.N;
pe = Tf{N+1}(1:3,4);
Jv = zeros(3,N);
Jw = zeros(3,N);
for i = 1:N
    z = Tf{i}(1:3,3);              % joint i turns about z of frame i-1
    p = Tf{i}(1:3,4);
    if DH.isRev(i)
        Jv(:,i) = cross(z, pe - p);
        Jw(:,i) = z;
    else
        Jv(:,i) = z;
        Jw(:,i) = [0;0;0];
    end
end
if useOrient
    J = [Jv; wc*Jw];
else
    J = Jv;
end
end


function r = dhRotVec(R)
% Rotation vector (axis * angle) of a rotation matrix, angle in [0, pi].
c   = max(-1, min(1, (trace(R) - 1)/2));
ang = acos(c);
if ang < 1e-10
    r = [0;0;0];
elseif pi - ang < 1e-4
    % Near 180 deg the skew part vanishes; recover the axis from R + I = 2 n n'.
    A = (R + eye(3))/2;
    [~, i] = max(diag(A));
    n = A(:,i);
    if norm(n) < 1e-12, n = [1;0;0]; else, n = n/norm(n); end
    r = ang*n;
else
    r = ang/(2*sin(ang)) * [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
end
end


function [lo, hi] = dhJointLimits(DH, L)
% The same limits the sliders use: +-180 deg for R, +-stroke for P.
N  = DH.N;
lo = zeros(N,1);
hi = zeros(N,1);
for i = 1:N
    if DH.isRev(i)
        lo(i) = -pi;
        hi(i) =  pi;
    else
        s = max(abs(L(i)), 0.001);
        lo(i) = -s;
        hi(i) =  s;
    end
end
end


function a = dhWrap(a)
% Wrap an angle to [-pi, pi).
a = mod(a + pi, 2*pi) - pi;
end


function d = dhQdist(q1, q2, DH)
% Joint-space distance, with revolute joints measured the short way round.
d = 0;
for i = 1:DH.N
    if DH.isRev(i)
        e = dhWrap(q1(i) - q2(i));
    else
        e = q1(i) - q2(i);
    end
    d = d + e^2;
end
d = sqrt(d);
end


function tf = dhQclose(q1, q2, DH)
% Are two joint vectors the same posture? 3 deg / 3 mm.
tf = true;
for i = 1:DH.N
    if DH.isRev(i)
        if abs(dhWrap(q1(i) - q2(i))) > 3*pi/180, tf = false; return; end
    else
        if abs(q1(i) - q2(i)) > 0.003, tf = false; return; end
    end
end
end


function R = dhRpy2R(roll, pitch, yaw)
% R = Rz(yaw) * Ry(pitch) * Rx(roll), angles in degrees (ZYX convention).
r = roll*pi/180; p = pitch*pi/180; y = yaw*pi/180;
Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
Rz = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
R  = Rz*Ry*Rx;
end


function [roll, pitch, yaw] = dhR2rpy(R)
% Inverse of dhRpy2R: the ZYX angles of R, in degrees.
pitch = asin(max(-1, min(1, -R(3,1))));
if abs(cos(pitch)) > 1e-6
    roll = atan2(R(3,2), R(3,3));
    yaw  = atan2(R(2,1), R(1,1));
else
    % gimbal lock: roll and yaw are not separable, put it all in yaw
    roll = 0;
    yaw  = atan2(-R(1,2), R(2,2));
end
roll  = roll*180/pi;
pitch = pitch*180/pi;
yaw   = yaw*180/pi;
end


% ------------------------------------------------------------ the robot model
function robot = dhBuildRobot(DH, linkLengths)
% Build a rigidBodyTree from an already generated DH set, with physical
% geometry on every link (cylinder along the link, sphere at the joint).
N           = DH.N;
linkLengths = double(linkLengths(:))';

robot = rigidBodyTree('DataFormat','column','MaxNumBodies',N+1);

posLen = linkLengths(linkLengths > 0);
if isempty(posLen)
    rLink = 0.015;
else
    rLink = max(0.005, 0.070 * min(posLen));   % slim, so joint frames stay visible
end
rJoint = 1.15 * rLink;

parentName = robot.BaseName;
for i = 1:N
    if DH.isRev(i)
        jtype  = 'revolute';
        limits = [-pi pi];
        a_i    = linkLengths(i);
    else
        jtype  = 'prismatic';
        limits = [-linkLengths(i) linkLengths(i)];   % travel both ways
        a_i    = 0;
    end
    jnt = rigidBodyJoint(sprintf('joint%d',i), jtype);
    jnt.PositionLimits = limits;
    setFixedTransform(jnt, [a_i DH.alpha(i) DH.d(i) DH.theta(i)], 'dh');

    bod = rigidBody(sprintf('link%d',i));
    bod.Joint = jnt;

    % Physical geometry. The fixed DH transform of this joint at q = 0 gives
    % the parent origin seen from this body's frame: -R'*t (constant in q).
    Tfix  = dhTransform(a_i, DH.alpha(i), DH.d(i), DH.theta(i));
    pPar  = -Tfix(1:3,1:3)' * Tfix(1:3,4);
    Llink = norm(pPar);

    if Llink > 1e-6
        addVisual(bod, 'Cylinder', [rLink Llink], spanTransform(pPar,[0;0;0]));
    end
    addVisual(bod, 'Sphere', rJoint);

    % If the NEXT joint is prismatic, this body carries its slide rail.
    if i < N && ~DH.isRev(i+1)
        stroke = linkLengths(i+1);
        if stroke > 0
            addVisual(bod, 'Cylinder', [rLink/2 2*stroke], transTransform([0;0;0]));
        end
    end

    addBody(robot, bod, parentName);
    parentName = sprintf('link%d',i);
end
end


function H = transTransform(v)
% Pure translation as a 4x4 homogeneous transform.
H = eye(4);
H(1:3,4) = v(:);
end


function H = spanTransform(p0, p1)
% Place a unit cylinder (axis along local z, centred) so it spans p0..p1.
p0 = p0(:); p1 = p1(:);
d  = p1 - p0;
L  = norm(d);
mid = (p0 + p1)/2;
if L < 1e-12
    R = eye(3);
else
    R = rotFromZ(d/L);
end
H = [R, mid; 0 0 0 1];
end


function R = rotFromZ(zhat)
% Rotation matrix that maps the z axis [0;0;1] onto the unit vector zhat.
zhat = zhat(:);
z0   = [0;0;1];
v    = cross(z0, zhat);
c    = dot(z0, zhat);
s    = norm(v);
if s < 1e-12
    if c >= 0
        R = eye(3);
    else
        R = [1 0 0; 0 -1 0; 0 0 -1];   % 180 deg about x
    end
else
    vx = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
    R  = eye(3) + vx + vx*vx*((1-c)/s^2);   % Rodrigues
end
end


% ------------------------------------------------------------- the graphics
function applyView(ax, reach, zl)
% Zoom the axes to the robot's reach so it fills the view, with equal scales.
% axis('equal') must come FIRST: it re-fits the limits, so it would undo any
% limits set before it. Setting the limits afterwards keeps the view stable.
% The camera is then re-framed on the new box with the SAME view direction: a
% camera left where it was would draw the box outside the axes rectangle.
if ~(reach > 0), reach = 1; end
lim = 1.18 * reach;      % slack so the projections and their labels fit inside
if nargin < 3 || isempty(zl), zl = [-lim lim]; end
[az, el] = view(ax);
axis(ax,'equal');
xlim(ax, [-lim lim]); ylim(ax, [-lim lim]); zlim(ax, zl);
view(ax, az, el);
grid(ax,'on');
end


function dhInstallMouse(fig)
% Install the mouse handlers: drag to orbit, wheel to zoom. The axes' own
% interaction layer is switched off, so this is the only thing that moves the
% camera - which also means the two views can be kept in step without linkprop.
set(fig, 'WindowButtonDownFcn',   @(~,~) dhMouseDown(fig), ...
         'WindowButtonMotionFcn', @(~,~) dhMouseMove(fig), ...
         'WindowButtonUpFcn',     @(~,~) dhMouseUp(fig), ...
         'WindowScrollWheelFcn',  @(~,e) dhWheelZoom(fig, e));
end


function dhMouseDown(fig)
% Start an orbit if the press landed on one of the two plots.
S = getappdata(fig,'dhstate');
if ~isvalid(fig) || isempty(S), return; end
if ~strcmp(get(fig,'SelectionType'),'normal'), return; end    % left button only
ax = ancestor(hittest(fig), 'axes');
if isempty(ax) || ~any(ax == [S.axF S.axI]), return; end
[az, el] = view(ax);
S.drag = struct('ax',ax, 'p0',get(fig,'CurrentPoint'), ...
                'az0',az, 'el0',el, 'moved',false);
setappdata(fig,'dhstate',S);
end


function dhMouseMove(fig)
% Orbit while the mouse is dragged: horizontal motion turns, vertical motion
% tilts. The other plot follows when "Link views" is ticked.
S = getappdata(fig,'dhstate');
if ~isvalid(fig) || isempty(S) || ~isfield(S,'drag') || isempty(S.drag), return; end
d  = S.drag;
p  = get(fig,'CurrentPoint');
dx = p(1) - d.p0(1);
dy = p(2) - d.p0(2);
if ~d.moved && hypot(dx,dy) < 2, return; end     % ignore a jittery click
S.drag.moved = true;
setappdata(fig,'dhstate',S);
dhOrbitTo(S, d.ax, d.az0 - 0.4*dx, d.el0 + 0.4*dy);
end


function dhMouseUp(fig)
S = getappdata(fig,'dhstate');
if ~isvalid(fig) || isempty(S), return; end
S.drag = [];
setappdata(fig,'dhstate',S);
end


function dhOrbitTo(S, ax, az, el)
% Look at the box from this direction. view() keeps the up vector consistent,
% so a top-down plane view can never end up with a degenerate camera.
el = min(max(el, -89.9), 89.9);
view(ax, az, el);
if get(S.chkLink,'Value') == 1
    if ax == S.axF, other = S.axI; else, other = S.axF; end
    view(other, az, el);
end
drawnow limitrate;
end


function dhWheelZoom(fig, e)
% Mouse-wheel zoom on the plot under the pointer. The limits are scaled about
% their centre and the camera is re-framed with view(), so the drawing grows
% inside its own box instead of the box growing out over the other plot.
S = getappdata(fig,'dhstate');
if ~isvalid(fig) || isempty(S), return; end
ax = ancestor(hittest(fig), 'axes');
if isempty(ax) || ~any(ax == [S.axF S.axI]), return; end
n = double(e.VerticalScrollCount);
if n == 0, return; end
dhZoomPair(S, ax, 0.90 ^ (-n));        % wheel up (n < 0) zooms in
end


function dhZoomPair(S, ax, f)
% Zoom one plot, and the other one too when the views are linked.
dhZoomAxes(ax, f);
if get(S.chkLink,'Value') == 1
    if ax == S.axF, other = S.axI; else, other = S.axF; end
    dhZoomAxes(other, f);
end
end


function dhZoomAxes(ax, f)
% Zoom by the factor f (< 1 zooms in): every range shrinks about its centre and
% the camera is re-framed, so axis('equal') and the box shape are preserved.
[az, el] = view(ax);
xlim(ax, dhScaleRange(xlim(ax), f));
ylim(ax, dhScaleRange(ylim(ax), f));
zlim(ax, dhScaleRange(zlim(ax), f));
view(ax, az, el);
end


function v = dhScaleRange(v, f)
% One axis range scaled by f about its centre, kept clear of a degenerate range.
c = 0.5*(v(1)+v(2));
h = 0.5*(v(2)-v(1))*f;
if ~(h > 1e-6), h = 1e-6; end
v = [c-h, c+h];
end


function zl = dhPlotZRange(robot, DH, L, isPlanar, reach)
% The z interval the plot should show.
%
% A SPATIAL arm really does sweep a cube, so it gets one. A PLANAR arm keeps
% every joint in one horizontal plane, so a thin slab lets MATLAB draw the arm
% about twice as large. The height is found by evaluating the chain at the
% ends of every prismatic stroke (at most 2^4 configurations).
N = DH.N;
if ~(reach > 0), reach = 1; end
if ~isPlanar
    zl = [-1.18 1.18] * reach;
    return;
end

zmin = 0;  zmax = 0;
idxP = find(~DH.isRev);
nP   = numel(idxP);
for m = 0:(2^nP - 1)
    q = zeros(N,1);
    for b = 1:nP
        q(idxP(b)) = L(idxP(b)) * (2*bitget(m, b) - 1);   % both ways
    end
    for k = 1:N
        zk   = getTransform(robot, q, sprintf('link%d',k));
        zmin = min(zmin, zk(3,4));
        zmax = max(zmax, zk(3,4));
    end
end

pad = max(0.25*reach, 0.05);
zl  = [zmin - pad, zmax + pad];
end


function G = dhCreateScene(ax, N)
% Create the whole scene ONCE and hand back its handles.
%
% Creating graphics objects is what costs time in MATLAB. By creating them
% exactly once, the animation only has to write coordinates into existing
% objects: the whole robot is three patches (links, pedestals, rails), one
% patch of joint spheres, three frame lines and the end marker.
[Vc, Fc] = dhCylinderMesh(16);
[Vs, Fs] = dhSphereMesh(10);
G.nv = size(Vc,1);
G.ns = size(Vs,1);
G.linkV = cell(1,N);
G.railV = cell(1,N);
G.colV  = cell(1,N);

G.grey     = [0.62 0.62 0.62];
G.frameCol = {[0.85 0.10 0.10], [0.10 0.65 0.10], [0.10 0.10 0.85]};

G.link = patch('Parent',ax,'Faces',dhTileFaces(Fc,N), ...
    'Vertices',zeros(N*G.nv,3),'FaceColor',G.grey,'EdgeColor','none', ...
    'Clipping','on');

G.col = patch('Parent',ax,'Faces',dhTileFaces(Fc,N), ...
    'Vertices',zeros(N*G.nv,3),'FaceColor',G.grey*0.82,'EdgeColor','none', ...
    'Clipping','on');

G.rail = patch('Parent',ax,'Faces',dhTileFaces(Fc,N), ...
    'Vertices',zeros(N*G.nv,3),'FaceColor',G.grey*0.92,'EdgeColor','none', ...
    'Clipping','on','Visible','off');

G.joint = patch('Parent',ax,'Faces',dhTileFaces(Fs,N+1), ...
    'Vertices',zeros((N+1)*G.ns,3),'FaceColor',[0.50 0.50 0.50], ...
    'EdgeColor','none','Clipping','on');

G.frame = gobjects(1,3);
for a = 1:3
    G.frame(a) = plot3(ax, nan(1,3*(N+1)), nan(1,3*(N+1)), nan(1,3*(N+1)), ...
        '-','Color',G.frameCol{a},'LineWidth',1.8,'Clipping','on');
end

G.marker = plot3(ax,0,0,0,'o','MarkerSize',11,'LineWidth',1.6, ...
    'MarkerFaceColor',[0.85 0.10 0.10],'MarkerEdgeColor','w','Clipping','on');

% projection lines as ONE NaN-separated object: P -> (x,y,0) -> axis feet
G.proj = plot3(ax, nan(1,9), nan(1,9), nan(1,9), ':', ...
    'Color',[0.45 0.45 0.45],'LineWidth',1.0,'Clipping','on');

G.foot = gobjects(1,3);
for a = 1:3
    G.foot(a) = plot3(ax,0,0,0,'o','LineStyle','none','MarkerSize',5, ...
        'MarkerFaceColor',G.frameCol{a},'MarkerEdgeColor','none','Clipping','on');
end
end


function gt = dhCreateTarget(ax)
% The TARGET pose, drawn as a dashed ghost frame with a diamond at its origin.
% It is created once and only moved when the target changes.
cols = {[0.85 0.10 0.10], [0.10 0.65 0.10], [0.10 0.10 0.85]};
gt.ax = gobjects(1,3);
for a = 1:3
    gt.ax(a) = plot3(ax, nan(1,2), nan(1,2), nan(1,2), '--', ...
        'Color',cols{a}, 'LineWidth',2.2, 'Clipping','on');
end
gt.marker = plot3(ax, nan, nan, nan, 'p', 'MarkerSize',13, ...
    'MarkerFaceColor',[1.00 0.80 0.20], 'MarkerEdgeColor',[0.45 0.30 0.00], ...
    'LineWidth',1.2, 'Clipping','on');
gt.label = text(ax, nan, nan, nan, 'target', 'FontSize',9, ...
    'FontWeight','bold', 'Color',[0.50 0.32 0.00], ...
    'Interpreter','none', 'Clipping','on');
gt.err = plot3(ax, nan(1,2), nan(1,2), nan(1,2), ':', ...
    'Color',[0.65 0.05 0.05], 'LineWidth',1.6, 'Clipping','on');
end


function dhUpdateTarget(gt, Tt, reach)
% Move the ghost frame to the target pose. The axes are 0.16*reach long, so
% the ghost is clearly visible without hiding the robot.
if ~(reach > 0), reach = 1; end
p = Tt(1:3,4);
L = 0.16*reach;
for a = 1:3
    v = Tt(1:3,a)*L;
    set(gt.ax(a), 'XData',[p(1) p(1)+v(1)], 'YData',[p(2) p(2)+v(2)], ...
        'ZData',[p(3) p(3)+v(3)]);
end
set(gt.marker,'XData',p(1),'YData',p(2),'ZData',p(3));
set(gt.label,'Position', p + [0.05*reach; 0.05*reach; 0.14*reach]);
end


function Ft = dhTileFaces(F, count)
% Repeat a face list `count` times, shifting the vertex indices of each copy so
% that one patch can hold `count` identical meshes.
nv = max(F(:));
m  = size(F,1);
Ft = repmat(F, count, 1) + repelem((0:count-1).' * nv, m, 1);
end


function G = dhSetGeometry(G, DH, L)
% Work out the body-frame geometry from the current link lengths. Nothing is
% created or deleted here - only vertex templates are recomputed - so dragging
% a LINK LENGTH slider is smooth as well.
N = DH.N;
posLen = L(L > 0);
if isempty(posLen)
    rLink = 0.015;
else
    rLink = max(0.005, 0.070 * min(posLen));
end

[Vc, ~] = dhCylinderMesh(16);
for i = 1:N
    if DH.isRev(i)
        a_i = L(i);
    else
        a_i = 0;
    end
    Tfix = dhTransform(a_i, DH.alpha(i), DH.d(i), DH.theta(i));
    pPar = -Tfix(1:3,1:3).' * Tfix(1:3,4);

    % d_i is the pedestal that rises along the PARENT z axis (a SCARA's
    % shoulder height); a_i is the link that runs out of the top of it.
    pTop       = Tfix(1:3,1:3).' * ([0;0;DH.d(i)] - Tfix(1:3,4));
    G.colV{i}  = dhCylBetween(Vc, 1.8*rLink, pPar,  pTop);
    G.linkV{i} = dhCylBetween(Vc, rLink,     pTop, [0;0;0]);

    % the rail of a following prismatic joint, centred on its zero position
    if i < N && ~DH.isRev(i+1) && L(i+1) > 0
        stroke = L(i+1);
        G.railV{i} = (Vc .* [rLink/2 rLink/2 2*stroke]);
    else
        G.railV{i} = [];
    end
end

G.jointV = dhSphereMesh(10) * (1.15 * rLink);
G.rLink  = rLink;
end


function V = dhCylBetween(Vc, r, p0, p1)
% A cylinder of radius r from p0 to p1, as body-frame vertices ([] if the two
% points coincide, so the slot collapses and nothing is drawn).
v  = p1(:) - p0(:);
Ln = norm(v);
if Ln < 1e-9
    V = [];
    return;
end
R = rotFromZ(v / Ln);
V = (Vc .* [r r Ln]) * R.' + ((p0(:) + p1(:))/2).';
end


function dhUpdateVisuals(G, N, Tframes, pEnd, reach)
% Move the whole scene to the configuration in Tframes. This runs on every
% animation frame, so it only writes coordinates into the few objects created
% by dhCreateScene.
if ~(reach > 0), reach = 1; end
len = 0.09 * reach;                      % arm length of the XYZ frames
nv  = G.nv;
ns  = G.ns;

% --- all links in one patch ---------------------------------------------
Vl = zeros(N*nv,3);
for i = 1:N
    pi = Tframes{i+1}(1:3,4);
    r  = (i-1)*nv + (1:nv);
    if isempty(G.linkV{i})
        Vl(r,:) = repmat(pi.', nv, 1);
    else
        Ri = Tframes{i+1}(1:3,1:3);
        Vl(r,:) = G.linkV{i}*Ri.' + pi.';
    end
end
set(G.link,'Vertices',Vl);

% --- all base columns (pedestals) in one patch --------------------------
Vk = zeros(N*nv,3);
for i = 1:N
    pi = Tframes{i+1}(1:3,4);
    r  = (i-1)*nv + (1:nv);
    if isempty(G.colV{i})
        Vk(r,:) = repmat(pi.', nv, 1);
    else
        Ri = Tframes{i+1}(1:3,1:3);
        Vk(r,:) = G.colV{i}*Ri.' + pi.';
    end
end
set(G.col,'Vertices',Vk);

% --- all slide rails in one patch ---------------------------------------
Vr = zeros(N*nv,3);
anyRail = false;
for i = 1:N
    pi = Tframes{i+1}(1:3,4);
    r  = (i-1)*nv + (1:nv);
    if isempty(G.railV{i})
        Vr(r,:) = repmat(pi.', nv, 1);
    else
        Ri = Tframes{i+1}(1:3,1:3);
        Vr(r,:) = G.railV{i}*Ri.' + pi.';
        anyRail = true;
    end
end
if anyRail
    set(G.rail,'Vertices',Vr,'Visible','on');
else
    set(G.rail,'Visible','off');
end

% --- the base joint plus one joint per body, in one patch ---------------
Vj = zeros((N+1)*ns,3);
for k = 1:N+1
    p = Tframes{k}(1:3,4);
    Vj((k-1)*ns + (1:ns), :) = G.jointV + p.';
end
set(G.joint,'Vertices',Vj);

% --- thick XYZ frame at every joint origin (one NaN-separated line per axis)
Xd = nan(3,3*(N+1)); Yd = Xd; Zd = Xd;
for k = 1:N+1
    p   = Tframes{k}(1:3,4);
    Rk  = Tframes{k}(1:3,1:3);
    idx = 3*(k-1);
    for a = 1:3
        v = Rk(:,a) * len;
        Xd(a, idx+1:idx+2) = [p(1) p(1)+v(1)];
        Yd(a, idx+1:idx+2) = [p(2) p(2)+v(2)];
        Zd(a, idx+1:idx+2) = [p(3) p(3)+v(3)];
    end
end
for a = 1:3
    set(G.frame(a), 'XData',Xd(a,:), 'YData',Yd(a,:), 'ZData',Zd(a,:));
end

% --- end marker and the projection of its coordinates onto the axes ------
set(G.marker,'XData',pEnd(1),'YData',pEnd(2),'ZData',pEnd(3));

x = pEnd(1); y = pEnd(2); z = pEnd(3);
Pf = [x y 0];
Px = [x 0 0];
Py = [0 y 0];
set(G.proj, 'XData',[x Pf(1) nan Pf(1) Px(1) nan Pf(1) Py(1)], ...
            'YData',[y Pf(2) nan Pf(2) Px(2) nan Pf(2) Py(2)], ...
            'ZData',[z Pf(3) nan Pf(3) Px(3) nan Pf(3) Py(3)]);
set(G.foot(1),'XData',Px(1),'YData',Px(2),'ZData',Px(3));
set(G.foot(2),'XData',Py(1),'YData',Py(2),'ZData',Py(3));
set(G.foot(3),'XData',Pf(1),'YData',Pf(2),'ZData',Pf(3));
end


function [V, F] = dhCylinderMesh(ns)
% Unit cylinder: radius 1, height 1, axis along +z, centred on the origin.
th = linspace(0,2*pi,ns+1).'; th(end) = [];
x = cos(th); y = sin(th);
V = [x y -0.5*ones(ns,1); x y 0.5*ones(ns,1); 0 0 -0.5; 0 0 0.5];
n = 2*ns;
F = zeros(3*ns,4);
for i = 1:ns
    j = mod(i,ns) + 1;
    F(i,:)        = [i j ns+j ns+i];
    F(ns+i,:)     = [n+1 j i i];
    F(2*ns+i,:)   = [n+2 ns+i ns+j ns+j];
end
end


function [V, F] = dhSphereMesh(n)
% Unit sphere (radius 1) as a quad mesh built on MATLAB's sphere() grid.
[X,Y,Z] = sphere(n);
V = [X(:) Y(:) Z(:)];
F = zeros(n*n,4);
for i = 1:n
    for j = 1:n
        v = (i-1)*(n+1) + j;
        F((i-1)*n+j,:) = [v v+1 v+n+2 v+n+1];
    end
end
end


% ------------------------------------------------------------- the live UI
function [sl, ed, rt] = dhSliderRow(parent, y, h, labTxt, rangeTxt, lo, hi, val, fmt)
% One [ label ][ slider ][ box ][ range ] row inside a control panel.
bg = [0.96 0.96 0.96];
uicontrol(parent,'Style','text','Units','normalized', ...
    'Position',[0.020 y 0.180 h],'String',labTxt,'FontSize',9.5, ...
    'HorizontalAlignment','left','BackgroundColor',bg);
sl = uicontrol(parent,'Style','slider','Units','normalized', ...
    'Position',[0.205 y+0.22*h 0.495 0.56*h], ...
    'Min',lo,'Max',hi,'Value',val);
ed = uicontrol(parent,'Style','edit','Units','normalized', ...
    'Position',[0.710 y+0.18*h 0.110 0.64*h], ...
    'String',sprintf(fmt,val),'BackgroundColor','w','FontSize',9.5, ...
    'HorizontalAlignment','right');
rt = uicontrol(parent,'Style','text','Units','normalized', ...
    'Position',[0.828 y 0.155 h],'String',rangeTxt,'FontSize',8, ...
    'HorizontalAlignment','left','BackgroundColor',bg, ...
    'ForegroundColor',[0.45 0.45 0.45]);
end


function ed = dhField(parent, x, y, lab, val)
% A small [ label ][ edit ] group in the IK panel. The unit is part of the
% label ("x (m)"), so nothing can collide with the edit box.
bg = [0.96 0.96 0.96];
uicontrol(parent,'Style','text','Units','normalized', ...
    'Position',[x y 0.080 0.100],'String',lab,'FontSize',9.5, ...
    'HorizontalAlignment','right','BackgroundColor',bg);
ed = uicontrol(parent,'Style','edit','Units','normalized', ...
    'Position',[x+0.085 y+0.015 0.165 0.070],'String',val, ...
    'BackgroundColor','w','FontSize',10,'HorizontalAlignment','right');
end


function [q, L] = dhReadQL(fig)
% Read the sliders and convert to SI: q in radians (revolute) or metres
% (prismatic); L in metres. The UI itself is in degrees and millimetres.
S = getappdata(fig,'dhstate');
N = S.N;
q = zeros(N,1);
L = zeros(1,N);
for i = 1:N
    if S.isRev(i)
        q(i) = S.jointSliders(i).Value * pi/180;
    else
        q(i) = S.jointSliders(i).Value / 1000;
    end
    L(i) = S.lengthSliders(i).Value / 1000;
end
end


function dhSyncEdits(fig)
% Refresh every text box so it shows the current slider value. Only boxes that
% actually changed are written: setting a uicontrol's String costs a repaint.
S = getappdata(fig,'dhstate');
if ~isvalid(fig), return; end
for i = 1:S.N
    s = sprintf('%.1f', S.jointSliders(i).Value);
    if ~strcmp(get(S.jointEdit(i),'String'), s)
        set(S.jointEdit(i),'String',s);
    end
    s = sprintf('%.0f', S.lengthSliders(i).Value);
    if ~strcmp(get(S.lengthEdit(i),'String'), s)
        set(S.lengthEdit(i),'String',s);
    end
end
end


function dhRedrawFK(fig, force)
% Forward kinematics for the current slider configuration, redraw the LEFT
% panel, and refresh its readout. Does NOT rebuild the robot.
%
% The pose is updated on EVERY call, but the text (title, projections, readout)
% is throttled to ~10 Hz unless force is true (a typed value, or a slider
% release, passes force = true).
if nargin < 2, force = false; end
S = getappdata(fig,'dhstate');
if ~isvalid(fig), return; end
N = S.N;

q = dhReadQL(fig);

Tee        = getTransform(S.robot, q, sprintf('link%d',N));
pEnd       = Tee(1:3,4);
Tframes    = cell(1,N+1);
Tframes{1} = eye(4);
for i = 1:N
    Tframes{i+1} = getTransform(S.robot, q, sprintf('link%d',i));
end

dhUpdateVisuals(S.gF, N, Tframes, pEnd, S.reachPlot);

doText = force || toc(S.tText) > 0.10;
if ~doText
    S.textStale = true;
    setappdata(fig,'dhstate',S);
    drawnow limitrate;
    return;
end
S.tText = tic;
S.textStale = false;
setappdata(fig,'dhstate',S);

set(get(S.axF,'Title'),'String', sprintf( ...
    '%d-DOF %s %s   |   FORWARD: joints -> pose', N, upper(S.jointSeq), S.typeStr));

[roll, pitch, yaw] = dhR2rpy(Tee(1:3,1:3));
lines = { ...
    sprintf('end = (%+.3f, %+.3f, %+.3f) m', pEnd), ...
    sprintf('RPY = (%+.1f, %+.1f, %+.1f) deg', roll, pitch, yaw), ...
    dhQText(S, q)};
set(S.fkRead,'String',lines);
drawnow limitrate;
end


function dhFitViews(fig)
% Put BOTH cameras on the same box, big enough for the arm AND the target, so
% the two panels are always directly comparable.
S = getappdata(fig,'dhstate');
if ~isvalid(fig) || isempty(S), return; end
[~, L] = dhReadQL(fig);
robotReach = dhReach(S.DH, L);

rp = robotReach;
if ~isempty(S.Tt)
    pn = norm(S.Tt(1:3,4));
    % include the target, but never shrink the arm to a dot for a far target
    rp = max(rp, min(pn + 0.12*robotReach, 1.8*robotReach));
end

zl = dhPlotZRange(S.robot, S.DH, L, S.isPlanar, robotReach);
if ~isempty(S.Tt) && abs(S.Tt(3,4)) <= 1.5*robotReach
    zt = S.Tt(3,4);
    zl = [min(zl(1), zt - 0.12*robotReach), ...
          max(zl(2), zt + 0.12*robotReach)];
end

S.reachPlot = rp;
S.zlimPlot  = zl;      % the frame applyView() puts on screen
setappdata(fig,'dhstate',S);
applyView(S.axF, rp, zl);
applyView(S.axI, rp, zl);
end


function dhRebuild(fig, doSolve)
% Rebuild the robot from the current link lengths, refit both views, redraw
% the FK panel and (optionally) re-solve the IK. Also keeps the prismatic
% slider ranges in step with the current stroke.
if nargin < 2, doSolve = true; end
S = getappdata(fig,'dhstate');
if ~isvalid(fig) || isempty(S), return; end

[~, L] = dhReadQL(fig);
DH    = dhGenerate(S.jointSeq, L, S.jointAlpha, S.baseOffsets);

for i = 1:S.N
    if ~S.isRev(i)
        hi = max(L(i)*1000, 0.001);          % keep Max > Min even at zero
        sl = S.jointSliders(i);
        if sl.Max ~= hi || sl.Min ~= -hi
            v = sl.Value;
            sl.Min = -hi;                    % a slide goes both ways
            sl.Max =  hi;
            sl.Value = min(max(v, sl.Min), sl.Max);
            set(S.jointRange(i),'String',sprintf('-%.0f..%.0f', hi, hi));
        end
    end
end

S.DH    = DH;
S.robot = dhBuildRobot(DH, L);
S.gF    = dhSetGeometry(S.gF, DH, L);
S.gI    = dhSetGeometry(S.gI, DH, L);
for k = 1:numel(S.ghost)
    if ~isempty(S.ghost{k}) && isgraphics(S.ghost{k}.link)
        S.ghost{k} = dhSetGeometry(S.ghost{k}, DH, L);
    end
end
S.reach = dhReach(DH, L);
setappdata(fig,'dhstate',S);

dhFitViews(fig);
dhRedrawFK(fig, true);
if doSolve
    dhIKSolve(fig, true);
end
end


function dhPoll(fig, force)
% Continuous-update poll: while a slider is dragged its Value changes, so this
% detects the change and animates the robot. Called by the timer (force false)
% and by the release callbacks of the sliders (force true).
if nargin < 2, force = false; end
if ~isvalid(fig), return; end
S = getappdata(fig,'dhstate');
if isempty(S) || S.animating, return; end

curJ = zeros(1,S.N); curL = zeros(1,S.N);
for i = 1:S.N
    curJ(i) = S.jointSliders(i).Value;
    curL(i) = S.lengthSliders(i).Value;
end
chJ = any(abs(curJ - S.lastJoint) > 1e-6);
chL = any(abs(curL - S.lastLen)   > 1e-6);

if chL
    % lengths changed: rebuild, but not more often than every 50 ms. The IK is
    % NOT re-solved here (that would stall the drag); dhLengthRelease does it
    % once the mouse is up.
    if toc(S.tLastRebuild) < 0.05 && ~force, return; end
    S.tLastRebuild = tic;
    S.lastLen = curL;  S.lastJoint = curJ;
    setappdata(fig,'dhstate',S);
    dhRebuild(fig, false);
    dhSyncEdits(fig);
elseif chJ
    S.lastJoint = curJ;
    setappdata(fig,'dhstate',S);
    dhSyncEdits(fig);
    dhRedrawFK(fig, force);
elseif force
    dhRedrawFK(fig, true);
elseif S.textStale
    S.textStale = false;
    setappdata(fig,'dhstate',S);
    dhRedrawFK(fig, true);
end
end


function dhLengthRelease(fig)
% A link-length slider was released: finalise the rebuild, then re-solve the
% IK with the new geometry.
if ~isvalid(fig), return; end
S = getappdata(fig,'dhstate');
for i = 1:S.N
    S.lastLen(i)   = S.lengthSliders(i).Value;
    S.lastJoint(i) = S.jointSliders(i).Value;
end
S.tLastRebuild = tic;
setappdata(fig,'dhstate',S);
dhRebuild(fig, true);
dhSyncEdits(fig);
end


function dhJointEdit(fig, i)
% A value was typed into a joint text box: clamp it, then animate the joint
% smoothly from its current value to the new one.
S = getappdata(fig,'dhstate');
if ~isvalid(fig), return; end
v  = str2double(get(S.jointEdit(i),'String'));
sl = S.jointSliders(i);
if isnan(v)
    dhSyncEdits(fig);
    return;
end
v = min(max(v, sl.Min), sl.Max);
from = sl.Value;

S.animating = true;
setappdata(fig,'dhstate',S);
nFrames = 15;
for k = 1:nFrames
    if ~isvalid(fig), return; end
    sl.Value = from + (v - from) * (k/nFrames);
    dhRedrawFK(fig);
end

S = getappdata(fig,'dhstate');
S.animating = false;
S.lastJoint(i) = v;
setappdata(fig,'dhstate',S);
dhRedrawFK(fig, true);
dhSyncEdits(fig);
end


function dhLengthEdit(fig, i)
% A value was typed into a link-length text box: clamp it (zero is allowed)
% and rebuild.
S = getappdata(fig,'dhstate');
if ~isvalid(fig), return; end
v  = str2double(get(S.lengthEdit(i),'String'));
sl = S.lengthSliders(i);
if isnan(v)
    dhSyncEdits(fig);
    return;
end
v = min(max(v, sl.Min), sl.Max);
sl.Value = v;
S.lastLen(i) = v;
S.tLastRebuild = tic;
setappdata(fig,'dhstate',S);
dhRebuild(fig, true);
dhSyncEdits(fig);
end


% ---------------------------------------------------------- the IK callbacks
function [Tt, ok, msg] = dhIKReadTarget(S)
% Read the six boxes and build the target transform. Never throws: a bad box
% returns ok = false with a message for the panel.
eds = {S.edtX, S.edtY, S.edtZ, S.edtRoll, S.edtPitch, S.edtYaw};
v   = zeros(1,6);
for k = 1:6
    v(k) = str2double(get(eds{k},'String'));
end
Tt  = eye(4);
ok  = false;
msg = '';
if any(~isfinite(v))
    msg = 'Enter six numbers: x y z in metres, roll pitch yaw in degrees.';
    return;
end
if any(abs(v(1:3)) > 1000)
    msg = 'The position must be given in metres (|x|,|y|,|z| <= 1000).';
    return;
end
Tt(1:3,1:3) = dhRpy2R(v(4), v(5), v(6));
Tt(1:3,4)   = v(1:3).';
ok = true;
end


function dhIKSolve(fig, silent)
% Read the target, solve, and draw the solution in the RIGHT panel.
if nargin < 2, silent = false; end
S = getappdata(fig,'dhstate');
if ~isvalid(fig) || isempty(S), return; end

[Tt, ok, msg] = dhIKReadTarget(S);
if ~ok
    set(S.ikHead,'String','Target not valid','ForegroundColor',[0.70 0.00 0.00]);
    set(S.ikBody,'String',{msg});
    return;
end

S.Tt = Tt;
S.useOrient = get(S.chkOrient,'Value') == 0;
setappdata(fig,'dhstate',S);

[q0, L] = dhReadQL(fig);
% Re-fit only when the target is outside the box on screen. Re-fitting on every
% solve would undo a zoom made with the mouse wheel; a target that IS out of
% view gets a full fit, camera angle included, so it is actually visible.
if dhOutsideView(S, Tt(1:3,4))
    dhFitButton(fig);
    S = getappdata(fig,'dhstate');
end

if ~silent
    set(S.ikHead,'String','solving ...','ForegroundColor',[0.25 0.25 0.25]);
    drawnow;
end

sols = dhSolveIK(S.DH, L, q0, Tt, S.useOrient);

S = getappdata(fig,'dhstate');
S.sols    = sols;
S.solIdx  = 1;
S.ghostOn = false(1,8);
S.ghostOn(1:numel(sols)) = S.showAll;   % "Show all" survives a new target
setappdata(fig,'dhstate',S);
dhIKDraw(fig);
end


function dhIKDraw(fig)
% Move the IK scene to the selected solution, move the target ghost, and write
% the error readout.
S = getappdata(fig,'dhstate');
if ~isvalid(fig) || isempty(S) || isempty(S.sols), return; end
sol = S.sols(S.solIdx);

[T, Tf] = dhFKchain(S.DH, sol.q);
dhUpdateVisuals(S.gI, S.N, Tf, T(1:3,4), S.reachPlot);
dhUpdateTarget(S.gt, S.Tt, S.reachPlot);

% The other postures, if their chips are ticked. They are drawn as translucent
% ghosts under the solid arm, so several postures can be compared at once.
[~, L] = dhReadQL(fig);
S = dhGhostDraw(S, L);
dhChipPaint(S);
setappdata(fig,'dhstate',S);

% The gap between "asked" and "achieved", drawn as a dotted red segment.
pA = T(1:3,4);
pT = S.Tt(1:3,4);
if norm(pT - pA) > 1e-4
    set(S.gt.err,'XData',[pA(1) pT(1)],'YData',[pA(2) pT(2)], ...
        'ZData',[pA(3) pT(3)],'Visible','on');
else
    set(S.gt.err,'Visible','off');
end

set(get(S.axI,'Title'),'String', sprintf( ...
    '%d-DOF %s %s   |   INVERSE: solution %d of %d', ...
    S.N, upper(S.jointSeq), S.typeStr, S.solIdx, numel(S.sols)));
set(S.solLabel,'String',sprintf('%d of %d', S.solIdx, numel(S.sols)));

exact = sol.pe <= 2e-4 && (~S.useOrient || sol.ae <= 0.10*pi/180);
if exact
    if S.useOrient
        headTxt = sprintf('Solution %d of %d  -  target reached', ...
            S.solIdx, numel(S.sols));
    else
        headTxt = sprintf('Solution %d of %d  -  position reached', ...
            S.solIdx, numel(S.sols));
    end
    set(S.ikHead,'String',headTxt,'ForegroundColor',[0.00 0.45 0.00]);
else
    set(S.ikHead,'String',sprintf('Closest pose %d of %d  -  target NOT reached', ...
        S.solIdx, numel(S.sols)),'ForegroundColor',[0.70 0.20 0.00]);
end

[roll, pitch, yaw] = dhR2rpy(T(1:3,1:3));
nGhost = sum(S.ghostOn(1:numel(S.sols))) - S.ghostOn(S.solIdx);
lines = { ...
    sprintf('pos err = %6.2f mm      ori err = %6.2f deg', ...
        sol.pe*1000, sol.ae*180/pi), ...
    dhQText(S, sol.q), ...
    sprintf('end = (%+.3f, %+.3f, %+.3f) m', T(1:3,4)), ...
    sprintf('RPY = (%+.1f, %+.1f, %+.1f) deg', roll, pitch, yaw)};

if ~exact
    if norm(pT) > S.reach*1.001
        lines{end+1} = sprintf( ...
            '|target| = %.2f m is beyond the arm reach (%.2f m).', ...
            norm(pT), S.reach);
    elseif sol.ae < 1e-3
        lines{end+1} = 'Out of reach: check the link lengths and the slide stroke.';
    elseif ~S.useOrient
        lines{end+1} = 'No posture reaches that point - try a nearer target.';
    else
        lines{end+1} = 'Tick "Ignore orientation" to solve for position only.';
    end
elseif numel(S.sols) > 1 && nGhost == 1
    lines{end+1} = 'The other posture is drawn as a ghost - < > picks the solid one.';
elseif numel(S.sols) > 1 && nGhost > 1
    lines{end+1} = sprintf('%d other postures drawn as ghosts - < > picks the solid one.', ...
        nGhost);
elseif numel(S.sols) > 1
    lines{end+1} = 'Press < > for the other postures (elbow up / down).';
end
set(S.ikBody,'String',lines);
end


function dhIKUseFK(fig)
% Copy the LEFT panel's pose into the six boxes and solve. The target is then
% guaranteed reachable, so the solver must return the FK joint values.
S = getappdata(fig,'dhstate');
if ~isvalid(fig), return; end
[q, ~] = dhReadQL(fig);
T = getTransform(S.robot, q, sprintf('link%d',S.N));
[roll, pitch, yaw] = dhR2rpy(T(1:3,1:3));
set(S.edtX,'String',sprintf('%.4f',T(1,4)));
set(S.edtY,'String',sprintf('%.4f',T(2,4)));
set(S.edtZ,'String',sprintf('%.4f',T(3,4)));
set(S.edtRoll,'String',sprintf('%.2f',roll));
set(S.edtPitch,'String',sprintf('%.2f',pitch));
set(S.edtYaw,'String',sprintf('%.2f',yaw));
dhIKSolve(fig, true);
end


function dhIKApplyFK(fig)
% Put the selected IK solution on the LEFT sliders, so both panels show the
% same posture - the visual proof that FK(IK(pose)) = pose.
S = getappdata(fig,'dhstate');
if ~isvalid(fig) || isempty(S.sols), return; end
sol = S.sols(S.solIdx);
for i = 1:S.N
    if S.isRev(i)
        v = sol.q(i)*180/pi;
    else
        v = sol.q(i)*1000;
    end
    sl = S.jointSliders(i);
    v  = min(max(v, sl.Min), sl.Max);
    sl.Value = v;
    S.lastJoint(i) = v;
end
setappdata(fig,'dhstate',S);
dhSyncEdits(fig);
dhRedrawFK(fig, true);
end


function dhIKStep(fig, d)
% Step through the alternative solutions found for the same target.
S = getappdata(fig,'dhstate');
if ~isvalid(fig) || isempty(S.sols), return; end
n = numel(S.sols);
S.solIdx = mod(S.solIdx - 1 + d, n) + 1;
setappdata(fig,'dhstate',S);
dhIKDraw(fig);
end


% ------------------------------------------ showing several postures at once
function c = dhGhostColours()
% One colour per alternative posture (the solver returns at most 8). Strong
% enough to tell the ghosts apart, translucent enough not to hide the solid
% arm.
c = [0.20 0.45 0.85;    % blue
     0.90 0.55 0.10;    % orange
     0.25 0.65 0.30;    % green
     0.65 0.35 0.75;    % purple
     0.10 0.65 0.70;    % teal
     0.80 0.30 0.35;    % red
     0.55 0.55 0.15;    % olive
     0.40 0.40 0.45];   % slate
end


function G = dhGhostScene(ax, N, col, DH, L)
% A copy of the robot drawn as a translucent ghost: links and joints only, no
% frame axes, no end marker, no projection lines. Created on demand (once per
% chip) and afterwards only moved, exactly like the two real scenes.
G = dhCreateScene(ax, N);
G = dhSetGeometry(G, DH, L);
set([G.link G.col G.rail G.joint], 'FaceColor',col, 'FaceAlpha',0.70, ...
    'EdgeColor','none');
set([G.frame G.marker G.proj G.foot], 'Visible','off');
for h = [G.link G.col G.rail G.joint]
    uistack(h,'bottom');                 % ghosts sit behind the solid arm
end
end


function S = dhGhostDraw(S, L)
% Draw the postures whose chip is ticked and hide the rest. The selected
% posture is never drawn as a ghost: it is the solid arm.
n = numel(S.sols);
for k = 1:8
    g = S.ghost{k};
    if k > n || k == S.solIdx || ~S.ghostOn(k)
        if ~isempty(g) && isgraphics(g.link)
            set([g.link g.col g.rail g.joint], 'Visible','off');
        end
        continue
    end
    if isempty(g) || ~isgraphics(g.link)
        g = dhGhostScene(S.axI, S.N, S.chipCol(k,:), S.DH, L);
        S.ghost{k} = g;
    end
    [T, Tf] = dhFKchain(S.DH, S.sols(k).q);
    dhUpdateVisuals(g, S.N, Tf, T(1:3,4), S.reachPlot);
    set([g.link g.col g.rail g.joint], 'Visible','on');
end
end


function dhChipPaint(S)
% One chip per posture, in the colour of its ghost. A chip is pressed when its
% posture is on screen; the selected posture is always on screen, as the solid
% arm. The whole row only appears when there is something to compare.
n = numel(S.sols);
if n >= 2, vis = 'on'; else, vis = 'off'; end
set(S.chkAll,'Visible',vis);
for k = 1:8
    c = S.solChip(k);
    if k > n || n < 2
        set(c,'Visible','off','Value',0);
        continue
    end
    on = S.ghostOn(k) || k == S.solIdx;
    set(c,'Visible','on','Value',double(on));
    if on
        set(c,'BackgroundColor',S.chipCol(k,:),'ForegroundColor',[1 1 1]);
    else
        set(c,'BackgroundColor',[0.90 0.90 0.90], ...
            'ForegroundColor',0.75*S.chipCol(k,:));
    end
    if k == S.solIdx
        set(c,'TooltipString',['This posture is the solid arm - use < > ' ...
                               'to select another']);
    else
        set(c,'TooltipString',sprintf('Show posture %d as a ghost',k));
    end
end
end


function dhShowAll(fig)
% "Show all": tick or clear every posture chip at once.
S = getappdata(fig,'dhstate');
if ~isvalid(fig) || isempty(S), return; end
on = get(S.chkAll,'Value') == 1;
n  = numel(S.sols);
S.showAll = on;
S.ghostOn = false(1,8);
S.ghostOn(1:n) = on;
setappdata(fig,'dhstate',S);
dhIKDraw(fig);
end


function dhChipToggle(fig, k)
% One chip: show or hide that posture as a ghost.
S = getappdata(fig,'dhstate');
if ~isvalid(fig) || isempty(S) || isempty(S.sols), return; end
if k == S.solIdx
    set(S.solChip(k),'Value',1);         % the solid arm cannot be switched off
    return
end
S.ghostOn(k) = get(S.solChip(k),'Value') == 1;
n = numel(S.sols);
S.showAll = all(S.ghostOn(1:n));
set(S.chkAll,'Value',double(S.showAll));
setappdata(fig,'dhstate',S);
dhIKDraw(fig);
end


function s = dhQText(S, q)
% "q1 +25.0 deg   q2 -45.0 deg ..." - units per joint, because an arm can mix
% revolute and prismatic joints.
parts = cell(1,S.N);
for i = 1:S.N
    if S.isRev(i)
        parts{i} = sprintf('q%d %+6.1f deg', i, q(i)*180/pi);
    else
        parts{i} = sprintf('q%d %+6.1f mm', i, q(i)*1000);
    end
end
s = strjoin(parts, '   ');
end


% ------------------------------------------------------------- the camera / UI
function dhSetPlaneView(fig, which)
% Camera straight down one coordinate plane, in ORTHOGRAPHIC projection so the
% drawing is a true projection of that plane. BOTH panels move together, so
% the FK and IK arms are always seen from the same side.
S = getappdata(fig,'dhstate');
if isempty(S) || ~isvalid(fig), return; end
which = upper(which);
axs   = [S.axF S.axI];

switch which
    case 'XY'
        for a = axs, view(a, 0, 90); camproj(a,'orthographic'); end
        S.planeView = 'XY';
    case 'YZ'
        for a = axs, view(a, 90, 0); camproj(a,'orthographic'); end
        S.planeView = 'YZ';
    case 'ZX'
        for a = axs, view(a, 0, 0); camproj(a,'orthographic'); end
        S.planeView = 'ZX';
    otherwise
        for a = axs, view(a, 3); camproj(a,'perspective'); end
        S.planeView = '';
end

setappdata(fig,'dhstate',S);
dhMarkViewButton(fig, S.planeView);
end


function dhFitButton(fig)
% Back to the fitted view: the same box on both panels, camera angle reset.
S = getappdata(fig,'dhstate');
if ~isvalid(fig) || isempty(S), return; end
for ax = [S.axF S.axI]
    ax.CameraViewAngleMode = 'auto';
end
dhFitViews(fig);
end


function tf = dhOutsideView(S, p)
% True when the point lies outside the FITTED frame - the box dhFitViews() puts
% on screen for this robot and target - not outside whatever the mouse has
% zoomed to. A solve therefore keeps the zoom whenever the target is already
% inside the frame, and only re-frames for a target that would be cut off.
lim = 1.18 * S.reachPlot;
if isempty(S.reachPlot) || ~(lim > 0)
    tf = true;                       % no frame known yet: fit it
    return;
end
if isfield(S,'zlimPlot') && numel(S.zlimPlot) == 2
    zl = S.zlimPlot;
else
    zl = [-lim lim];
end
tf = abs(p(1)) > lim || abs(p(2)) > lim || p(3) < zl(1) || p(3) > zl(2);
end


function dhMarkViewButton(fig, active)
% Highlight the button of the view that is on screen ('3D' while in 3-D).
bt = findall(fig, 'Tag','viewbutton');
for k = 1:numel(bt)
    isOn = strcmpi(get(bt(k),'String'), active) || ...
        (isempty(active) && strcmpi(get(bt(k),'String'),'3D'));
    if isOn
        set(bt(k), 'BackgroundColor',[0.82 0.90 1.00]);
    else
        set(bt(k), 'BackgroundColor',[0.94 0.94 0.94]);
    end
end
end


function dhLinkViews(fig)
% "Link views" is read directly by the mouse handlers (dhOrbitTo / dhZoomPair),
% so natively there is nothing to link. Ticking it copies the forward panel's
% view onto the inverse panel, so the two start from the same direction.
S = getappdata(fig,'dhstate');
if isempty(S) || ~isvalid(fig), return; end
if get(S.chkLink,'Value') == 1
    [az, el] = view(S.axF);
    view(S.axI, az, el);
    xlim(S.axI, xlim(S.axF));
    ylim(S.axI, ylim(S.axF));
    zlim(S.axI, zlim(S.axF));
end
setappdata(fig,'dhstate',S);
end


function dhNewRobot(fig)
% Ask for another robot definition and build it in place of this window.
if ~isvalid(fig), return; end
S = getappdata(fig,'dhstate');
if isempty(S), return; end

[~, L] = dhReadQL(fig);                 % current lengths, in metres

% The form opens PRE-FILLED with what is on screen now.
cfg = dhInputDialog(S.jointSeq, L, S.jointAlpha*180/pi, S.baseOffsets);

tw = cfg.twist_deg(:)';
if isempty(tw), tw = zeros(1,numel(cfg.jointSeq)); end
same = strcmp(cfg.jointSeq, S.jointSeq) && ...
       isequal(cfg.linkLengths(:)', L) && ...
       isequal(tw, S.jointAlpha(:)') && ...
       isequal(cfg.baseOffsets(:)', S.baseOffsets(:)');
if same
    return;
end

if isfield(S,'timer') && isa(S.timer,'timer') && isvalid(S.timer)
    stop(S.timer); delete(S.timer);
end
delete(fig);

dhfkLaunchApp(cfg);
end


function dhClose(fig)
% Stop the timer and close the window cleanly.
S = getappdata(fig,'dhstate');
if ~isempty(S) && isfield(S,'timer') && isa(S.timer,'timer') && isvalid(S.timer)
    stop(S.timer); delete(S.timer);
end
delete(fig);
end
