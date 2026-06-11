classdef uSTOL_Demonstrator_AERO_Params
% AeroParams  Aerodynamic-coefficient parameter set for the uSTOL Demonstrator.
%
%   Builds a nested struct (obj.geom_ad) grouped by component, injected into
%   the aero model alongside the aircraft geometry:
%
%       ac   = uSTOL_Demonstrator_Aircraft_Params();
%       ad   = uSTOL_Demonstrator_AERO_Params();
%       aero = uSTOL_Demonstrator_AERO_Model(ac.geom_ac, ad.geom_ad);
%
%   Access as obj.geom_ad.wing.k_fit, obj.geom_ad.tail.a_t, obj.geom_ad.controls.tau_f.
%   Coefficients [-] unless noted. FILL = placeholder awaiting verification.
%
%   Lateral/roll/yaw (CY, Cml, Cn) fits are performed in DEGREES (angles); Cmyu
%   is dimensionless.
%   Post-stall drag: CD_flat = K_flat*(2*sin(alpha)^2); stall angle
%   alpha0 = (a0_const + a0_Cmyu*Cmyu)*pi/180 [rad]; W(alpha) is a Beard-style
%   blend with sharpness k (see CD_stall_weight in the aero model).
%   Propulsion fits: Cmyu = cmyu_J2*(1/J^2) + cmyu_0 (J = advance ratio);
%   CT = CT_1 + CT_J*J + CT_JM*J*Mtip
%   (Mtip = blade tip Mach number).

    properties
        geom_ad     % nested aero struct (.wing, .tail, .fuse, .controls, .lateral, .roll, .yaw, .stall, .cl_stall, .prop)
    end

    methods

        % Constructor — fill component sub-structs, then store on the object
        function obj = uSTOL_Demonstrator_AERO_Params()

            % --- Wing (blown-lift scaling + induced drag) ---
            wing.k_fit      = 0.535;      % Spence amplitude scaling        [-]
            wing.cmyu_a     = 0.7643;     % Cmyu_fit = cmyu_a*Cmyu + cmyu_b [-]
            wing.cmyu_b     = 0.039;      % intercept of the fit above      [-]
            wing.cl0_camber = 0.519;      % Zero-alpha camber lift (2-D)    [-]
            wing.k_w        = 2.632757;   % Induced drag factor 1/(pi*e)    [-]  FILL
            wing.r          = -0.005473;  % Blown-wing momentum drag coeff  [-]  FILL

            % --- Tail ---
            tail.a_t      = 4.27;         % DATCOM 3-D lift-curve slope     [1/rad]
            tail.eta_t_0  = 1.0271;       % Tail efficiency offset          [-]
            tail.eta_t_1  = 0.010824;     % Tail efficiency Cmyu slope      [-]s
            tail.eps0     = 0.030729;     % Zero-AoA downwash               [rad]
            tail.deps_neg = 0.283491;     % d(eps)/d(alpha), alpha <  0     [-]
            tail.deps_pos = -0.018088;    % d(eps)/d(alpha), alpha >= 0     [-]
            tail.CD_ht0   = 0.004050;     % HT zero-lift drag               [-]  FILL
            tail.k_ht     = 3.13292;      % HT induced drag factor          [-]  FILL

            % --- Fuselage (+ baseline parasite drag) ---
            fuse.CD_a2     = 1.956124;    % alpha^2 coeff                   [rad^-2] FILL
            fuse.CD_b2     = 0.453343;    % beta^2 coeff                    [rad^-2] FILL
            fuse.CD_a2_Cmyu = 0.010846;   % alpha^2*Cmyu cross term         [rad^-2] FILL
            fuse.CD0       = 0.045249;    % Zero-lift parasite drag         [-]  FILL
            fuse.CDp_cu    =  0.000252791;% d(CD)/d(p*Cmyu)                 [per rate]
            fuse.CDq_cu    = -0.0120005;  % d(CD)/d(q*Cmyu)                 [per rate]
            fuse.CDr_cu    = -0.023006;   % d(CD)/d(r*Cmyu)                 [per rate]

            % --- Control surfaces (effectiveness + drag) ---
            controls.tau_f  = 0.297565;   % 2-D Fowler-flap effectiveness   [-]
            controls.tau_a  = 0.086943;   % 2-D aileron effectiveness       [-]
            controls.theta_j_a = deg2rad(18);  % aileron jet-turning angle (fixed; no flaperons) [rad]
            controls.Kb_f   = 0.506727;   % Fowler-flap span effectiveness  [-]
            controls.Kb_a   = 0.131584;   % Aileron span effectiveness       [-]
            controls.tau_e  = 0.625;      % 3-D elevator effectiveness      [-]
            % Elevator effectiveness rolloff (asymmetric, hard kink — see CL_tail/elevEffective):
            controls.de_eff_pos_break_deg = 10;    % +del_e beyond this loses authority (TE-down) [deg]
            controls.de_eff_pos_factor    = 0.25;  % marginal effectiveness beyond +break (1 = none) [-]
            controls.de_eff_neg_break_deg = 6;     % -del_e beyond this loses authority (TE-up)   [deg]
            controls.de_eff_neg_factor    = 0.5;   % marginal effectiveness beyond -break (1 = none) [-]
            controls.CD_df2 = 0.217554;   % delta_f^2                       [rad^-2] FILL
            controls.CD_da2 = 0.030418;   % delta_a^2                       [rad^-2] FILL
            controls.CD_da  = 0.013566;   % delta_a (linear)                [rad^-1] FILL
            controls.CD_de2 = 0.164864;   % delta_e^2                       [rad^-2] FILL
            controls.CD_de  = 0.010177;   % delta_e (linear)                [rad^-1] FILL
            controls.CD_dr2 = 0.142042;   % delta_r^2                       [rad^-2] FILL

             % --- Post-stall drag (flat-plate surrogate + stall blend; see header) ---
            stall.K_flat   = 5.057530;    % flat-plate drag scaling         [-]
            stall.a0_const = 18.459054;   % stall-angle offset              [deg]
            stall.a0_Cmyu  = -0.125111;    % stall-angle Cmyu sensitivity    [deg]
            stall.k        = 25.6810;     % blend sharpness                 [1/rad]

            % --- Post-stall LIFT (two Randall-Beard blends; see CL_total_poststall) ---
            %   WING : Mw=wM0+wM1*Cmyu [1/rad]; onset=(wa00+wa0mu*Cmyu+wa0f*[flap=20]) deg;
            %          flat-plate surrogate sin(2a), scale wkflat.
            %   REST : Mr [1/rad]; onset=(ra0+rkcu*Cmyu) deg; surrogate 2*sin(a)^2*cos(a),
            %          scale rkflat. Cmyu scaled by 0.52 inside the weights (as in CD).
            %   Fit on config1+config2.xlsx (AOA<=15 deg), no dCLw offset correction.
            %   NOTE: wa00 sits at its fit bound (~20 deg) -> the wing-stall onset is
            %   NOT physically resolved by this data (stall only weakly present at low
            %   Cmyu); lower wa00 (~7-9) if a ~12 deg onset is required (raises RMSE).
            cl_stall.wM0    =  21.8813;   % wing blend sharpness, const      [1/rad]
            cl_stall.wM1    =  -2.4776;   % wing blend sharpness, Cmyu slope [1/rad]
            cl_stall.wa00   =  19.9997;   % wing stall onset, const          [deg]
            cl_stall.wa0mu  =   1.4955;   % wing onset Cmyu sensitivity      [deg]
            cl_stall.wa0f   =  -5.2527;   % wing onset flap(=20) shift       [deg]
            cl_stall.wkflat =   4.0284;   % wing flat-plate (sin2a) scale    [-]
            cl_stall.rM     =  14.3286;   % rest blend sharpness             [1/rad]
            cl_stall.ra0    =  13.2261;   % rest stall onset, const          [deg]
            cl_stall.rkcu   =   0.4765;   % rest onset Cmyu sensitivity      [deg]
            cl_stall.rkflat =   1.3514;   % rest flat-plate scale            [-]

            % NOTE: control limits (del_e/del_a/del_r/del_thr/alpha/beta min,max) live ONLY in
            % uSTOL_demonstrator_constraints (con.controls); all code reads them from there.

            % --- Lateral side force (CY: attached linear + post-stall blend; fit in deg) ---
            lateral.theta0    =  0.000424;   % constant offset                 [-]
            lateral.theta_b   = -0.014394;   % d(CY)/d(beta)                   [1/deg]
            lateral.theta_aL  = -0.000145;   % d(CY)/d(delta_aL)               [1/deg]
            lateral.theta_aR  =  0.000141;   % d(CY)/d(delta_aR)               [1/deg]
            lateral.theta_r   =  0.004425;   % d(CY)/d(delta_r)                [1/deg]
            lateral.theta_bcu = -0.006646;   % d(CY)/d(beta*Cmyu)              [1/deg]
            lateral.beta0     = 12.00000;    % stall-onset sideslip            [deg]
            lateral.kr        =  0.0;        % onset rudder sensitivity        [deg/deg]
            lateral.kaf       =  0.0;        % onset flap/aileron sensitivity  [deg/deg]
            lateral.kcu       =  0.0;        % onset Cmyu sensitivity          [deg]
            lateral.kalpha    =  0.0;        % onset alpha sensitivity         [deg/deg]
            lateral.kv        = -1.370707;   % flat-plate surrogate scaling    [-]
            lateral.kps_r     =  0.606335;   % post-stall rudder shift         [deg/deg]
            lateral.M         = 59.998281;   % Beard blend sharpness           [1/rad]
            % Dynamic derivatives (non-dim rates: p_hat = p*b/2V, r_hat = r*c/2V; rates in rad/s)
            lateral.CYp     = -0.13707;    % d(CY)/d(p_hat)                  [-]
            lateral.CYr     =  0.35017;    % d(CY)/d(r_hat)                  [-]
            lateral.CYp_cu  = -0.057771;   % d(CY)/d(p_hat*Cmyu)             [-]
            lateral.CYr_cu  =  0.042414;   % d(CY)/d(r_hat*Cmyu)             [-]
            lateral.CYp2_cu =  0.40687;    % d(CY)/d(p_hat^2*Cmyu)           [-]
            lateral.CYr2_cu = -1.3478;     % d(CY)/d(r_hat^2*Cmyu)           [-]

            % --- Rolling moment (Cml: attached linear-in-parameters model; fit in deg) ---
            roll.theta0    = -0.000102;   % constant offset                 [-]
            roll.theta_aL  =  0.000638;   % d(Cml)/d(delta_aL)              [1/deg]
            roll.theta_aR  = -0.000677;   % d(Cml)/d(delta_aR)             [1/deg]
            roll.theta_aLcu =  0.000108;  % d(Cml)/d(delta_aL*Cmyu)        [1/deg]
            roll.theta_aRcu = -0.000094;  % d(Cml)/d(delta_aR*Cmyu)        [1/deg]
            roll.theta_b   = -0.000381;   % d(Cml)/d(beta)                 [1/deg]
            roll.theta_b_cu = 0.000069;   % d(Cml)/d(beta*Cmyu)                 [1/deg]
            roll.theta_r   =  0.000497;   % d(Cml)/d(delta_r)              [1/deg]
            % Dynamic derivatives (non-dim rates: p_hat=p*b/2V, q_hat=q*c/2V, r_hat=r*b/2V; rates in rad/s)
            roll.Clp    = -0.54644;    % d(Cl)/d(p_hat)                  [-]
            roll.Clr    = -0.57722;    % d(Cl)/d(r_hat)                  [-]
            roll.Clp_cu =  0.061697;   % d(Cl)/d(p_hat*Cmyu)             [-]
            roll.Clr_cu = 0.025503;  % d(Cl)/d(r_hat*Cmyu)             [-]

            % --- Lateral yawing moment (Cn: attached linear + post-stall blend; fit in deg) ---
            yaw.theta0     =  0.000281;   % constant offset                 [-]
            yaw.theta_b    =  0.003317;   % d(Cn)/d(beta)                   [1/deg]
            yaw.theta_aL   = -0.000016;   % d(Cn)/d(delta_aL)               [1/deg]
            yaw.theta_aR   =  0.000001;   % d(Cn)/d(delta_aR)               [1/deg]
            yaw.theta_r    = -0.001842;   % d(Cn)/d(delta_r)                [1/deg]
            yaw.theta_bcu  = -0.000090;   % d(Cn)/d(beta*Cmyu)              [1/deg]
            yaw.theta_aLcu = -0.000029;   % d(Cn)/d(delta_aL*Cmyu)          [1/deg]
            yaw.theta_aRcu =  0.000027;   % d(Cn)/d(delta_aR*Cmyu)          [1/deg]
            yaw.Cnp        =  0.0437962;  % d(Cn)/dp                        [per rate]
            yaw.Cnp_cu     = -0.00190315; % d(Cn)/d(p*Cmyu)                 [per rate]
            yaw.Cnr        = -0.244335;   % d(Cn)/dr                        [per rate]
            yaw.Cnr_cu     =  0.0239989;  % d(Cn)/d(r*Cmyu)                 [per rate]
            yaw.beta0      = 11.999974;   % stall-onset sideslip            [deg]
            yaw.kr         =  0.0;        % onset rudder sensitivity        [deg/deg]
            yaw.kaf        =  0.0;        % onset flap/aileron sensitivity  [deg/deg]
            yaw.kcu        =  0.000009;   % onset Cmyu sensitivity          [deg]
            yaw.kalpha     =  0.0;        % onset alpha sensitivity         [deg/deg]
            yaw.kv         =  0.290114;   % flat-plate surrogate scaling    [-]
            yaw.kps_r      =  0.655515;   % post-stall rudder shift         [deg/deg]
            yaw.M          = 59.998281;   % Beard blend sharpness           [1/rad]

           
            % --- Propulsion (blowing momentum coeff & thrust coeff fits; see header) ---
            prop.cmyu_J2 =  0.53211;   % 1/J^2 coeff                     [-]
            prop.cmyu_0  =  0.03669;   % constant offset                 [-]
            prop.CT_1    =   0.691683; % constant                       [-]
            prop.CT_J    =  -0.734466; % J                              [-]
            prop.CT_JM   =   1.64705;  % J*Mtip                         [-]

            % Advance-ratio / blowing-coeff clamps (fit-validity bounds; shared by
            % the dynamic stepper and trim driver). NOTE: the trim solve uses J_max=5
            % for robustness while the dynamic loads runners use Inf — set J_max=Inf
            % here if the time-domain stepper should match the dynamic runners.
            prop.J_min    =  1e-3;    % advance-ratio lower clamp                       [-]
            prop.J_max    =  5;       % advance-ratio upper clamp                       [-]
            prop.Cmyu_max =  9.21;    % blowing-coeff upper clamp (fit ceiling)         [-]

            geom_ad.wing     = wing;
            geom_ad.tail     = tail;
            geom_ad.fuse     = fuse;
            geom_ad.controls = controls;
            geom_ad.lateral  = lateral;
            geom_ad.roll     = roll;
            geom_ad.yaw      = yaw;
            geom_ad.stall    = stall;
            geom_ad.cl_stall = cl_stall;
            geom_ad.prop     = prop;
            obj.geom_ad = geom_ad;
        end
    end
end
