classdef uSTOL_Demonstrator_AERO_Model
% AeroModel  Aerodynamic model for uSTOL Demonstrator
%
%   Constants are injected as two nested structs:
%     geom_ac : aircraft geometry / mass   (uSTOL_Demonstrator_Aircraft_Params)
%     geom_ad : aerodynamic coefficients   (uSTOL_Demonstrator_AERO_Params)
%   Read them as obj.geom_ac.wing.b, obj.geom_ad.tail.a_t, obj.geom_ad.controls.tau_f.
%
%   Input bundles (arrays) used by the assembler methods:
%     states_ac   = [theta, q, alphadot, V]              body / flight states
%     aero_states = [alpha, beta, Cmyu]                  aerodynamic states
%     ctrl        = [delta_e, delta_f, delta_a, delta_r] control deflections
%   All angles in radians.
%
%   CD model (post-stall blended drag polar):
%     CD = (1 - W).*CD_baseline + W.*CD_flat + CD_rest, where W(alpha, Cmyu)
%     blends the attached (baseline) polar into a flat-plate surrogate beyond
%     the stall angle. CD_rest (sideslip, blowing, control, tail-induced drag)
%     is always present. CD_baseline = CD0 + k_w*CL_wing^2/(pi*AR + 2*Cmyu)
%     + CD_a2*alpha^2 + CD_a2_Cmyu*alpha^2*Cmyu. CD_flat = K_flat*(2*sin(alpha)^2).
%   CD_stall_weight: alpha0 = (a0_const + a0_Cmyu*Cmyu) in deg, converted to rad;
%     W ~ 0 below stall, W ~ 1 fully stalled.
%   CL model: CL_total is the attached (pre-stall) lift; CL_total_poststall adds a
%     Randall-Beard blend (mirrors the CD structure) - the WING and the REST
%     (tail+fuselage) each blend their attached lift into a flat-plate surrogate
%     beyond a blowing/flap-dependent onset. Coeffs in geom_ad.cl_stall.
%   CY/Cml/Cn fits are in DEGREES; public inputs stay in radians (file
%   convention) and are converted internally.
%   Propeller kinematics inputs: del_T = throttle [0..1], V = airspeed [m/s],
%     a = speed of sound [m/s]. Propulsion fits take propeller operating-point
%     quantities: J = advance ratio [-], Mtip = blade tip Mach number [-].
%
%   Moment transfer to a point P = [x_P, z_P]  (x aft of wing LE +, z up from A/C base +):
%     Cm_about_shifted_cg(...)         - P is the NEW CG; pure aero moment (wing+tail)
%                                        transferred to P, no thrust/weight/pseudo terms.
%     Cm_about_rotation_point(...,T,W,a_cg) - CG stays as in params, P is a rotation point;
%                                        aero, thrust, weight & the pseudo force all contribute.
%   a_cg = [a_fwd, a_up] CG accel [m/s^2], optional ([] -> 0); pseudo force = -(W/g)*a_cg.
%   Both return Cm_P non-dimensionalised by q*S*c; transfer_setup is their shared helper.

    properties
        geom_ac     % aircraft geometry/mass struct (.wing, .tail, .cg)
        geom_ad     % aerodynamic-coefficient struct (.wing, .tail, .fuse, .controls, .lateral, .roll, .yaw, .stall, .cl_stall, .prop)
        atmos       % atmospheric parameters struct (.rho, .a)
    end

    methods

        %% Constructor — inject the geometry & aero structs
        function obj = uSTOL_Demonstrator_AERO_Model(geom_ac, geom_ad, atmos)
            obj.geom_ac = geom_ac;
            obj.geom_ad = geom_ad;
            obj.atmos = atmos;
        end

        %% CL components
        % CL — Wing
        function CL_w = CL_wing(obj, Cmyu_star, alpha, delta_f, delta_a)
            Cmyu = Cmyu_star * obj.geom_ac.wing.lambda_b;  % 3D blown value

            cl_alpha = @(cm) 2*pi .* (1 + 0.151.*sqrt(cm) + 0.219.*cm);
            cl_jet_turn = @(cm) sqrt(4*pi .* cm .* (1 + 0.151.*sqrt(cm) + 0.139.*cm));

            cla     = cl_alpha(Cmyu_star);
            cl_jt   = cl_jet_turn(Cmyu_star);
            cl_delf    = 2*pi * obj.geom_ad.controls.tau_f;
            cl_dela    = 2*pi * obj.geom_ad.controls.tau_a;

            G = (obj.geom_ac.wing.AR_w + 0.637.*Cmyu) ./ ...
        (obj.geom_ac.wing.AR_w + 2 + 0.604.*sqrt(Cmyu) + 0.876.*Cmyu);

            nu_camber = obj.geom_ad.wing.k_fit;
            nu_alpha    = (obj.geom_ac.wing.lambda_b + (1 - obj.geom_ac.wing.lambda_b) .* cl_alpha(0) ./ cla) * obj.geom_ad.wing.k_fit;
            nu_jet_turn = obj.geom_ac.wing.lambda_b;
            nu_delf    = obj.geom_ac.wing.S_f / obj.geom_ac.wing.S * obj.geom_ad.controls.Kb_f * obj.geom_ad.wing.k_fit;
            nu_dela    = obj.geom_ac.wing.S_a / obj.geom_ac.wing.S * obj.geom_ad.controls.Kb_a * obj.geom_ad.wing.k_fit;

            % jet turning angle: flap from theta_j(flap) linear model, aileron fixed param (no flaperons)
            jet_turn = obj.theta_j_flap(delta_f) * 7/9 + obj.geom_ad.controls.theta_j_a * 2/9;

            CL_w = G .* (1 + obj.geom_ac.wing.tc_w) .* ...
            ( nu_camber .* obj.geom_ad.wing.cl0_camber ...
            + nu_jet_turn .* cl_jt .* jet_turn ...
            + nu_alpha    .* cla .* alpha ...
            + nu_delf    .* cl_delf .* delta_f ...
            + nu_dela    .* cl_dela .* delta_a) ...
           - obj.geom_ac.wing.tc_w .* (jet_turn + alpha) .* Cmyu;
        end

        % theta_j_flap — flap jet-turning angle [rad]: linear in flap, 0 deg -> 18 deg, 20 deg -> 32 deg
        function theta_j = theta_j_flap(~, delta_f)
            theta_j = deg2rad(18) + 0.7 .* delta_f;   % slope 0.7 = (32-18)/(20-0) [deg/deg]
        end

        % CL — Horizontal Tail
        function CL_t = CL_tail(obj, alpha, Cmyu_star, delta_e)
            eta_t       = obj.geom_ad.tail.eta_t_0 - obj.geom_ad.tail.eta_t_1 .* Cmyu_star;
            deps        = (alpha <  0) .* obj.geom_ad.tail.deps_neg + (alpha >= 0) .* obj.geom_ad.tail.deps_pos;
            de_eff      = obj.elevEffective(delta_e);   % asymmetric effectiveness rolloff beyond +pos_break / -neg_break
            CL_t = eta_t .* obj.geom_ac.tail.S_ht_S .* obj.geom_ad.tail.a_t .* ...
           (-obj.geom_ad.tail.eps0 + (1 - deps) .* alpha + obj.geom_ad.controls.tau_e .* de_eff);
        end

        % elevEffective — effective elevator deflection feeding the tail lift, with an
        % ASYMMETRIC effectiveness rolloff (hard kink, no blend). Authority is FULL inside the
        % band [-de_eff_neg_break, +de_eff_pos_break]; beyond +break the slope drops to
        % de_eff_pos_factor (e.g. 1/4, TE-down) and below -break to de_eff_neg_factor (e.g. 1/2,
        % TE-up). So d(CL_tail)/d(delta_e) past each break is tau_e*(that factor), while CL_tail
        % stays continuous at the breaks. Set both factors = 1 to recover the linear elevator.
        function de = elevEffective(obj, delta_e)
            c  = obj.geom_ad.controls;
            bp = deg2rad(c.de_eff_pos_break_deg);  kp = c.de_eff_pos_factor;   % +del_e side (TE-down)
            bn = deg2rad(c.de_eff_neg_break_deg);  kn = c.de_eff_neg_factor;   % -del_e side (TE-up)
            pos = max(delta_e, 0);   neg = -min(delta_e, 0);                   % split into +/- parts, both >= 0
            de  = ( min(pos, bp) + kp .* max(pos - bp, 0) ) ...                % delta_e >= 0 branch
                - ( min(neg, bn) + kn .* max(neg - bn, 0) );                  % delta_e <  0 branch
        end

        % CL — Fuselage
        function CL_f = CL_fus(~, alpha, Cmyu_star)
            CL_f = ( 0.030384 + 0.004932.*sqrt(Cmyu_star) ) ...
           + ( 0.196106 + 0.073289.*sqrt(Cmyu_star) - 0.016049.*Cmyu_star ) .* alpha;
        end

        % CL — Dynamic Derivatives
        %   ctrl = [delta_e, delta_f, delta_a, delta_r]
        function [CLq, CLadot] = CL_dynamic(obj, flap_config, alpha, Cmyu_star, ctrl)
            delta_f = ctrl(2);  delta_a = ctrl(3);
            eta_t   = obj.geom_ad.tail.eta_t_0 - obj.geom_ad.tail.eta_t_1 .* Cmyu_star;

            dalpha   = 1e-6;
            CL_alpha = ( obj.CL_wing(Cmyu_star, alpha + dalpha, delta_f, delta_a) ...
                       - obj.CL_wing(Cmyu_star, alpha - dalpha, delta_f, delta_a) ) ./ (2*dalpha);

            x_ac_w = obj.aero_center(flap_config, Cmyu_star);
            deps   = (alpha <  0) .* obj.geom_ad.tail.deps_neg + (alpha >= 0) .* obj.geom_ad.tail.deps_pos;
            CLq    = 2 .* CL_alpha .* (x_ac_w - obj.geom_ac.cg.x_cg_c) ...
                   + 2 .* eta_t .* obj.geom_ad.tail.a_t .* obj.geom_ac.tail.V_H;
            CLadot = 2 .* eta_t .* obj.geom_ad.tail.a_t .* obj.geom_ac.tail.V_H .* deps;
        end

        % CL — Assembler (FULL model: attached lift + post-stall blend applied BY DEFAULT,
        % matching CD_total/CY_total/Cn_total). Wing and rest (tail+fus) each blend into a
        % flat-plate surrogate beyond a blowing/flap-dependent stall onset; see the
        % CL_*_stall_weight helpers and geom_ad.cl_stall below.
        function [CL, comp] = CL_total(obj, flap_config, states_ac, aero_states, ctrl)
            % states_ac = [theta, q, alphadot, V]; aero_states = [alpha, beta, Cmyu]
            % Optional 2nd output 'comp' = section-wise contributions (wing / tail+fus / dynamic).
            % ctrl = [delta_e, delta_f, delta_a, delta_r]
            q = states_ac(2);  alphadot = states_ac(3);  V = max(states_ac(4), 0.01);
            alpha = aero_states(1);  Cmyu = aero_states(3);
            delta_e = ctrl(1);  delta_f = ctrl(2);  delta_a = ctrl(3);
            c = obj.geom_ac.wing.c;
            s = obj.geom_ad.cl_stall;

            % Attached (pre-stall) components (jet turning derived inside CL_wing from delta_f)
            CL_w = obj.CL_wing(Cmyu, alpha, delta_f, delta_a);
            CL_t = obj.CL_tail(alpha, Cmyu, delta_e);
            CL_f = obj.CL_fus(alpha, Cmyu);

            % Post-stall blends (wing & rest) into their flat-plate surrogates
            Ww = obj.CL_wing_stall_weight(alpha, Cmyu, flap_config);
            Wr = obj.CL_rest_stall_weight(alpha, Cmyu);
            CL_wing_post = (1 - Ww) .* CL_w + Ww .* s.wkflat .* sin(2 .* alpha);
            CL_rest_post = (1 - Wr) .* (CL_t + CL_f) ...
                         + Wr .* s.rkflat .* (2 .* sign(alpha) .* sin(alpha).^2 .* cos(alpha));

            % Dynamic (rate) terms
            [CLq, CLadot] = obj.CL_dynamic(flap_config, alpha, Cmyu, ctrl);
            CL_dyn = CLq .* (q .* c ./ (2 .* V)) + CLadot .* (alphadot .* c ./ (2 .* V));
            CL = CL_wing_post + CL_rest_post + CL_dyn;

            % Section-wise contributions (sum to CL); built only when a 2nd output is requested
            comp = [];
            if nargout > 1
                comp = struct('wing', CL_wing_post, 'rest_tail_fus', CL_rest_post, ...
                              'dynamic', CL_dyn, ...
                              'wing_attached', CL_w, 'tail_attached', CL_t, 'fus_attached', CL_f, ...
                              'wing_stall_wt', Ww, 'rest_stall_wt', Wr);
            end
        end

        %% CL — full post-stall model (pre-stall attached CL + Randall-Beard blend)
        % Mirrors the CD post-stall structure: the WING and the REST (tail+fuselage)
        % each blend their attached lift into a flat-plate surrogate beyond stall,
        % with a blowing/flap-dependent onset. Coefficients in geom_ad.cl_stall.
        %   CL_wing^post = (1-Ww).*CL_wing + Ww.*wkflat.*sin(2*alpha)
        %   CL_rest^post = (1-Wr).*(CL_tail+CL_fus) + Wr.*rkflat.*(2*sign(a)*sin(a)^2*cos(a))
        % Cmyu is scaled by 0.52 inside the weights (same convention as CD_stall_weight).

        % CL — wing post-stall blend weight (Beard; sharpness M & onset vary w/ blowing,flap)
        function W = CL_wing_stall_weight(obj, alpha, Cmyu, flap_config)
            Cmyu = 0.52 .* Cmyu;
            s   = obj.geom_ad.cl_stall;
            f20 = (flap_config == 32);                                     % flap = 20 deg config
            M   = max(s.wM0 + s.wM1 .* Cmyu, 0.5);                         % sharpness [1/rad], kept > 0
            a0  = (s.wa00 + s.wa0mu .* Cmyu + s.wa0f .* f20) .* pi/180;    % onset AOA(Cmyu,flap) [rad]
            e1  = exp(-M .* (alpha - a0));
            e2  = exp( M .* (alpha + a0));
            W   = (1 + e1 + e2) ./ ((1 + e1) .* (1 + e2));
        end

        % CL — rest (tail+fuselage) post-stall blend weight (Beard; onset a0 + kcu*Cmyu)
        function W = CL_rest_stall_weight(obj, alpha, Cmyu)
            Cmyu = 0.52 .* Cmyu;
            s  = obj.geom_ad.cl_stall;
            a0 = (s.ra0 + s.rkcu .* Cmyu) .* pi/180;                       % onset AOA(Cmyu) [rad]
            e1 = exp(-s.rM .* (alpha - a0));
            e2 = exp( s.rM .* (alpha + a0));
            W  = (1 + e1 + e2) ./ ((1 + e1) .* (1 + e2));
        end

        % CL_total_poststall — retained alias. CL_total now applies the post-stall blend
        % by default, so this just forwards to it (kept for callers/scripts that named it).
        function CL = CL_total_poststall(obj, flap_config, states_ac, aero_states, ctrl)
            CL = obj.CL_total(flap_config, states_ac, aero_states, ctrl);
        end

        %% CD components (post-stall blended drag polar; see header)

        % CD — pre-stall baseline polar  (scaled by (1 - W))
        function CD_b = CD_baseline(obj, CL_wing, alpha, Cmyu)
            Cmyu = 0.52 * Cmyu;
            CD_b = obj.geom_ad.fuse.CD0 ...
                 + obj.geom_ad.wing.k_w .* CL_wing.^2 ./ (pi .* obj.geom_ac.wing.AR_w + 2 .* Cmyu) ...
                 + obj.geom_ad.fuse.CD_a2      .* alpha.^2 ...
                 + obj.geom_ad.fuse.CD_a2_Cmyu .* alpha.^2 .* Cmyu;
        end

        % CD — fully-stalled flat-plate surrogate  (scaled by W)
        function CD_fp = CD_flat(obj, alpha)
            CD_fp = obj.geom_ad.stall.K_flat .* (2 .* sin(alpha).^2);
        end

        % CD — smooth stall-blend (Beard) weight [-] (see header)
        function W = CD_stall_weight(obj, alpha, Cmyu)
            Cmyu = 0.52 * Cmyu;
            s  = obj.geom_ad.stall;
            a0 = (s.a0_const + s.a0_Cmyu .* Cmyu) .* pi/180;
            e1 = exp(-s.k .* (alpha - a0));
            e2 = exp( s.k .* (alpha + a0));
            W  = (1 + e1 + e2) ./ ((1 + e1) .* (1 + e2));
        end

        % CD — always-present terms (sideslip, blowing, controls, tail induced)
        function CD_r = CD_rest(obj, CL_ht, beta, Cmyu, delta_e, delta_f, delta_a, delta_r)
            Cmyu = 0.52 * Cmyu;
            CD_r = obj.geom_ad.fuse.CD_b2  .* beta.^2 ...
                 + obj.geom_ad.wing.r      .* Cmyu ...                 % CD_Cmyu (linear blowing drag)
                 + obj.geom_ad.controls.CD_df2 .* delta_f.^2 ...
                 + obj.geom_ad.controls.CD_de2 .* delta_e.^2 + obj.geom_ad.controls.CD_de .* delta_e ...
                 + obj.geom_ad.controls.CD_da2 .* delta_a.^2 + obj.geom_ad.controls.CD_da .* delta_a ...
                 + obj.geom_ad.controls.CD_dr2 .* delta_r.^2 ...
                 + obj.geom_ad.tail.CD_ht0 ...                          % CD_0,rest (constant)
                 + obj.geom_ad.tail.k_ht .* CL_ht.^2 ./ (pi .* obj.geom_ac.tail.AR_ht);
        end

        % CD — Assembler  (p, q, r are roll/pitch/yaw rates [rad/s]; optional, default 0)
        function [CD, comp] = CD_total(obj, CL_wing, CL_ht, aero_states, ctrl, p, q, r)
            % aero_states = [alpha, beta, Cmyu]; ctrl = [delta_e, delta_f, delta_a, delta_r]
            % Optional 2nd output 'comp' = buildup (wing polar / flat-plate / rest / rate).
            if nargin < 6 || isempty(p), p = 0; end
            if nargin < 7 || isempty(q), q = 0; end
            if nargin < 8 || isempty(r), r = 0; end
            alpha = aero_states(1);  beta = aero_states(2);  Cmyu = aero_states(3);
            delta_e = ctrl(1);  delta_f = ctrl(2);  delta_a = ctrl(3);  delta_r = ctrl(4);

            % Roll/pitch/yaw-rate drag (blowing cross terms; rates in rad/s)
            CD_rate = obj.geom_ad.fuse.CDp_cu .* (Cmyu*0.52) .* p ...
                    + obj.geom_ad.fuse.CDq_cu .* (Cmyu*0.52) .* q ...
                    + obj.geom_ad.fuse.CDr_cu .* (Cmyu*0.52) .* r;

            W        = obj.CD_stall_weight(alpha, Cmyu);
            CD_polar = (1 - W) .* obj.CD_baseline(CL_wing, alpha, Cmyu);   % attached wing polar
            CD_fp    =      W  .* obj.CD_flat(alpha);                       % stalled flat-plate
            CD_r     = obj.CD_rest(CL_ht, beta, Cmyu, delta_e, delta_f, delta_a, delta_r);  % tail+controls+sideslip+blowing
            CD = CD_polar + CD_fp + CD_r + CD_rate;

            comp = [];
            if nargout > 1
                comp = struct('wing_polar', CD_polar, 'flat_plate', CD_fp, ...
                              'rest_tail_ctrl', CD_r, 'rate', CD_rate, 'stall_wt', W);
            end
        end

        %% CY components (lateral side force; fit in degrees, see header)

        % CY — stall-onset sideslip angle [deg]
        function beta_stall = CY_stall_onset(obj, delta_aL, delta_aR, delta_r, delta_f, Cmyu, alpha)
            p   = obj.geom_ad.lateral;
            r2d = 180/pi;
            beta_stall = p.beta0 ...
                       + p.kr     .* (delta_r .* r2d) ...
                       + p.kaf    .* (0.5 .* (delta_aL + delta_aR) .* r2d + delta_f .* r2d) ...
                       + p.kcu    .*  Cmyu ...
                       + p.kalpha .* (alpha   .* r2d);
        end

        % CY — smooth post-stall blending (Beard) weight [-]
        function W = CY_beard_weight(obj, beta_eff_rad, beta_stall_deg)
            M  = obj.geom_ad.lateral.M;
            b0 = beta_stall_deg .* pi/180;
            e1 = exp(-M .* (beta_eff_rad - b0));
            e2 = exp( M .* (beta_eff_rad + b0));
            W  = (1 + e1 + e2) ./ ((1 + e1) .* (1 + e2));
        end

        % CY — Assembler  (p, r are roll/yaw rates [rad/s], V airspeed [m/s]; optional, default 0)
        function CY = CY_total(obj, beta, delta_aL, delta_aR, delta_r, delta_f, Cmyu, alpha, p, r, V)
            Cmyu = 0.52 * Cmyu;
            if nargin < 9  || isempty(p), p = 0; end
            if nargin < 10 || isempty(r), r = 0; end
            if nargin < 11 || isempty(V), V = 0; end
            lat = obj.geom_ad.lateral;
            r2d = 180/pi;

            beta_d = beta     .* r2d;
            daL_d  = delta_aL .* r2d;
            daR_d  = delta_aR .* r2d;
            dr_d   = delta_r  .* r2d;

            % Attached linear terms
            CY_att   = lat.theta0 + lat.theta_b .* beta_d ...
                     + lat.theta_aL .* daL_d + lat.theta_aR .* daR_d ...
                     + lat.theta_r  .* dr_d  + lat.theta_bcu .* (beta_d .* Cmyu);
            CY_base  = lat.theta_b .* beta_d;
            CY_other = CY_att - CY_base;

            % Stall onset & post-stall effective sideslip
            beta_stall   = obj.CY_stall_onset(delta_aL, delta_aR, delta_r, delta_f, Cmyu, alpha);
            beta_eff_rad = (beta_d + lat.kps_r .* dr_d) .* pi/180;

            % Beard blend + flat-plate surrogate
            W       = obj.CY_beard_weight(beta_eff_rad, beta_stall);
            CY_flat = 2 .* sign(beta_eff_rad) .* sin(beta_eff_rad).^2 .* cos(beta_eff_rad);

            % Roll/yaw-rate side force (non-dim rates p_hat=p*b/2V, r_hat=r*c/2V; rates in rad/s)
            if V <= 0
                CY_rate = 0;
            else
                p_hat = p .* obj.geom_ac.wing.b ./ (2 .* V);
                r_hat = r .* obj.geom_ac.wing.b ./ (2 .* V);
                CY_rate = lat.CYp     .* p_hat ...
                        + lat.CYr     .* r_hat ...
                        + lat.CYp_cu  .* (p_hat      .* Cmyu) ...
                        + lat.CYr_cu  .* (r_hat      .* Cmyu) ...
                        + lat.CYp2_cu .* (p_hat.^2   .* Cmyu) ...
                        + lat.CYr2_cu .* (r_hat.^2   .* Cmyu);
            end

            CY = CY_base .* (1 - W) + W .* (lat.kv .* CY_flat) + CY_other + CY_rate;
        end

        %% Cml components (rolling moment; fit in degrees, see header)

        % Cml — Assembler  (p, q, r are roll/pitch/yaw rates [rad/s], V airspeed [m/s]; optional, default 0)
        function Cml = Cml_total(obj, beta, delta_aL, delta_aR, delta_r, Cmyu, p, q, r, V)
            Cmyu = 0.52 * Cmyu;
            if nargin < 7  || isempty(p), p = 0; end
            if nargin < 8  || isempty(q), q = 0; end
            if nargin < 9  || isempty(r), r = 0; end
            if nargin < 10 || isempty(V), V = 0; end
            rl  = obj.geom_ad.roll;
            r2d = 180/pi;

            beta_d = beta     .* r2d;
            daL_d  = delta_aL .* r2d;
            daR_d  = delta_aR .* r2d;
            dr_d   = delta_r  .* r2d;

            % Left/right aileron base + blowing cross terms
            A_L = rl.theta_aL .* daL_d + rl.theta_aLcu .* (daL_d .* Cmyu);
            A_R = rl.theta_aR .* daR_d + rl.theta_aRcu .* (daR_d .* Cmyu);

            % Attached sideslip / rudder term
            R_att = rl.theta0 + rl.theta_b .* beta_d + rl.theta_r .* dr_d + rl.theta_b_cu.*(beta_d .* Cmyu);

            % Roll/pitch/yaw-rate rolling moment (non-dim rates p_hat=p*b/2V, q_hat=q*c/2V, r_hat=r*b/2V; rates in rad/s)
            if V <= 0
                Cml_rate = 0;
            else
                p_hat = p .* obj.geom_ac.wing.b ./ (2 .* V);
                %q_hat = q .* obj.geom_ac.wing.c ./ (2 .* V);
                r_hat = r .* obj.geom_ac.wing.b ./ (2 .* V);
                Cml_rate = rl.Clp    .* p_hat ...
                         + rl.Clr    .* r_hat ...
                         + rl.Clp_cu .* (p_hat .* Cmyu) ...
                         + rl.Clr_cu .* (r_hat .* Cmyu);
                         
            end

            Cml = A_L + A_R + R_att + Cml_rate;
        end

        %% Cn components (lateral yawing moment; fit in degrees, see header)

        % Cn — stall-onset sideslip angle [deg]
        function beta_stall = Cn_stall_onset(obj, delta_aL, delta_aR, delta_r, delta_f, Cmyu, alpha)
            p   = obj.geom_ad.yaw;
            r2d = 180/pi;
            beta_stall = p.beta0 ...
                       + p.kr     .* (delta_r .* r2d) ...
                       + p.kaf    .* (0.5 .* (delta_aL + delta_aR) .* r2d + delta_f .* r2d) ...
                       + p.kcu    .*  Cmyu ...
                       + p.kalpha .* (alpha   .* r2d);
        end

        % Cn — smooth post-stall blending (Beard) weight [-]
        function W = Cn_beard_weight(obj, beta_eff_rad, beta_stall_deg)
            M  = obj.geom_ad.yaw.M;
            b0 = beta_stall_deg .* pi/180;
            e1 = exp(-M .* (beta_eff_rad - b0));
            e2 = exp( M .* (beta_eff_rad + b0));
            W  = (1 + e1 + e2) ./ ((1 + e1) .* (1 + e2));
        end

        % Cn — Assembler  (p, r are roll/yaw rates; optional, default 0)
        function Cn = Cn_total(obj, beta, delta_aL, delta_aR, delta_r, delta_f, Cmyu, alpha, p, r)
            Cmyu_raw = Cmyu;        % Cn dynamic derivatives (Cnp/Cnr) were fitted vs raw C_mu
            Cmyu = 0.52 * Cmyu;
            if nargin < 9  || isempty(p), p = 0; end
            if nargin < 10 || isempty(r), r = 0; end
            y   = obj.geom_ad.yaw;
            r2d = 180/pi;

            beta_d = beta     .* r2d;
            daL_d  = delta_aL .* r2d;
            daR_d  = delta_aR .* r2d;
            dr_d   = delta_r  .* r2d;

            % Attached linear terms (incl. beta/aileron blowing cross terms)
            Cn_att   = y.theta0 + y.theta_b .* beta_d ...
                     + y.theta_aL .* daL_d + y.theta_aR .* daR_d ...
                     + y.theta_r  .* dr_d  + y.theta_bcu  .* (beta_d .* Cmyu) ...
                     + y.theta_aLcu .* (daL_d .* Cmyu) + y.theta_aRcu .* (daR_d .* Cmyu);
            Cn_base  = y.theta_b .* beta_d;
            Cn_other = Cn_att - Cn_base;

            % Stall onset & post-stall effective sideslip
            beta_stall   = obj.Cn_stall_onset(delta_aL, delta_aR, delta_r, delta_f, Cmyu, alpha);
            beta_eff_rad = (beta_d + y.kps_r .* dr_d) .* pi/180;

            % Beard blend + flat-plate surrogate
            W       = obj.Cn_beard_weight(beta_eff_rad, beta_stall);
            Cn_flat = 2 .* sign(beta_eff_rad) .* sin(beta_eff_rad).^2 .* cos(beta_eff_rad);

            % Roll/yaw-rate damping (with blowing cross terms)
            Cn_rate = (y.Cnp + y.Cnp_cu .* Cmyu_raw) .* p ...
                    + (y.Cnr + y.Cnr_cu .* Cmyu_raw) .* r;

            Cn = Cn_base .* (1 - W) + W .* (y.kv .* Cn_flat) + Cn_other + Cn_rate;
        end
        %% Propeller kinematics (operating point from throttle / airspeed; see header)

        % n_prop — propeller speed [rev/s] from throttle (n_max stored in rpm)
        function n = n_prop(obj, del_T)
            n = del_T .* obj.geom_ac.prop.n_max ./ 60;
        end

        % J — advance ratio from throttle & airspeed:  J = V / (n * Dia)
        function J = J(obj, del_T, V)
            J = V ./ (obj.n_prop(del_T) .* obj.geom_ac.prop.Dia);
        end

        % Mtip — blade tip Mach number:  Mtip = (pi * n * Dia) / a
        function Mtip = Mtip(obj, del_T)
         Mtip = pi .* obj.n_prop(del_T) .* obj.geom_ac.prop.Dia ./ obj.atmos.a;
        end
        % Propulsion components (blowing momentum coeff & thrust coeff; inputs J, Mtip — see header)

        % Cmyu — blowing momentum coefficient from advance ratio
        % Capped at the blown-model validity limit (prop.Cmyu_max): the 1/J^2 fit diverges as
        % J->0 (low speed), so without this the ground-roll CL/CD use unphysical Cmyu. Same cap
        % the aero_center / moment_about_aero_center fits apply -- centralized in AERO_Params.
        function Cmyu = Cmyu_prop(obj, J)
            p    = obj.geom_ad.prop;
            Cmyu = p.cmyu_J2 .* (1 ./ J.^2) + p.cmyu_0;
            Cmyu = min(Cmyu, p.Cmyu_max);
        end

        % CT — thrust coefficient from advance ratio & tip Mach number
        function CT = CT_prop(obj, J, Mtip)
            p  = obj.geom_ad.prop;
            CT = p.CT_1 + p.CT_J .* J + p.CT_JM .* J .* Mtip;
        end

        % thrustFromCT — total thrust [N] from prop speed & CT (SINGLE definition of the
        %   thrust formula): T = n_edf * rho * n_prop^2 * Dia^4 * CT.  n_edf = number of
        %   props/EDFs (param, NOT hardcoded). Pass CT = 1 to get the KT factor (T = KT*CT).
        function T = thrustFromCT(obj, n_prop, CT)
            T = obj.geom_ac.prop.n_edf .* obj.atmos.rho .* n_prop.^2 ...
                .* obj.geom_ac.prop.Dia.^4 .* CT;
        end

        % thrust — total thrust [N] from throttle & airspeed (computes the prop operating
        %   point, then thrustFromCT). J clamped to J_max (default Inf) for prop-fit validity.
        function T = thrust(obj, del_T, V, J_max)
            if nargin < 4 || isempty(J_max), J_max = Inf; end
            n_prop = obj.n_prop(del_T);
            J      = min(obj.J(del_T, V), J_max);
            CT     = obj.CT_prop(J, obj.Mtip(del_T));
            T      = obj.thrustFromCT(n_prop, CT);
        end

        % flapConfig — discrete pitch-fit selector derived from the flap deflection:
        %   del_f = 0 deg  -> 18,   del_f = 20 deg -> 32   (the only two settings the fits exist at).
        %   Nearest of the two is used (threshold 10 deg) so flap deflection is the single input and
        %   flap_config can never disagree with it.
        function fc = flapConfig(~, delta_f)
            if delta_f >= deg2rad(10), fc = 32; else, fc = 18; end
        end

        %% Aerodynamic center & pitching moment
        % Aero center — non-dim location x_ac_w(flap_config, Cmyu)
        function x_ac_w = aero_center(obj, flap_config, Cmyu)
            if flap_config == 18
                const = +0.26590;  b = -0.07152;  c_lin = +0.01033;   % refit to flap-0deg data (R2=0.996)
            elseif flap_config == 32
                const = +0.36842;  b = -0.17732;  c_lin = +0.00156;   % refit to corrected flap-20deg data (R2=0.999)
            else
                error('dyn_models:aero_center:BadFlap', ...
                      'flap_config must be 18 or 32 (got %g).', flap_config);
            end
            Cmyu_clip = min(max(Cmyu, 0), obj.geom_ad.prop.Cmyu_max);
            x_ac_w = const + b .* sqrt(Cmyu_clip) + c_lin .* Cmyu_clip;
        end

        % Pitching moment about aero center — Cm_ac_w(flap_config, Cmyu)
        function Cm_ac_w = moment_about_aero_center(obj, flap_config, Cmyu)
            if flap_config == 18
                a = -0.09464;  b = -0.04901;  c_lin = -0.08333;   % refit to flap-0deg data (R2=0.99995)
            elseif flap_config == 32
                a = -0.34177;  b = +0.29446;  c_lin = -0.30770;   % refit to flap-20deg data (R2=0.9999)
            else
                error('dyn_models:moment_about_aero_center:BadFlap', ...
                      'flap_config must be 18 or 32 (got %g).', flap_config);
            end
            Cmyu_clip = min(max(Cmyu, 0), obj.geom_ad.prop.Cmyu_max);
            Cm_ac_w = a + b .* sqrt(Cmyu_clip) + c_lin .* Cmyu_clip;
        end

        % Pitching moment about CG — Cm_total(flap_config, states_ac, aero_states, ctrl)
        % Uses the same post-stall blended lift as CL_total (wing via Ww, tail via Wr) in the
        % moment arms, so Cm stalls together with the lift (reduces to attached below stall).
        % Pitch-rate damping Cmq*q_hat (q_hat = q*c/(2V)) is ALWAYS included: q and V come from
        % the aircraft-state vector states_ac (same convention as CL_total) so no caller can
        % silently drop it; q = 0 (static trim) simply makes the damping term zero.
        function [Cm, comp] = Cm_total(obj, flap_config, states_ac, aero_states, ctrl)
            % states_ac = [theta, q, alphadot, V]; aero_states = [alpha, beta, Cmyu]
            % Optional 2nd output 'comp' = contributions (wing AC / wing lift / tail lift / pitch damp).
            % ctrl = [delta_e, delta_f, delta_a, delta_r]
            q = states_ac(2);  V = max(states_ac(4), 0.01);   % V floored to avoid /0 (matches CL_total)
            alpha = aero_states(1);  Cmyu = aero_states(3);
            delta_e = ctrl(1);  delta_f = ctrl(2);  delta_a = ctrl(3);
            s = obj.geom_ad.cl_stall;

            Cm0_ac_wing = obj.moment_about_aero_center(flap_config, Cmyu);
            x_ac_w      = obj.aero_center(flap_config, Cmyu);

            % Post-stall blended lift (same blends as CL_total: wing -> Ww, tail -> Wr)
            CL_w = obj.CL_wing(Cmyu, alpha, delta_f, delta_a);
            CL_t = obj.CL_tail(alpha, Cmyu, delta_e);
            Ww   = obj.CL_wing_stall_weight(alpha, Cmyu, flap_config);
            Wr   = obj.CL_rest_stall_weight(alpha, Cmyu);
            CL_w_post = (1 - Ww) .* CL_w + Ww .* s.wkflat .* sin(2 .* alpha);
            CL_t_post = (1 - Wr) .* CL_t + Wr .* s.rkflat .* (2 .* sign(alpha) .* sin(alpha).^2 .* cos(alpha));

            Cm_wing_ac   = Cm0_ac_wing;                                       % wing moment about its own AC
            Cm_wing_lift = CL_w_post .* (obj.geom_ac.cg.x_cg_c - x_ac_w);     % signed CG-to-AC arm (no abs: stays correct if CG moves ahead of the AC)
            Cm_tail_lift = -CL_t_post .* obj.geom_ac.cg.x_cg_tac_abs;         % tail lift about CG

            % Pitch-rate damping (CFD fit; q non-dimensionalised q_hat = q*c/(2V)); zero when q=0
            Cmq = (-329.962 + 101.129 .* sqrt(Cmyu))*0.1;   % fit to CFD pitch-rate damping (R2=0.9999), scaled by 0.3 to match flight data magnitude (see header)
            Cm_pitch_damp = Cmq .* (q .* obj.geom_ac.wing.c ./ (2 .* V));
            Cm = Cm_wing_ac + Cm_wing_lift + Cm_tail_lift + Cm_pitch_damp;

            comp = [];
            if nargout > 1
                comp = struct('wing_ac', Cm_wing_ac, 'wing_lift', Cm_wing_lift, ...
                              'tail_lift', Cm_tail_lift, 'pitch_damp', Cm_pitch_damp);
            end
        end

        %% aeroEval — full forward aero+propulsion evaluation at ONE condition.
        % Returns EVERY coefficient (CL,CD,Cm,CY,Cml,Cn) and the dimensional forces/
        % moments (L,D,Y, M_pitch,M_roll,M_yaw) plus thrust, with the dynamic (rate)
        % derivatives switched ON through the body rates p,q,r: q feeds CLq/Cmq/CDq,
        % p feeds Clp/Cnp/CYp/CDp, r feeds Clr/Cnr/CYr/CDr. This is the assembler that
        % runner_at_condition and the nDOF stepper otherwise build by hand; centralised
        % here so every caller sees the SAME forces.
        %   flap_config : pitch-fit selector  (aero.flapConfig(del_f))
        %   V           : airspeed [m/s];  alpha,beta : aero angles [rad]
        %   del_T       : throttle [0..1]  (sets thrust AND blowing Cmyu)
        %   ctrl        : [del_e, del_f, del_a, del_r] control deflections [rad]
        %   p,q,r       : body roll/pitch/yaw rates [rad/s] (drive the rate derivatives)
        %   alphadot    : AoA rate [rad/s] (feeds CLadot); optional, default 0
        %   J_max       : advance-ratio clamp for the prop fits; optional, default Inf (matches loads)
        function s = aeroEval(obj, flap_config, V, alpha, beta, del_T, ctrl, p, q, r, alphadot, J_max)
            if nargin < 11 || isempty(alphadot), alphadot = 0;   end
            if nargin < 12 || isempty(J_max),    J_max    = Inf; end

            del_e = ctrl(1);  del_f = ctrl(2);  del_a = ctrl(3);  del_r = ctrl(4);
            del_aL = -del_a;  del_aR = +del_a;       % antisymmetric aileron split (matches runner_at_condition / nDOF)

            S = obj.geom_ac.wing.S;  c = obj.geom_ac.wing.c;  b = obj.geom_ac.wing.b;
            rho = obj.atmos.rho;     W = obj.geom_ac.mass.W;   z_T = obj.geom_ac.prop.z_T;

            % Propulsion operating point (centralised thrust formula)
            n_prop = obj.n_prop(del_T);
            J      = min(obj.J(del_T, V), J_max);
            Mtip   = obj.Mtip(del_T);
            Cmyu   = obj.Cmyu_prop(J);
            CT     = obj.CT_prop(J, Mtip);
            T      = obj.thrustFromCT(n_prop, CT);

            % Input bundles (file convention). theta is unused by CL/Cm; on the
            % gamma=0 takeoff run theta = alpha, so pass alpha.
            states_ac   = [alpha, q, alphadot, V];
            aero_states = [alpha, beta, Cmyu];

            % Coefficients (rate terms ON via p,q,r)
            CL_w = obj.CL_wing(Cmyu, alpha, del_f, del_a);
            CL_t = obj.CL_tail(alpha, Cmyu, del_e);
            CL_f = obj.CL_fus(alpha, Cmyu);
            CL   = obj.CL_total(flap_config, states_ac, aero_states, ctrl);
            CD   = obj.CD_total(CL_w, CL_t, aero_states, ctrl, p, q, r);
            Cm   = obj.Cm_total(flap_config, states_ac, aero_states, ctrl);
            CY   = obj.CY_total(beta, del_aL, del_aR, del_r, del_f, Cmyu, alpha, p, r, V);
            Cml  = obj.Cml_total(beta, del_aL, del_aR, del_r, Cmyu, p, q, r, V);
            Cn   = obj.Cn_total(beta, del_aL, del_aR, del_r, del_f, Cmyu, alpha, p, r);

            % Dimensional forces & moments
            qbar = 0.5 * rho * V^2;   qS = qbar * S;
            L = qS*CL;   D = qS*CD;   Y = qS*CY;
            M_aero  = qS*c*Cm;                 % pure aero pitching moment about CG [N m]
            M_pitch = M_aero + T*z_T;           % total pitch about CG (adds thrust-line offset; matches loads.M_cg)
            M_roll  = qS*b*Cml;                 % rolling moment  [N m]
            M_yaw   = qS*b*Cn;                  % yawing moment   [N m]

            s = struct('V',V, 'alpha',alpha, 'beta',beta, 'p',p, 'q',q, 'r',r, ...
                       'Cmyu',Cmyu, 'J',J, 'Mtip',Mtip, 'CT',CT, 'T',T, 'TW',T/W, ...
                       'CL',CL, 'CL_w',CL_w, 'CL_t',CL_t, 'CL_f',CL_f, 'CD',CD, 'LD',L/max(D,eps), ...
                       'Cm',Cm, 'CY',CY, 'Cml',Cml, 'Cn',Cn, ...
                       'L',L, 'D',D, 'Y',Y, ...
                       'M_aero',M_aero, 'M_pitch',M_pitch, 'M_roll',M_roll, 'M_yaw',M_yaw, ...
                       'qbar',qbar, 'qS',qS);
        end

        % shared helper: aero coeffs about the design CG + lever arms to transfer to P
        %   P = [x_P, z_P]; states_ac = [theta, q, alphadot, V]
        %   aero_states = [alpha, beta, Cmyu]; ctrl = [delta_e, delta_f, delta_a, delta_r]
        function [Cm, CL, CD, CL_w, CL_t, CL_f, qbar, c, S, arm_horz, arm_vert] = ...
                transfer_setup(obj, P, flap_config, states_ac, aero_states, ctrl)
            theta = states_ac(1);              % pitch attitude [rad]
            V     = max(states_ac(4), 0.01);   % airspeed, floored to avoid /0 [m/s]
            alpha = aero_states(1);  Cmyu = aero_states(3);
            delta_e = ctrl(1);  delta_f = ctrl(2);  delta_a = ctrl(3);

            x_P = P(1);  z_P = P(2);
            c    = obj.geom_ac.wing.c;
            S    = obj.geom_ac.wing.S;
            qbar = 0.5 * obj.atmos.rho * V.^2;   % dynamic pressure [Pa]
            x_cg = obj.geom_ac.cg.x_cg_c * c;    % design CG x from LE [m]
            z_cg = obj.geom_ac.cg.z_cg;          % design CG z from A/C base [m]
            CL_w = obj.CL_wing(Cmyu, alpha, delta_f, delta_a);
            CL_t = obj.CL_tail(alpha, Cmyu, delta_e);
            CL_f = obj.CL_fus(alpha, Cmyu);
            CL   = CL_w + CL_t + CL_f;
            CD   = obj.CD_total(CL_w, CL_t, aero_states, ctrl);
            Cm   = obj.Cm_total(flap_config, states_ac, aero_states, ctrl);   % aero Cm about design CG (incl. pitch-rate damping)

            x_cg_P = x_cg - x_P;       % design CG relative to P, longitudinal [m]
            z_cg_P = z_cg - z_P;       % design CG relative to P, vertical     [m]
            % Rotate the body-fixed CG->P offset to earth axes by pitch theta (nose-up +).
            % Geometry frame is x = aft +, z = up +, so the nose-up rotation is [c s; -s c]:
            % the +z_cg_P (CG above P) swings AFT as the nose pitches up (inverted-pendulum sense).
            ct = cos(theta);  st = sin(theta);
            arm_horz = x_cg_P.*ct + z_cg_P.*st;    % horizontal lever (vertical forces: lift, weight)
            arm_vert = -x_cg_P.*st + z_cg_P.*ct;   % vertical   lever (horizontal forces: drag, thrust)
        end

        % Cm about a shifted CG (P = new CG): pure aero moment (wing+tail) transferred, no thrust
        %   P = [x_P, z_P]; states_ac = [theta, q, alphadot, V]
        %   aero_states = [alpha, beta, Cmyu]; ctrl = [delta_e, delta_f, delta_a, delta_r]
        function [Cm_P, CL, CD, Cm, CL_w, CL_t, CL_f] = Cm_about_shifted_cg(obj, P, flap_config, states_ac, aero_states, ctrl)
            [Cm, CL, CD, CL_w, CL_t, CL_f, ~, c, ~, arm_horz, arm_vert] = ...
                obj.transfer_setup(P, flap_config, states_ac, aero_states, ctrl);

            Cm_P = Cm + (-CL.*arm_horz + CD.*arm_vert) ./ c;   % aero transfer only (lift term: -CL*arm_horz)
        end

        % Cm about rotation point P (CG fixed in params): aero + thrust + weight + pseudo
        %   P = [x_P, z_P]; states_ac = [theta, q, alphadot, V]
        %   aero_states = [alpha, beta, Cmyu]; ctrl = [delta_e, delta_f, delta_a, delta_r]
        %   a_cg = [a_fwd, a_up]
        function [Cm_P, CL, CD, Cm, CL_w, CL_t, CL_f] = Cm_about_rotation_point(obj, P, flap_config, states_ac, aero_states, ctrl, T, W, a_cg)
            if nargin < 9 || isempty(a_cg), a_cg = [0, 0]; end   % optional CG accel -> no pseudo
            alpha = aero_states(1);
            g     = obj.geom_ac.mass.g;
            [Cm, CL, CD, CL_w, CL_t, CL_f, qbar, c, S, arm_horz, arm_vert] = ...
                obj.transfer_setup(P, flap_config, states_ac, aero_states, ctrl);

            a_fwd = a_cg(1);  a_up = a_cg(2);
            F_up  = T.*sin(alpha) - W - (W./g).*a_up;     % thrust + weight + pseudo (vertical)
            F_fwd = T.*cos(alpha)     - (W./g).*a_fwd;    % thrust + pseudo (forward)
            Cm_P = Cm + (-CL.*arm_horz + CD.*arm_vert) ./ c ...        % aero transfer (lift: -CL*arm_horz)
                      + (-F_up.*arm_horz - F_fwd.*arm_vert) ./ (qbar.*S.*c);   % thrust + weight + pseudo (vertical: -F_up*arm_horz)
        end

        %% Mass / inertia
        % inertia_about_LG — moments of inertia about the main-gear contact point,
        % obtained by parallel-axis shift of the CG inertias (geom_ac.mass.I_xx..I_xz)
        % for a given gear position. Inputs are the gear offsets from the CG:
        %   x_lg : longitudinal offset (aft +)  [m]
        %   z_lg : vertical offset    (down +)  [m]
        % (gear assumed on the centreline, y_lg = 0). Called with no offsets it
        % uses the stored geom_ac.lg position. Returns a struct of inertias about
        % the gear [kg m^2]; the .delta field is the change from the CG values.
        function I_LG = inertia_about_LG(obj, x_lg, z_lg)
            if nargin < 2 || isempty(x_lg), x_lg = obj.geom_ac.lg.x_lg; end
            if nargin < 3 || isempty(z_lg), z_lg = obj.geom_ac.lg.z_lg; end
            m = obj.geom_ac.mass.m;

            % Parallel-axis additions (change in inertia from CG to gear)
            I_LG.delta.I_xx = m .* z_lg.^2;
            I_LG.delta.I_yy = m .* (x_lg.^2 + z_lg.^2);
            I_LG.delta.I_zz = m .* x_lg.^2;
            I_LG.delta.I_xz = m .* x_lg .* z_lg;

            % Inertias about the gear = CG inertia + change
            I_LG.I_xx = obj.geom_ac.mass.I_xx + I_LG.delta.I_xx;
            I_LG.I_yy = obj.geom_ac.mass.I_yy + I_LG.delta.I_yy;
            I_LG.I_zz = obj.geom_ac.mass.I_zz + I_LG.delta.I_zz;
            I_LG.I_xz = obj.geom_ac.mass.I_xz + I_LG.delta.I_xz;
        end
    end
end
