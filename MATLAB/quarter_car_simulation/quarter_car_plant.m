function [ddyc, ddyw] = quarter_car_plant(yc, dyc, yw, dyw, F_act, Mc, Mw, ks, cs, kt, ct)
%QUARTER_CAR_PLANT  Equations of motion for 2-DOF quarter car model.
%
%  Used as a MATLAB Function block inside Simulink.
%  States:  yc  = car body vertical displacement   [m]
%           dyc = car body vertical velocity        [m/s]
%           yw  = wheel vertical displacement       [m]
%           dyw = wheel vertical velocity           [m/s]
%  Input:   F_act = actuator force (positive = push Mc upward) [N]
%  Params:  Mc, Mw [kg],  ks, kt [N/m],  cs, ct [N·s/m]
%
%  Equations:
%    Mc * ddyc =  F_act - ks*(yc-yw) - cs*(dyc-dyw)
%    Mw * ddyw = -F_act + ks*(yc-yw) + cs*(dyc-dyw) - kt*yw - ct*dyw

rel_disp = yc  - yw;
rel_vel  = dyc - dyw;

ddyc = ( F_act - ks*rel_disp - cs*rel_vel ) / Mc;
ddyw = (-F_act + ks*rel_disp + cs*rel_vel - kt*yw - ct*dyw) / Mw;
end
