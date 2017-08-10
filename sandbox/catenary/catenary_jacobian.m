%% Calculation

clear all;
reset(symengine);

% Distributed load direction
syms ux uy uz real;
syms t0x t0y t0z real;
syms L real;
syms qs real;

% ux = 0;
% uy = 0;
% uz = -1;

assume(qs < 0);

% Preliminary variables
u = [ux; uy; uz];
q = qs*u;
t0 = [t0x; t0y; t0z];
I = eye(3);

tL = t0 - q*L;

rho_L = norm(t0 - q*L) - u' * (t0-q*L);
rho_0 = norm(t0) - u'*t0;

U = I - u*u';

% Evaluation of the quantity to be differentiated for jacobian computation
pc = (1/qs) * ( U*t0*log(rho_L/rho_0) - u*(norm(tL)-norm(t0)) );

%% Symbolic computation of the jacobian
Jac = [diff(pc, t0x), diff(pc, t0y), diff(pc, t0z)];
% Jac = Jac';
% pretty(Jac(1, 1))


%% Calcul analytique par moi...
tLn = norm(tL);
t0n = norm(t0);

f1 = tL / (tLn*rho_L) - t0 / (t0n*rho_0) + (1/rho_0 - 1/rho_L)*u ;
f2 = (t0/t0n - tL/tLn);

Jac_analytic = (1/qs) * (...
                    U * log(rho_L/rho_0)...
                  + (U*t0) * f1'...
                  + u * f2'...
);



% pretty(Jac_analytic(1, 2) - Jac_analytic(2, 1))


% pretty(Jac_analytic)

%% Manipulations
% jac_anal_11 = Jac_analytic(1, 1);
% % pretty(Jac_analytic(0, 0))
% 
% % Essai implementation J11
% x1 = (qs*L-t0z) / sqrt(t0x^2+t0y^2);
% x2 = t0z / sqrt(t0x^2+t0y^2);
% 
% j11 = (  log(x1 + sqrt(1+x1^2)) + log(x2 + sqrt(1+x2^2) ) )  / qs;
% pretty(simplify(j11-jac_anal_11))

%% Verification betwwen symbolic and analytical expressions
% Jac_diff = simplify(Jac-Jac_analytic);
% 
% disp('If analytical expression is correct, the following matrix must be null matrix:')
% pretty(Jac_diff)
% 
% if all(Jac_diff == 0)
%     disp('SUCCCESS !!')
% end

