%% Calculation

clear all;
reset(symengine);

% Distributed load direction
syms ux uy uz real;
syms t0x t0y t0z real;
syms L real;
syms qs real;

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


Jac_diff = simplify(Jac-Jac_analytic);

disp('If analytical expression is correct, the following matrix must be null matrix:')
pretty(Jac_diff)

if all(Jac_diff == 0)
    disp('SUCCCESS !!')
end

