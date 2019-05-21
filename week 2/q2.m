A = [0 1;
    -1 -1.5];

[E, L] = eig(A);
X0 = [4;0];
t = 4;

eL = [exp(t*L(1,1)) 0;
    0 exp(t*L(2,2))];
X = E*eL*inv(E)*X0;

E_s = [E(:,1)/E(2,1) E(:,2)/E(2,2)];

Z = inv(E_s)*[2;-2];

S = E_s*[1 1i; 1 -1i];
A_tilde = inv(S)*A*S;

t2 = 2;
eL2 = [exp(t2*L(1,1)) 0;
    0 exp(t2*L(2,2))];
X2 = E*eL2*inv(E)*X0;
P = [2 0;
    0 2];

Energy = (norm(sqrt(P)*X2))^2;