% The XOR Example - Batch-Mode Training
%
% Author: Marcelo Augusto Costa Fernandes
% DCA - CT - UFRN
% mfernandes@dca.ufrn.br

p = 3; % Number of inputs - 3 sonars
H = 16; % Number of hidden neurons
m = 2; % Number of output neurons - left and right wheels speed

mu = 7; % Learning rate
alpha = 0.000000001; % Momentum parameter

epoch = 1000000; % Number of iterations
MSEmin = 10e-20; % Minimal error threshold
%load('test_set')
%X = samples([1 2 3],:);
X = [3.0  3.0  3.0  2.0  1.0  0.2  3.0  3.0  3.0  3.0  0.2;  
     3.0  0.5  0.1  3.0  3.0  3.0  3.0  3.0  3.0  0.2  0.2; 
     3.0  3.0  3.0  3.0  3.0  3.0  2.0  1.0  0.2  0.2  3.0;]

D = [0.5  0.4  0.3  0.4  0.4  0.5  0.4  0.4 -0.2 -0.5  0.5;
     0.4  0.2  0.0  0.4  0.4 -0.2  0.4  0.4  0.5  0.5 -0.5;]
D = matmap(D);

[Wx,Wy,MSE]=trainMLP(p,H,m,mu,alpha,X,D,epoch,MSEmin);
semilogy(MSE);

save('weights','Wx','Wy')
Y = runMLP(X,Wx,Wy);
%disp(['Y = [' num2str(Y) ']']);
%disp(matdemap(D));
%disp(matdemap(Y));
matdemap(D)
matdemap(Y)


disp('<-Y')

