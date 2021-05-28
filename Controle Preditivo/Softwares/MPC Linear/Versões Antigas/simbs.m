clear all; close all; clc;

N = 5;
Kf = 10;

syms A1 A2 A3 A4 A5 A6 A7 A8 A9 A10 B1 B2 B3 B4 B5 B6 B7 B8 B9 B10;
A = [A1 A2 A3 A4 A5 A6 A7 A8 A9 A10];
B = [B1 B2 B3 B4 B5 B6 B7 B8 B9 B10];

n = 1;
m = 1;

for k = 1 : Kf
    
    for col = 1 : N
        for lin = 1 : N
            if col <= lin
                S(lin*n-n+1:lin*n, col*m-m+1:col*m) = A(col)^(lin-col)*B(col);
            end
        end
    end
    
end
