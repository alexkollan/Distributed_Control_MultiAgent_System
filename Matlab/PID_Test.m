clear all
clc


x0=[3 1]';
G=[ 0.6 -0.3; -0.71 -0.4];

C2=[1 0];
C1=[0 1];


Kp=.139;
Ki=.145;
Kd=.35; 

x=x0;
y=x0;
for k=1:20
	x= G*x  + Kp*C1*x +Ki*C2*x;
	y=[y x];
	end
	
	plot(y(2,:))















syms s

% Plant function G=1/(s^2+3s+1)  ti einai auto? :P profanws einai o
% "kanonas" pou leitourgei to sistima mas.. h kanw la8os?

num=1;  %apo pou prokuptei?
den=sym2poly(s^2+3*s+1); %apo pou prokuptei?

G=tf(num,den); %ti einai auto?

H=1;  %auto einai o "stoxos" mas?
step(feedback(G,H))
hold on

%%
Kp=139;
Ki=145;
Kd=35;



C=pid(Kp,Ki,Kd);
%edw evala egw ta alla eidh  term gia na dw tis diafores sto grapho
Cp=pid(Kp);
Cpi=pid(Kp,Ki);
Cpd=pid(Kp,Kd);


A=C*G;
B=Cp*G;
C=Cpi*G;
D=Cpd*G;

T=feedback(A,H);
%T=feedback(B,H);
%T=feedback(C,H);
%T=feedback(D,H);

step(T)
hold off