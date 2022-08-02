clc;close all;clear all;
p1 = xlsread('nozzle.xlsx','value','B2');
T1 = xlsread('nozzle.xlsx','value','B3');
FT = xlsread('nozzle.xlsx','value','B4');
m_dot = xlsread('nozzle.xlsx','value','B5');
Alt = xlsread('nozzle.xlsx','value','B6');
g = xlsread('nozzle.xlsx','value','B7');
R = xlsread('nozzle.xlsx','value','B8');


if (1100>Alt) && (Alt<25000)
    T = -56.46;
    po = 1000*(22.65*exp(1.73-0.00157*Alt));
elseif Alt>=2500
    T = -131.21 + 0.00299*Alt;
    po = 1000*(2.488*((T+273.1)/216.6)^-11.388);
else
    T = 15.04 - 0.00649*Alt;
    po = 1000*(101.29*((T+273.1)/288.08)^5.256);
end


PR = po/p1;
PR2 = (po/p1)^((g-1)/g);
TT = (2*g*R*T1)/(g-1);
pt = ((2/(g+1))^(g/(g-1)))*2.068;
vt = sqrt((2*g*R*T1))/(g+1);
ve = sqrt(TT*(1-PR2));

if m_dot==0
    m_dot=FT/ve;
elseif FT==0
    FT = m_dot/ve;
else
    fprintf('推力或流量至少輸入一項')
end

Te = T1*(po/p1)^((g-1)/g);
ae = sqrt(g*R*Te);
Me = ve/ae;

TR = 3.5;
RTOD = 180/pi;
DTOR = pi/180;
P = [];


A = sqrt((g+1)/(g-1));
B = (g-1)/(g+1);
vPM = @(x) A*atan(sqrt(B*(x^2-1))) - atan(sqrt(x^2-1));


Tmax = 0.5*vPM(Me)*RTOD;
DT = (90-Tmax) - fix(90-Tmax);
T(1) = DT*DTOR;
n = Tmax*2;

for m = 2:n+1
    T(m) = (DT + (m-1))*DTOR;
    x_int = [1 1.01*Me];
    func = @(x) T(m) - vPM(x);
    M(m) = fzero(func,x_int);
    P(m) = 0 +TR*tan(T(m));
    RR(m) = -TR/P(m);
    LR(m) = tan(m)+asin(1/M(m));
    SL(m) = -RR(m);
end


P(1) = [];
l = length(P);

for j = 1:l
    P1 = [0 TR];
    P2 = [P(j) 0];
    plot(P2,P1,'k')
    hold on
    xlabel('Center Line')
    ylabel('Radius')
end
hold on;
LR(1) = []; R(1) = [];
SL(1) = [];
F = RR(m-1);

for c = 1:length(P)-1
    x(c) = (TR+SL(c)*P(c))/(SL(c)-F);
    y(c) = F*x(c)+TR;
    XP = [P(c) x(c)];
    YP = [0 y(c)];
    plot(XP,YP,'b');
end
hold on

TM = Tmax*DTOR;
xw(1) = (TR+SL(1)*P(1))/(SL(1)-tan(TM));
yw(1) = tan(TM)*xw(1)+TR;
XP2 = [P(l) xw];
YP2 = [P(2) yw];
plot(XP2,YP2,'g');


DTW = tan(TM)/(length(P)-1);
s(l) = tan(TM);
b(l) = TR;

for k = 2:length(P)-1
    s(k) = tan(TM)-(k-1)*DTW;
    b(k) = yw(k-1)-s(k)*xw(k-1);
    xw(k) = (b(k)+SL(k)*P(k))/(SL(k)-s(k));
    yw(k) = s(k)*xw(k)+b(k);
    XP3 = [x(k) xw(k)];
    YP3 = [y(k) yw(k)];
    plot(XP3,YP3,'r');
end
hold on


xf = (b(length(b))+SL(length(SL))*P(length(P)))/SL(length(SL));
yf = b(length(b));
XF = [P(length(P)) xf];
YF = [0 yf];
plot(XF,YF,'r');

xw = [0 xw];
yw = [TR yw];
Rthroat = TR;
Rexit = yw(length(yw));
AR = (Rthroat/Rexit)^2

xlswrite('nozzle.xlsx',transpose(xw),'characteristic','A1:A62');
xlswrite('nozzle.xlsx',transpose(yw),'characteristic','B1:B62');
