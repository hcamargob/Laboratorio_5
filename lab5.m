%Letra H
clc;
tint=pi/2:0.1:3*pi/2;

xext=[0 300*cos(tint) 0 ];
yext=[300 300*sin(tint) -300 ];
zext=[10 -10*ones(1,length(xext)-2) 0 ];

xint=[0 195*cos(tint) 0 ];
yint=[195 195*sin(tint) -195 ];
zint=[10 -10*ones(1,length(xint)-2) 0 ];

Pint=[xint' yint' zint'];
Pext=[xext' yext' zext'];

x1=-189:8:-155;
x2=-158:8:-124;
x3=-172:8:-139;


m1=(156-197)/(-155+189);
b1=197-m1*(-189);
y1=m1.*x1+b1;
b2=226-m1*(-158);
y2=m1.*x2+b2;
m2=-1/m1;
b3=176-m2*(-172);
y3=m2.*x3+b3;

XH=[-189 x1 -157 -172 x3 -140 -158 x2 -126];
YH=[197 y1 158 176 y3 203 226 y2 187];
ZH=[30 -10*ones(1,length(x1)) 10 10 -10*ones(1,length(x3)) 10 10 -10*ones(1,length(x2)) 30];
PH=[XH' YH' ZH'];

%Letra S
ts1=3*pi/4:0.8:7*pi/4;
ts2=-pi/4:0.8:3*pi/4;
r1=sqrt((-214+236)^2+(131-152)^2)/2;

xs1=-236:5:(r1*sin(pi/4)-236);
xs1in=(r1*sin(pi/4)-236*2)-xs1-1.46+sin(pi/4);

ys1=152:5:(r1*sin(pi/4)+152);
ys1in=(r1*sin(pi/4)+2*152)-ys1-1.46+sin(pi/4);

xs2=r1*cos(ts1)-(214+236)/2;
ys2=r1*sin(ts1)+(131+152)/2;
xs3=r1*cos(pi/2-ts2)-(214+236)/2+(236-214);
ys3=r1*sin(pi/2-ts2)+(131+152)/2-(-131+152);

xs4=-191.8093-r1*sin(pi/4):5:-191.8093;
xs4in=-2*191.8093-xs4-r1*sin(pi/4)-1.46+sin(pi/4);

ys4=110.2035-r1*sin(pi/4):5:110.2035;
ys4in=2*110.2035-ys4-r1*sin(pi/4)-1.46+sin(pi/4);

XS=[-226 xs1in xs2 xs3 xs4in -202.5622];
YS=[162 ys1in ys2 ys3 ys4in 99.4506];
ZS=[30 -10*ones(1,length(XS)-2) 30];
PS=[XS' YS' ZS'];

XLet=[XS XH -250];
YLet=[YS YH 0];
ZLet=[ZS ZH 30];
PLet=[XLet' YLet' ZLet'];
%triangulo

xt1=-276:4:-256;
xt2=-256:4:-236;
xt3=-276:4:-236;

xt3in=-236-xt3-276;
yt1=tan(pi/3)*(xt1+276)+28;
yt2=-tan(pi/3)*(xt2+236)+28;
yt3=ones(1,length(xt3))*28;

XT=[-276 xt1 xt2 xt3in -276];
YT=[28 yt1 yt2 yt3 28];
ZT=[30 -10*ones(1,length(XT)-2) 30];
PT=[XT' YT' ZT'];

%CIRCULO

tc=0:0.2:2*pi;
XC=[-210 20*cos(tc)-230 -210];
YC=[0 20*sin(tc) 0];
ZC=[30 -10*ones(1,length(XC)-2) 30];
PC=[XC' YC' ZC'];

%Paralelas

xp1=-260:10:-230;
yp1=tan(pi/6)*(xp1+260)-47;
yp2=tan(pi/6)*(xp1+260)-67;
yp3=tan(pi/6)*(xp1+260)-87;

XP=[-260 xp1 -230 -260 xp1 -230 -260 xp1 -230];
YP=[yp1(1) yp1 yp1(length(yp1)) yp2(1) yp2 yp2(length(yp3)) yp3(1) yp3 yp3(length(yp3))];
ZP=[30 -10*ones(1,length(xp1)) 30 30 -10*ones(1,length(xp1)) 30 30 -10*ones(1,length(xp1)) 30];

PP=[XP' YP' ZP'];

XFIG=[XT XC XP -250];
YFIG=[YT YC YP 0];
ZFIG=[ZT ZC ZP 30];

PFIG=[XFIG' YFIG' ZFIG'];
%Trebol

ttr=0:0.2:2*pi;
XTR=[-136 cos(ttr).*7.*(cos(5*ttr)+5)-178 -136 ];
YTR=[-171 sin(ttr).*7.*(cos(5*ttr)+5)-171 -171 ];
ZTR=[30 -11*ones(1,length(ttr)) 30 ];

PTR=[XTR' YTR' ZTR'];

%puntos

tp=0:2*pi/5:2*pi;
xpui=-230+30*cos(tp);
ypui=-110+30*sin(tp);

XPui=[xpui(1) xpui(1) xpui(1) xpui(2) xpui(2) xpui(2) xpui(3) xpui(3) xpui(3) xpui(4) xpui(4) xpui(4) xpui(5) xpui(5) xpui(5) -250];
YPui=[ypui(1) ypui(1) ypui(1) ypui(2) ypui(2) ypui(2) ypui(3) ypui(3) ypui(3) ypui(4) ypui(4) ypui(4) ypui(5) ypui(5) ypui(5) 0];
ZPui=[30 -10 30 30 -10 30 30 -10 30 30 -10 30 30 -10 30 30];

PPui=[XPui' YPui' ZPui'];

plot(XH,YH)
hold on
plot(XS,YS)
plot(XT,YT)
plot(XC,YC)
plot(xp1,yp1)
plot(xp1,yp2)
plot(xp1,yp3)
plot(XTR,YTR)
plot(xext,yext)
plot(xint,yint)
plot(xpui,ypui)

l1=45;l2=105;l3=105;l4=100;

p=PTM;
  
x=p(:,1);y=p(:,2);z=p(:,3)-l1;
theta1=atan(y./x);
r=sqrt(x.^2+y.^2);
theta3=acos((z.^2+(r-l4).^2-l2^2-l3^2)/(2*l2*l3));
theta2=pi/2-atan(z./(r-l4))-atan((l3.*sin(theta3))./(l2+l3.*cos(theta3)));
theta2=-theta2;
theta3=-theta3;
theta4=-theta2-theta3-pi/2;
theta5=-2*ones(length(xtm)+1,1);
theta2=theta2+(8*pi/180);

q=[theta1 theta2 theta3 theta4 theta5];
n=size(q);

fprintf('[')
for i = 1:n(1)
    if i==n(1)
        fprintf('[%.4f,%.4f,%.4f,%.4f,%.4f]]\n',theta1(i), theta2(i), theta3(i), theta4(i), theta5(i))
    else
        fprintf('[%.4f,%.4f,%.4f,%.4f,%.4f],\n',theta1(i), theta2(i), theta3(i), theta4(i), theta5(i))
    end
end

