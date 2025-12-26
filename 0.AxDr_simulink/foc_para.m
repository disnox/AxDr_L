%开关频率
SwitchFrequency = 20e3;
Ts = 1/SwitchFrequency;
Tpwm = 4000-1;


%mode
mode = 1;           % 0: 无感 1：有感

vbus = 24;          % 直流侧母线电压
vn = vbus/sqrt(2);  % 线电压有                                                                                                                  效


%电机参数
Np = 4;             % 极对数
Rs = 0.45;          % 定子电阻
Ls= 0.00042;        % 定子电感
Ld= 0.00042;        % 定子电感
Lq= 0.00042;        % 定子电感
Flux = 0.00525;   % 磁链
% Flux = 1.0e-6;
Jx = 0.000004483738;% 转动惯量
% Np = 14;             % 极对数
% Rs = 0.87;          % 定子电阻
% Ls= 0.000354;        % 定子电感
% Ld= 0.000354;        % 定子电感
% Lq= 0.000354;        % 定子电感
% Flux = 0.00488;   % 磁链
% Jx = 2.007053e-05;% 转动惯量
delta = 4;          % 阻尼因子
Fcoef = 0;
spd_max = 1000;
P = 120;
Te = P/(spd_max/Np);
KT = 1.5*Np*Flux;

% 电感辨识 注入的电压
vl = 1.5;

% CurrentLoop PI
CurrentLoopBandwidth = 500*2*pi;
id_kp = Ls*CurrentLoopBandwidth; 
id_ki = Rs*CurrentLoopBandwidth;
vd_limit = vbus/sqrt(3);

iq_kp = Ls*CurrentLoopBandwidth;
iq_ki = Rs*CurrentLoopBandwidth;
vq_limit = vbus/sqrt(3);
CurrentLoopMax = vbus/sqrt(3);

% SpeedLoop PI
K = (3.0*Np*Flux) / (4.0 * Jx);
% spd_kp = iq_kp/(Ls*delta*K)/9.55;
% spd_ki = iq_ki/(delta*delta*Ls)/9.55/1000;

spd_kp = (iq_kp/Ls)/(delta*K)/9.55;
spd_ki = ((iq_kp/Ls)*(iq_kp/Ls))/(delta*delta*delta*K);

iq_limit = 50;
SpdLoopMax = 50;

% PosLoop PI
pos_kp = 54;
pos_ki = 0;
speed_limit = 100;

% smo 滑模观测器
smo_wn = 2*pi*Ts;
smo_freq = 100;
Kslf = smo_freq*smo_wn;
Fsmopos = 1.0 - Ts * Rs / Ls;
Gsmopos = Ts / Ls;
Kslide = 10000.865;
E0 = 1000.5;

smo_pll_wn = 50*2*pi;

smo_pll_kp = 2*smo_pll_wn;
smo_pll_ki = smo_pll_wn*smo_pll_wn;
smo_pll_out_max = 333*pi*2;

% 磁链观测器
% Nonlinear flux PLL
pll_wn = 50*2*pi;

pll_kp = 2*pll_wn;
pll_ki = pll_wn*pll_wn;
pll_out_max = 333*2*pi*2;

obs_gain  = 500;
% bw_factor = 1/(Flux*Flux);
% gamma_now = 0.5*obs_gain*bw_factor;

gamma_now =10*(Flux*Flux);

% 静态电压补偿
alpha0 = 1000;
% lamda1 = 0.8;
% lamda2 = 1;



lpf_freq = 100;
lpf_coef = 1/(1+lpf_freq*6.28*Ts);

% active flux


flux_currentloop_bandwidth = 50*2*pi;
active_gain = flux_currentloop_bandwidth*Ls;
active_gain2 = 50000*Ls;

% 高频方波注入
vdh = 0.5;
% 正交锁相环
hfsi_ab_pll_wn = 500*2*pi;
hfsi_ab_pll_kp = 2*hfsi_ab_pll_wn;
hfsi_ab_pll_ki = hfsi_ab_pll_wn*hfsi_ab_pll_wn;
hfsi_ab_pll_out_max = 500*2*pi*2;
% 锁相环
hfsi_pll_wn = 50*2*pi;
hfsi_pll_kp = 2*hfsi_pll_wn;
hfsi_pll_ki = hfsi_pll_wn*hfsi_pll_wn;
hfsi_pll_out_max = 333*2*pi*2;

delta1 = 0.2;
delta_spd = 200;
inertia = (0.001079*Np)/(delta_spd*Np);


%% 弱磁控制算法V1
Ki_fw = 90;
%%弱磁控制算法V2
fw_kp = 0.0005;
fw_ki = 0.001;
Ld_Lq=Ld-Lq;   
%%  电流电压极限值设置
i_max = 50;             %电流极限值设定为50A
i_s = 50;               %可以人为设置更小
u_max = vbus/sqrt(3); 
u_s = vbus/sqrt(3);         %输出最大电压矢量幅值为u_max，即六扇区内接圆半径，调制比m=1

i_dmax = Flux/Ld;     %深度弱磁时，电机最大去磁电流，负d轴施加

%% 基速wA 临界速度wB A点电磁转矩计算
i_dA = (-Flux + sqrt(Flux^2 + 8*(Ld - Lq)^2*i_s^2))/4/(Ld - Lq);
i_qA = sqrt(i_s^2 - i_dA^2);



T_eA = 1.5*Np*((Ld-Lq)*i_dA + Flux)*i_qA;

w_A = u_max/sqrt((Ld*i_dA + Flux)^2+(Lq*i_qA)^2);
w_B = u_max / Flux;

w_AList = linspace(4.942e+02,800,200);
i_dAB=[];
i_qAB=[];
for i=1:length(w_AList)
    i_dAB(i) = (-Flux*Ld+sqrt((Flux*Ld)^2-(Ld^2-Lq^2)*(Flux^2+Lq^2*i_s^2-u_s^2/w_AList(i)^2)))/(Ld^2-Lq^2);
    i_qAB(i) = sqrt(i_s^2 - i_dAB(i)^2);
end
    
%% MTPA计算式中预计算
Ld_Lq=Ld-Lq;                        %直交轴电感差值
a_1 = 4*(Flux^2-4*(Ld-Lq)^2);        %表示iq计算式中的一个系数


final_i_ref = 0;
% final_iq_ref;








