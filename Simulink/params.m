%paus = 100;     % ms vahe aerut�mmete vahel
%paus2 = 100;    % paus teise parda aeru jaoks
min_paus = 100; % minimaalne taastumists�kli pikkus
nurk = deg2rad(0);       % algnurk paadi liikumissuuna ja x-telje vahel (kraadides)
mb = 10800;        %paadi mass  (90 s�udja, 14 paat, 2 aer (x2)); �hikmass on 100 kg 

x_0 =0;   % algne x-kordinaat;
y_0 =0;   % algne y- koordinaat;

vx_0 =0;  % algkiirus x-telje suunal
vy_0 =0;  % algkiirus y-telje suhtes
v_nurk=0; %

x_OK = 100;  % soovitud l�ppkoordinaat x
y_OK = 0;  %soovitud l�ppkoordinaat y