% InitCodes
Pmerg = 1250;
Pmerg0 = 850;
MPCrng = 2000;
RallNodes = []; VallN = []; Idnm = []; MrgD = []; MrgLD = [];
Intv_dnm = 20; VmrgZone = []; CarCnt = 0;

Df = 0.0;
%% DETECTORS

Block = 0;

L1 = Car.empty();  
LR = Car.empty(); 

L1(end+1,1) = Car(10000, 1, 0);
LR(end+1,1) = Car(20000, 2, Pmerg + 10);
% LR(end+1,1) = Car(20001, 2, LR(end).X - 510);
LR(end+1,1) = Car(20001, 2, LR(end).X - 1200);
LR(1).V = 0;

