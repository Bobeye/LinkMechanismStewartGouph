function [lambdaAve,CpAve,lambdaCpMaxModelExt,cpMaxModelExt,lambdaAveExtended,cp_Lambda_EqAveExtended,parCpLambdaAve] = postProcessingCpLambda(WT,nn)
%
% Calculation of Cp/Lambda curve
% ------------------------------
%
% This function calculates Cp/Lambda if the experiment keeps a constant
% steady-state vgRef for 45 seconds, and repeats this over and over
% again for 900 seconds total, i.e. 20 times, or 20 steady-state points.
%
%                                       CESC, February 15, 2016, Mario GS
%
% -----------------------------------------------------------------------
%
% Inputs
%   WT structure
%   nn = number of WT
%
% Outputs
%   lambdaAve = 20 points, from avarage points, 45 sec
%   CpAve = 20 points, from avarage points, 45 sec
%   lambdaCpMaxModelExt = lambda of CpMax
%   cpMaxModelExt = Cpmax
%   lambdaAveExtended  = [minLambda-0.5:0.01:maxLambda+0.5]
%   cp_Lambda_EqAveExtended = Cp for lambdaAveExtended
%   parCpLambdaAve: parameters polynomial Cp = f(lambda)
%
% -----------------------------------------------------------------------


lambda = WT(nn).lambda;
Cp = WT(nn).Cp;
vgRefWT = WT(nn).vgRefWT;


% Average for each step

nSamples = length(lambda);

% intervals for calculations
initialintervalRef = 1;
for ii=1:nSamples
    valIniRef = vgRefWT(initialintervalRef);
    indIntervalRef = find(vgRefWT==valIniRef);
    nIndIntervalRef = length(indIntervalRef);
    nForAverage = round(nIndIntervalRef/2);
    indForAve_1(ii) = initialintervalRef + nForAverage - 1;
    indForAve_2(ii) = initialintervalRef + nIndIntervalRef - 1;
    initialintervalRef = indIntervalRef(1) + nIndIntervalRef;
    if initialintervalRef>=nSamples
        break;
    end
end

% averages
nWorkPoints = length(indForAve_2);
workPointsAve = [1:1:nWorkPoints];
for ii = 1:nWorkPoints
    lambdaAve(ii) = mean([lambda(indForAve_1(ii):indForAve_2(ii))]);
    CpAve(ii) = mean([Cp(indForAve_1(ii):indForAve_2(ii))]);
end

% -----------------------------------------------------------------------

% Polynomial and max

minL = min(lambdaAve);
maxL = max(lambdaAve);
lambdaAveExtended = [minL-0.5:0.01:maxL+0.5]; 
parCpLambdaAve = polyfit(lambdaAve,CpAve,4); 
cp_Lambda_EqAveExtended = polyval(parCpLambdaAve,lambdaAveExtended); % for plot model: plot(lambdaAveExtended,cp_Lambda_EqAveExtended,'-r'); 
[val,ind] = find(lambdaAveExtended>maxL+1 | lambdaAveExtended<minL); 
lambdaAveExtended(ind) = [];
cp_Lambda_EqAveExtended(ind) = [];
[cpMaxModelExt,indCpMaxModelExt] = max(cp_Lambda_EqAveExtended); % From Data Average.
lambdaCpMaxModelExt = lambdaAveExtended(indCpMaxModelExt);


% Plot Cp/lambda

figure;
plot(lambdaAveExtended,cp_Lambda_EqAveExtended,'-r'); 
hold on;
plot(lambdaAve,CpAve,'*b');
plot(lambdaCpMaxModelExt,cpMaxModelExt,'or','MarkerSize',8,'MarkerFaceColor','r');
xlabel('TSR lambda'); 
ylabel('Cp');
title(['Cp/lambda. lambdaOptEq = ',num2str(lambdaCpMaxModelExt,'%15.2f'), '. CpMaxEq = ',num2str(cpMaxModelExt,'%15.4f')]);
grid;
axis([min(lambdaAveExtended)*0.8 max(lambdaAveExtended)*1.2 max(min(cp_Lambda_EqAveExtended)*0.8,0) max(cp_Lambda_EqAveExtended)*1.2]);
