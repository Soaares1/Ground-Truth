clc;
clear all;
close all;

% Read file in as a series of strings
fid = fopen('log_y1.txt', 'rb');
%fid = fopen('log_lidar_longo.txt', 'rb');
strings = textscan(fid, '%s', 'Delimiter', ';');
fclose(fid);
       
% Convert to doubles 
data = cellfun(@str2num, strings{1}, 'uni', false);
data = cat(1, data{:});

[row col] = size(data);

qtde = fix(row/360);

cont = 0;
for i=1:qtde
    for j=1:360
        cont = cont + 1;
        vetor(i,j) = data(cont,1); 
    end
end

soma = zeros(1,360);

for i=1:qtde
    for j=1:360
        soma(j) = soma(j) + vetor(i,j); 
    end
end

media = soma/qtde;

theta = linspace(0, pi, 360);

exc_cte = 0.1;

for i = 1:360
   if (i <= 90) % de 0 a pi/4
        r(i) = 4 / (cos(theta(i)));
        acceptance(i) = r(i) - exc_cte;
   elseif (i > 90) && (i <= 270) % de pi/4 a 3pi/4
        r(i) = 4 / (sin(theta(i)));
        acceptance(i) = r(i) - exc_cte;
   else % de 3pi/4 a pi
        r(i) = -4 / (cos(theta(i)));
        acceptance(i) = r(i) - exc_cte;
   end
end

figure;
polarplot(theta, r, 'b', ... % Wall
    pi/2, 2, 'rx', ... % Robot
    pi/2, 0.1, 'mx', ... % Camera
    pi/2, 0.001, 'kx', ... % LIDAR
    'LineWidth', 3.5, 'MarkerSize' , 10);
legend('Wall','Robot','Camera','LIDAR','Location','northeast')
%thetalim([-10 190]);
thetalim([0 180]);
rlim ([0 4*sqrt(2)]);
pax = gca;
pax.ThetaAxisUnits = 'radians';
set(gca,'FontSize',15,'GridAlpha',0.35)
title('Scene Schematic');
grid on;
grid minor;  

figure;
polarplot(theta, r, 'b', ...  % Wall
    theta, vetor(1,:), 'k.', ... % Lidar Points
    'LineWidth', 3.5, 'MarkerSize' , 10);
legend('Wall','LIDAR Points','Location','northeast')
thetalim([0 180]);
rlim ([0 4*sqrt(2)]);
pax = gca;
pax.ThetaAxisUnits = 'radians';
set(gca,'FontSize',15,'GridAlpha',0.35)
title('LIDAR data - Sample');
grid on;
grid minor;  

figure;
polarplot(theta, r, 'b', ...  % Wall
    theta, vetor(1,:), 'k.', ... % Lidar Points
    theta, acceptance, 'g-.', ... % Acceptance Zone
    'LineWidth', 3.5, 'MarkerSize' , 10);
legend('Wall','LIDAR Points','Acceptance Zone','Location','northeast')
thetalim([0 180]);
rlim ([0 4*sqrt(2)]);
pax = gca;
pax.ThetaAxisUnits = 'radians';
set(gca,'FontSize',15,'GridAlpha',0.35)
title('LIDAR data - Sample');
grid on;
grid minor;  

% figure;
% polarplot(theta, media, 'k.', theta, r, 'b', theta, exclusion, 'g--', 'LineWidth', 1);
% legend('LIDAR Points','Wall','Allowed zone')
% thetalim([0 180]);
% rlim ([0 4*sqrt(2)]);
% pax = gca;
% pax.ThetaAxisUnits = 'radians';
% title('LIDAR data - Average');
% grid on;
% grid minor; 
