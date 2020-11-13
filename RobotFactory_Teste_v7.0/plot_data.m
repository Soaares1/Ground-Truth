clc;
clear all;
close all;

% Read file in as a series of strings
fid = fopen('data.txt', 'rb');
%fid = fopen('log_lidar_longo.txt', 'rb');
strings = textscan(fid, '%s', 'Delimiter', ';');
fclose(fid);
       
% Convert to doubles 
data_all = cellfun(@str2num, strings{1}, 'uni', false);
data_all = cat(1, data_all{:});

posx_estim_s_mm = data_all(:,1); 
posy_estim_s_mm = data_all(:,2); 
posx_estim_c_mm = data_all(:,3); 
posy_estim_c_mm = data_all(:,4); 
ang_estimado = data_all(:,5); 
posx_real = data_all(:,6); 
posy_real = data_all(:,7); 
ang_real = data_all(:,8); 
posx_diferenca = data_all(:,9); 
posy_diferenca = data_all(:,10); 
ang_diferenca = data_all(:,11); 

% n = length(posx_estim_c_mm);
% 
% if (n < 1100)
%     fprintf('Menos de 1100 amostras, repetir coleta !')
% else 
%     ini = n - 1000;
% 
%     for i = ini:(n-1)
%         pos_x_novo(i-ini+1) = posx_estim_c_mm(i);
%         pos_y_novo(i-ini+1) = posy_estim_c_mm(i);
%         posx_real_novo(i-ini+1) = posx_real(i);
%         posy_real_novo(i-ini+1) = posy_real(i);
%         ang_estimado_novo(i-ini+1) = ang_estimado(i);
%         ang_real_novo(i-ini+1) = ang_real(i);
%     end
% 
%         AE_pos_x_novo = abs(pos_x_novo - posx_real_novo);
%         AE_pos_y_novo = abs(pos_y_novo - posy_real_novo);
%         AE_ang_estimado_novo = abs(ang_estimado_novo - ang_real_novo);
%         
%     for i = 1:1000
%        if (AE_ang_estimado_novo(i) > 350)
%             AE_ang_estimado_novo(i) = 360 - AE_ang_estimado_novo(i);
%        end
%     end
% 
%     AE_media_x = mean(AE_pos_x_novo);
%     AE_media_y = mean(AE_pos_y_novo);
%     AE_media_angle = mean(AE_ang_estimado_novo);
%     AE_std_x = std(AE_pos_x_novo);
%     AE_std_y = std(AE_pos_y_novo);
%     AE_std_angle = std(AE_ang_estimado_novo);
%     
%     media_x = mean(pos_x_novo);
%     media_y = mean(pos_y_novo);
%     media_angle = mean(ang_estimado_novo);
%     std_x = std(pos_x_novo);
%     std_y = std(pos_y_novo);
%     std_angle = std(ang_estimado_novo);
%     mode_x = mode(pos_x_novo);
%     mode_y = mode(pos_y_novo);
%     mode_angle = mode(ang_estimado_novo);
% 
%     fprintf('media_x:%d, media_y:%d, media_angle:%d, std_x:%d, std_y:%d, std_angle:%d, mode_x:%d, mode_y:%d, mode_angle:%d, AE_media_x:%d, AE_media_y:%d, AE_media_angle:%d, AE_std_x:%d, AE_std_y:%d, AE_std_angle:%d', media_x, media_y, media_angle, std_x, std_y, std_angle, mode_x, mode_y, mode_angle, AE_media_x, AE_media_y, AE_media_angle, AE_std_x, AE_std_y, AE_std_angle)
%     
%     figure;
%     subplot(1,3,1)
%     h1 = histogram(pos_x_novo); title('Histogram Position E (x coordinate)'); ylabel('Number of Samples'); xlabel('x [m]'); set(gca,'FontSize',15);
%     subplot(1,3,2)
%     h2 = histogram(pos_y_novo); title('Histogram Position E (y coordinate)'); ylabel('Number of Samples'); xlabel('y [m]'); set(gca,'FontSize',15);
%     subplot(1,3,3)
%     h3 = histogram(ang_estimado_novo); title('Histogram Position E (angle)'); ylabel('Number of Samples'); xlabel('angle [degrees]'); set(gca,'FontSize',15);
% end

%%

figure;

subplot(2,3,1)
plot(posx_real, 'k');
hold on;
%plot(posx_estim_s_mm, 'b');
plot(posx_estim_c_mm, 'r');
hold off;
legend('Posição real', 'Posição est. s/ MM','Posição est. c/ MM')
title('Coordenada X')
grid on
grid minor;

subplot(2,3,2)
plot(posy_real, 'k');
hold on;
%plot(posy_estim_s_mm, 'b');
plot(posy_estim_c_mm, 'r');
hold off;
legend('Posição real', 'Posição est. s/ MM','Posição est. c/ MM')
title('Coordenada Y')
grid on
grid minor;

subplot(2,3,3)
plot(ang_real);
hold on;
plot(ang_estimado);
hold off;
legend('Ângulo real','Ângulo estimado')
title('Ângulo')
grid on
grid minor;

subplot(2,3,4)
plot(posx_diferenca);
ylim([-0.005 0.2])
title('Erro absoluto - Coordenada X')
grid on
grid minor;

subplot(2,3,5)
plot(posy_diferenca);
ylim([-0.005 0.2])
title('Erro absoluto - Coordenada Y')
grid on
grid minor;

subplot(2,3,6)
plot(ang_diferenca);
title('Erro absoluto - Ângulo')
grid on
grid minor;

n = length(posx_estim_c_mm);
aux = 10;
for i = 1:(n-aux)
    pos_x_novo(i) = posx_estim_c_mm(i+aux);
    pos_y_novo(i) = posy_estim_c_mm(i+aux);
    posx_real_novo(i) = posx_real(i+aux);
    posy_real_novo(i) = posy_real(i+aux);
end

figure;
plot(posx_real_novo, posy_real_novo, 'b', 'LineWidth',1.5);
grid on;
grid minor;
hold on;
plot(pos_x_novo, pos_y_novo, 'r', 'LineWidth',1.5);
hold off;
title('');
xlabel('x (m)');
xlim([-4 4])
ylim([0 4])
ylabel('y (m)');
legend('Real Coordinates', 'Estimated Coordinates')
set(gca,'FontSize',20)

%%
n = length(posx_estim_c_mm);
figure;
plot(posx_real, 'b', 'LineWidth',1.5);
hold on;
%plot(posx_estim_s_mm, 'b');
plot(posx_estim_c_mm, 'r', 'LineWidth',1.5);
hold off;
legend('Real Values', 'Estimated Values')
grid on
grid minor;
xlim([0 n])
set(gca,'FontSize',20)
xlabel ('Sample')
ylabel ('X Coordinate value (m)')
%%
n = length(posy_estim_c_mm);
figure;
plot(posy_real, 'b', 'LineWidth',1.5);
hold on;
%plot(posx_estim_s_mm, 'b');
plot(posy_estim_c_mm, 'r', 'LineWidth',1.5);
hold off;
legend('Real Values', 'Estimated Values')
grid on
grid minor;
xlim([0 n])
set(gca,'FontSize',20)
xlabel ('Sample')
ylabel ('Y Coordinate value (m)')
%%
n = length(posx_diferenca);
figure;
plot(posx_diferenca, 'r', 'LineWidth',1.5);
legend('Absolute Error (X Coordinate)')
grid on
grid minor;
xlim([0 n])
ylim([-0.005 0.25])
set(gca,'FontSize',20)
xlabel ('Sample')
ylabel ('Absolute Error (m)')
%%
n = length(posy_diferenca);
figure;
plot(posy_diferenca, 'r', 'LineWidth',1.5);
legend('Absolute Error (Y Coordinate)')
grid on
grid minor;
xlim([0 n])
ylim([-0.005 0.25])
set(gca,'FontSize',20)
xlabel ('Sample')
ylabel ('Absolute Error (m)')
%%

n = length(ang_real);
aux = 80;
for i = 1:(n-aux)
    ang_estimado_novo(i) = ang_estimado(i+aux);
    ang_real_novo(i) = ang_real(i+aux);
end

n = length(ang_estimado_novo);
figure;
plot(ang_real_novo, 'b', 'LineWidth',1.5);
hold on;
%plot(posx_estim_s_mm, 'b');
plot(ang_estimado_novo, 'r', 'LineWidth',1.5);
hold off;
legend('Real Values', 'Estimated Values')
grid on
grid minor;
xlim([0 n])
set(gca,'FontSize',20)
xlabel ('Sample')
ylabel ('Values (degrees)')
%%
n = length(ang_diferenca);
aux = 80;
for i = 1:(n-aux)
    ang_diferenca_novo(i) = ang_diferenca(i+aux);
end

n = length(ang_diferenca_novo);
figure;
plot(ang_diferenca_novo, 'r', 'LineWidth',1.5);
legend('Absolute Error (Angle)')
grid on
grid minor;
xlim([0 n])
ylim([-0.005 2*max(ang_diferenca_novo)])
set(gca,'FontSize',20)
xlabel ('Sample')
ylabel ('Absolute Error (degrees)')
%%

n = length(ang_real);
aux = 160;
for i = 1:(n-aux)
    ang_estimado_novo(i) = ang_estimado(i+aux);
    ang_real_novo(i) = ang_real(i+aux);
end

n = length(ang_estimado_novo);
figure;
plot(ang_real_novo, 'b', 'LineWidth',1.5);
hold on;
%plot(posx_estim_s_mm, 'b');
plot(ang_estimado_novo, 'r', 'LineWidth',1.5);
hold off;
legend('Real Values', 'Estimated Values')
grid on
grid minor;
xlim([0 n])
set(gca,'FontSize',20)
xlabel ('Sample')
ylabel ('Values (degrees)')

n = length(ang_diferenca);
aux = 160;
for i = 1:(n-aux)
    ang_diferenca_novo(i) = ang_diferenca(i+aux);
end

n = length(ang_diferenca_novo);
figure;
plot(ang_diferenca_novo, 'r', 'LineWidth',1.5);
legend('Absolute Error (Angle)')
grid on
grid minor;
xlim([0 n])
ylim([-0.005 1.1*max(ang_diferenca_novo)])
set(gca,'FontSize',20)
xlabel ('Sample')
ylabel ('Absolute Error (degrees)')

