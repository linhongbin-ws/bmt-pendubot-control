load('/home/ben/code/bmt-pendubot-control/archive/lqr+pilco/lqr_pilco_data-2020-09-26-hz50_pendubot_13_H150.mat')
set(gcf, 'Position',  [100, 100, 500, 500])
t = tiledlayout(4,1,'TileSpacing','Compact','Padding','Compact')

start_index = 1;
end_index = 3000;
LineWidth_1 = 1.5;
LineWidth_2 = 0.8;
Color_1 = [0, 0.4470, 0.7410];
Color_2 = [0.8500, 0.3250, 0.0980];
FontSize = 12;
satuated_current = 2.5;
current_2_torque = 29.2e-3;
tor_ratio = 5;
desire_q2 = 720;
margin_rate = 0.2

% truncate data
for i=1:8
    record{i} = record{i}(start_index: end_index) 
end
for i=1:size(record{5},2)
    if abs(record{5}(i))>satuated_current
        record{5}(i) = satuated_current * sign(record{5}(i));
    end
    record{5}(i) = record{5}(i)* current_2_torque*tor_ratio;  
end
for i=1:size(record{7},2)
    if record{7}(i) == 1
        record{7}(i) =1;
    end
    if record{7}(i) == 2
        record{7}(i) =0;
    end
end
for i=1:size(record{2},2)
    record{2}(i) = record{2}(i) - record{1}(i); 
end

for i=1:size(record{1},2)
    record{2}(i) = record{2}(i)* 360 / pi;
    record{1}(i) = record{1}(i)* 360 / pi; 
end


% the first plot
nexttile
hold on
plot(record{8}, zeros(size(record{8})), '--','LineWidth',LineWidth_2, 'Color',Color_2)
plot(record{8}, record{1},'LineWidth',LineWidth_1, 'Color',Color_1)
legend('Desired', 'Actual', 'Location', 'northeast','Orientation','horizontal')
ylabel('\boldmath $q_1$ (Deg)' ,'Interpreter','latex') 
hold off
set(gca,'FontSize',FontSize)
set(gca,'fontname','times')
set(gca,'FontWeight','bold')
set(gca,'XTickLabel',[]);
ymax = max(record{1});
ymin = min(record{1});
range=ymax - ymin;
ylim([ymin-range*margin_rate, ymax+range*margin_rate])
set(gca,'box','on')

% the second plot
nexttile
hold on
plot(record{8}, ones(size(record{8}))*desire_q2, '--','LineWidth',LineWidth_2, 'Color',Color_2)
plot(record{8}, record{2} ,'LineWidth',LineWidth_1, 'Color',Color_1)
legend('Desired','Actual',  'Location', 'southeast','Orientation','horizontal')
ylabel('\boldmath $q_2$ (Deg)' ,'Interpreter','latex') 
hold off
set(gca,'FontSize',FontSize)
set(gca,'fontname','times')
set(gca,'FontWeight','bold')
set(gca,'XTickLabel',[]);
ymax = max(record{2});
ymin = min(record{2});
range=ymax - ymin;
ylim([ymin-range*margin_rate, ymax+range*margin_rate])
set(gca,'box','on')

% the 3th plot
nexttile
plot(record{8}, record{7},'LineWidth',LineWidth_1, 'Color',Color_1)
legend(sprintf('0: PILCO \n 1: LQR'), 'Location', 'southeast','Orientation','horizontal')
ylabel('Mode') 
set(gca,'FontSize',FontSize)
set(gca,'fontname','times')
set(gca, 'XLimSpec', 'tight');
set(gca,'FontWeight','bold')
set(gca,'XTickLabel',[]);
ymax = max(record{7});
ymin = min(record{7});
range=ymax - ymin;
ylim([ymin-range*margin_rate, ymax+range*margin_rate])
set(gca,'box','on')

% the 4rd plot
nexttile
plot(record{8}, record{5},'LineWidth',LineWidth_1, 'Color',Color_1)
ylabel('\boldmath $\tau_1$ (N.m)' ,'Interpreter','latex') 
ymax = max(record{5});
ymin = min(record{5});
range=ymax - ymin;
ylim([ymin-range*margin_rate, ymax+range*margin_rate])
xlabel('Time(s)')
set(gca,'FontSize',FontSize)
set(gca,'fontname','times')
set(gca,'FontWeight','bold')
set(gca,'box','on')

exportgraphics(t,'/home/ben/code/bmt-pendubot-control/archive/lqr+pilco/pilco-lqr.pdf','BackgroundColor','none')
