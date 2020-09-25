record = controller.record_buffer;
tiledlayout(3,2)
legend_list = {'q1', 'q2', 'dq1_fil', 'dq2_fil', 'tau1', 'mode'};
for i = 1:6
    nexttile
    if i == 6
        k = 7
    else
        k = i
    end
    plot(record{8}, record{k});
    legend(legend_list{k})
end