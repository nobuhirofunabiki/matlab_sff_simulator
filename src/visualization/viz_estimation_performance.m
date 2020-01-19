figure
[plot_ceif_mean, plot_ceif_var] = analysis_ceif_.visualizePositionErrorWithMeanAndVariance(time_list, color_list.ceif, '-', 1.0);
[plot_deif_mean, plot_deif_var] = analysis_deif_(1).visualizePositionErrorWithMeanAndVariance(time_list, color_list.deif, '-', 1.0);
set(gca,'FontSize',10);
ax = gca;
grid on
legend([plot_ceif_mean plot_ceif_var plot_deif_mean plot_deif_var], ...
    {'Centralized EIF (mean)', 'Centralized EIF (variance)', ...
    'Decentralized EIF (mean)', 'Decentralized EIF (variance'}, ...
    'Location', 'northwest', 'FontSize', 8)
xlabel('Time [sec]','FontSize',12)
ylabel('Position estimation error [m]','FontSize',12)
hold off

figure
[plot_ceif_mean, plot_ceif_var] = analysis_ceif_.visualizeVelocityErrorWithMeanAndVariance(time_list, color_list.ceif, '-', 1.0);
[plot_deif_mean, plot_deif_var] = analysis_deif_(1).visualizeVelocityErrorWithMeanAndVariance(time_list, color_list.deif, '-', 1.0);
set(gca,'FontSize',10);
ax = gca;
grid on
legend([plot_ceif_mean plot_ceif_var plot_deif_mean plot_deif_var], ...
    {'Centralized EIF (mean)', 'Centralized EIF (variance)', ...
    'Decentralized EIF (mean)', 'Decentralized EIF (variance'}, ...
    'Location', 'northwest', 'FontSize', 8)
xlabel('Time [sec]','FontSize',12)
ylabel('Velocity estimation error [m/s]','FontSize',12)
hold off