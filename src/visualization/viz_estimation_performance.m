figure
if b_use_ceif
    [plot_ceif_mean, plot_ceif_var] = analysis_ceif_.visualizePositionErrorWithMeanAndVariance(time_list, color_list.ceif, '-', 1.0);
end
if b_use_deif
    [plot_deif_mean, plot_deif_var] = analysis_deif_(1).visualizePositionErrorWithMeanAndVariance(time_list, color_list.deif, '-', 1.0);
end
set(gca,'FontSize',10);
ax = gca;
grid on
plots = [];
plot_names = {};
if (b_use_ceif)
    plots = horzcat(plots, [plot_ceif_mean, plot_ceif_var]);
    plot_names = horzcat(plot_names, {'CEIF (mean)', 'CEIF (variance)'});
end
if (b_use_deif)
    plots = horzcat(plots, [plot_deif_mean plot_deif_var]);
    plot_names = horzcat(plot_names, {'DEIF (mean)', 'DEIF (variance)'});
end
legend(plots, plot_names, 'Location', 'northwest', 'FontSize', 8);
xlabel('Time [sec]','FontSize',12)
ylabel('Estimation Error of Position [m]','FontSize',12)
hold off

figure
if b_use_ceif
    [plot_ceif_mean, plot_ceif_var] = analysis_ceif_.visualizeVelocityErrorWithMeanAndVariance(time_list, color_list.ceif, '-', 1.0);
end
if b_use_deif
    [plot_deif_mean, plot_deif_var] = analysis_deif_(1).visualizeVelocityErrorWithMeanAndVariance(time_list, color_list.deif, '-', 1.0);
end
set(gca,'FontSize',10);
ax = gca;
grid on
plots = [];
plot_names = {};
if (b_use_ceif)
    plots = horzcat(plots, [plot_ceif_mean, plot_ceif_var]);
    plot_names = horzcat(plot_names, {'CEIF (mean)', 'CEIF (variance)'});
end
if (b_use_deif)
    plots = horzcat(plots, [plot_deif_mean plot_deif_var]);
    plot_names = horzcat(plot_names, {'DEIF (mean)', 'DEIF (variance)'});
end
xlabel('Time [sec]','FontSize',12)
ylabel('Estimation Error of Velocity [m/s]','FontSize',12)
hold off