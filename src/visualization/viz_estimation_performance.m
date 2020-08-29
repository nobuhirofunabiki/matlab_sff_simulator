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
    if strcmp(language,'English')
        plot_names = horzcat(plot_names, {'CEIF (mean)', 'CEIF (variance)'});
    elseif strcmp(language,'Japanese')
        plot_names = horzcat(plot_names, {'中央集権型 (期待値)', '中央集権型 (分散)'});
    end
end
if (b_use_deif)
    plots = horzcat(plots, [plot_deif_mean plot_deif_var]);
    if strcmp(language,'English')
        plot_names = horzcat(plot_names, {'DEIF (mean)', 'DEIF (variance)'});
    elseif strcmp(language,'Japanese')
        plot_names = horzcat(plot_names, {'分散型 (期待値)', '分散型 (分散)'});
    end
end
legend(plots, plot_names, 'Location', 'northwest', 'FontSize', 8);
if strcmp(language,'English')
    xlabel('Time [sec]','FontSize',12)
    ylabel('Estimation Error of Position [m]','FontSize',12)
elseif strcmp(language,'Japanese')
    xlabel('経過時間 [sec]','FontSize',12)
    ylabel('相対位置推定誤差 [m]')
end
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
    if strcmp(language,'English')
        plot_names = horzcat(plot_names, {'CEIF (mean)', 'CEIF (variance)'});
    elseif strcmp(language,'Japanese')
        plot_names = horzcat(plot_names, {'中央集権型 (期待値)', '中央集権型 (分散)'});
    end
end
if (b_use_deif)
    plots = horzcat(plots, [plot_deif_mean plot_deif_var]);
    if strcmp(language,'English')
        plot_names = horzcat(plot_names, {'DEIF (mean)', 'DEIF (variance)'});
    elseif strcmp(language,'Japanese')
        plot_names = horzcat(plot_names, {'分散型 (期待値)', '分散型 (分散)'});
    end
end
legend(plots, plot_names, 'Location', 'northwest', 'FontSize', 8);
if strcmp(language,'English')
    xlabel('Time [sec]','FontSize',12)
    ylabel('Estimation Error of Velocity [m/s]','FontSize',12)
elseif strcmp(language,'Japanese')
    xlabel('経過時間 [sec]','FontSize',12)
    ylabel('相対速度推定誤差 [m/s]')
end
hold off