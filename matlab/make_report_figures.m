%% Run the whole MATLAB part and save the two report screenshots
% Output: report_figures/matlab_ident.png   (compare plot from step1_identify)
%         report_figures/simulink_model.png (diagram of car_position_loop.slx)
% Copy both into report/figures/ and rebuild the report.
%
% The step scripts start with "clear; close all", so nothing is kept in
% variables between the steps here.

step1_identify;
if ~exist('report_figures', 'dir'), mkdir('report_figures'); end
exportgraphics(findobj('Type', 'figure', 'Name', 'Identification'), ...
               fullfile('report_figures', 'matlab_ident.png'), 'Resolution', 200);

step2_tune_pid;
step3_simulate;
build_simulink_model;
try
    print('-scar_position_loop', '-dpng', '-r200', fullfile('report_figures', 'simulink_model.png'));
catch
    saveas(get_param('car_position_loop', 'Handle'), fullfile('report_figures', 'simulink_model.png'));
end

fprintf('\nSaved report_figures/matlab_ident.png and report_figures/simulink_model.png\n');
