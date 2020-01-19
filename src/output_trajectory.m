
fid = fopen(['../output/', 'node_positions.csv'],'wt');
fprintf(fid,'%s,', 'Time [sec]');
for iAgents = 1:num_agents
    num = num2str(iAgents);
    str = {...
        strcat('sc',num,'.x [m]'),...
        strcat('sc',num,'.y [m]'),...
        strcat('sc',num,'.z [m]')};
    fprintf(fid, '%s, %s, %s,', str{:});
end
fprintf(fid,'\n');
fclose(fid);

% data = zeros(num_steps, 1+3*num_agents);
% data(:,1) = transpose(time_list);
% for iAgents = 1:num_agents
%     data(:, 1+3*(iAgents-1)+1:1+3*(iAgents-1)+3) = ...
%     transpose(v_agents_true_(iAgents).getPosition());
% end

data = [];
time_delta = 10;
for iSteps = 1:num_steps
    if (mod(time_list(iSteps), time_delta) == 0)
        delta_data = zeros(1,1+num_dims*num_agents);
        delta_data(1,1) = time_list(1,iSteps);
        for iAgents = 1:num_agents
            positions = transpose(v_agents_true_(iAgents).getPosition());
            delta_data(1, 1+1+num_dims*(iAgents-1):1+num_dims*iAgents) ...
                = positions(iSteps,:);
        end
        data = vertcat(data, delta_data);
    end
end

dlmwrite(['../output/', 'node_positions.csv'], data, '-append');