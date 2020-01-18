
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

data = zeros(num_steps, 1+3*num_agents);
data(:,1) = transpose(time_list);
for iAgent = 1:num_agents
    data(:, 1+3*(iAgent-1)+1:1+3*(iAgent-1)+3) = ...
    transpose(v_agents_true_(iAgent).getPosition());
end   

dlmwrite(['../output/', 'node_positions.csv'], data, '-append');