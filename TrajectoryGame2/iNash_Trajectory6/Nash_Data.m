clc, clear, close all hidden

data = csvread('Nash_Data2.csv');
length_ = size(data); length = length_(1);
nash_times = data(:, 1);
robot_numbers = data(:, 2);
max_robot_number = max(robot_numbers);

plot_data = [];
for j = 1:max_robot_number
    test_num = 1;
    for i = 1:length
        if data(i, 2) == j
            plot_data(j, test_num) = data(i, 1);
            test_num = test_num + 1;
        end
    end
end

len_ = size(plot_data); len = len_(2); avg = [];
figure(1);
for i = 1:max_robot_number
    stop_ = 0;
    for j = 1:len
       if plot_data(i, j) ~= 0
           if i == 1 && j == 1
               showleg = 'on'
           elseif i == 1 && j == 2
               showleg = 'off'
           end
           if j == len
               stop = j;
           end
           scatter(i, plot_data(i, j), 'HandleVisibility', showleg, 'DisplayName', 'Raw Individual Trial Data');  %, 'DisplayName', 'Raw Data');
           hold on
       else
           stop = j - 1;
           break
       end
    end
    avg(i) = mean(plot_data(i, 1:stop));
end
plot([1:max_robot_number], avg, 'DisplayName', 'Average');
legend
title('Time to Reach Nash Equilibrium');
xlabel('Number of Robots'); ylabel('Time (Seconds)');

diff = [];
for i = 1:max_robot_number-1
    diff(i) = avg(i+1) - avg(i);
end
figure(2);
plot(diff); title('Difference of Nash Time Average Between Robot Numbers');
ylabel('Difference in Nash Time (seconds)');

diff2 = [];
for i = 1:max_robot_number-2
    diff2(i) = diff(i+1) - diff(i);
end
figure(3);
plot(diff2); title('Difference of Difference of Nash Time Average Between Robot Numbers');
ylabel('Difference of Difference in Nash Time (seconds)');

avg_avg = [];
for i = 2:max_robot_number-1
    avg_avg(i) = mean([avg(i-1), avg(i), avg(i+1)]);
end
figure(4);
plot(avg_avg); title('Nash Time Averaged Between 3 Consecutive Robot Numbers');
axis([5, max_robot_number, min(avg_avg), max(avg_avg)]);
ylabel('Average Nash Time (seconds)'); xlabel('Average Robot Number');

avg_avg_diff = [];
for i = 1:max_robot_number-2
    avg_avg_diff(i) = avg_avg(i+1) - avg_avg(i);
end
figure(5);
plot(avg_avg_diff); title('Difference of Averaged Nash Times, Between Averaged Robot Numbers');
ylabel('Difference in Averaged Nash Times');

avg_avg_diff_diff = [];

for i = 1:max_robot_number-3
    avg_avg_diff_diff(i) = avg_avg_diff(i+1) - avg_avg_diff(i);
end
figure(6);
plot(avg_avg_diff_diff); title('Second Difference Equation of Averaged Nash Times');
axis([1, max_robot_number-1, -5, 5]);
ylabel('Second Difference in Averaged Nash Times');
xlabel('Average Robot Number');
