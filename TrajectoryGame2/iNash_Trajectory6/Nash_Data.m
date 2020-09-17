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
           scatter(i, plot_data(i, j), 'HandleVisibility', showleg, 'DisplayName', 'Raw Data');  %, 'DisplayName', 'Raw Data');
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
plot(diff); title('Difference Plot from Nash Time Average');
ylabel('Difference in Nash Time');



%figure(1);
%plot_data([1:8], [1:12], data, 12);
%title('Centralized Computing Time to Nash Equilibrium for iNash Trajectory Algorithm')
%legend('No Static Obstacles, Robots Crossing Paths','Static Obstacles, Robots Crossing Paths','Static Obstacles, Robots Crossing Paths, Part II','No Static Obstacles, Minimal Crossing of Paths','Static Obstacles, Minimal Crossing of Paths','Larger Position Space, No Static Obstacles, Crossing Paths','Larger Position Space, Static Obstacles, Crossing Paths','Larger Position Space, No Obstacles, Minimal Crossing')
%xlabel('Number of Robots')
%ylabel('Time (seconds) to reach Nash Equilibrium')

%figure(2);
%plot_data([17:23], [1:12], data, 12);
%title('Centralized Computing Time to Nash Equilibrium for iNash Trajectory Algorithm, Averages of Trials')
%legend('Total Average','No Static Obstacles Average','Static Obstacles Average','Robots Crossing Paths Average','Minimal Crossing Paths Average','Smaller Position Space Average','Larger Position Space Average')
%xlabel('Number of Robots')
%ylabel('Time (seconds) to reach Nash Equilibrium')

%figure(3);
%plot_data([9:16], [1:12], data, 12);
%title('Decentralized Computing Time to Nash Equilibrium for iNash Trajectory Algorithm')
%legend('No Static Obstacles, Robots Crossing Paths','Static Obstacles, Robots Crossing Paths','Static Obstacles, Robots Crossing Paths, Part II','No Static Obstacles, Minimal Crossing of Paths','Static Obstacles, Minimal Crossing of Paths','Larger Position Space, No Static Obstacles, Crossing Paths','Larger Position Space, Static Obstacles, Crossing Paths','Larger Position Space, No Obstacles, Minimal Crossing')
%xlabel('Number of Robots')
%ylabel('Time (seconds) to reach Nash Equilibrium')

%figure(4);
%plot_data([24:30], [1:12], data, 12);
%title('Decentralized Computing Time to Nash Equilibrium for iNash Trajectory Algorithm, Averages of Trials')
%legend('Total Average','No Static Obstacles Average','Static Obstacles Average','Robots Crossing Paths Average','Minimal Crossing Paths Average','Smaller Position Space Average','Larger Position Space Average')
%xlabel('Number of Robots')
%ylabel('Time (seconds) to reach Nash Equilibrium')