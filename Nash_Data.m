% data = csvread('Test_Data_no_Words.csv')
r1 = data(1, :)
r2 = data(2, :)
r3 = data(3, :)
r4 = data(4, :)
r5 = data(5, :)
r6 = data(6, :)
r7 = data(7, :)
r8 = data(8, :)
r9 = data(9, :)
r10 = data(10, :)
r11 = data(11, :)
r12 = data(12, :)
r13 = data(13, :)
r14 = data(14, :)
r15 = data(15, :)

figure(1);
for i = 1:15
    plot(data(i, :))
    hold on
end