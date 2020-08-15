import csv

csvfile = open('csv_test.csv')
data = []
for line in csvfile:
    row_data = line.strip("\n").split()
    for i, item in enumerate(row_data):
        print item
        item_ = []
        for j in range(len(item)):
            if item[j] != ',':
                item_.append(float(item[j]))
        row_data[i] = item_
    data.append(row_data)
print data
