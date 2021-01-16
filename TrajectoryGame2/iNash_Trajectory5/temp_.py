import csv

with open('csv_test.csv', 'w') as csvfile:
    writer = csv.writer(csvfile, delimiter = ',', quotechar = '|', quoting = csv.QUOTE_MINIMAL)
    writer.writerow([[1, 1], [2, 2], [3, 3], [4, 4]])
    writer.writerow([[5, 6], [7, 9], [4, 4], [5, 1]])


"""csvfile = open('csv_test.csv')
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
print data"""