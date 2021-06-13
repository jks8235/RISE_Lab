


def scaling(list_data):
    list_num = len(list_data)
    absoute_sum = 0
    # scalled_data = [0 for num in range(list_num)]
    for i in list_data:
        absoute_sum += abs(i)

    scalled_data = [float(list_data[num])/float(absoute_sum) for num in range(list_num)]

    return scalled_data