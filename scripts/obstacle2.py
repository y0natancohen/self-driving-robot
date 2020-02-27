

def is_obstacle(front):
    last_far_left = 0
    thresh = 1

    for i, x in enumerate(front):
        if x > thresh:
            last_far_left = i
        else:
            break

    front_left = front[:last_far_left]
    last_close = last_far_left

    for i, x in enumerate(front[last_far_left + 1:]):
        if x < thresh:
            last_close = i + last_far_left + 1
        else:
            break

    last_far_right = last_close
    front_middle = front[last_far_left + 1: last_close]

    for i, x in enumerate(front[last_close + 1:]):
        if x > thresh:
            last_far_right = i + last_close + 1
        else:
            break

    front_right = front[last_close + 1:last_far_right + 1]
    # print len(l1), len(l2), len(l3)
    # noinspection PyChainedComparisons
    res = len(front_middle) > 15 and\
        len(front_middle) < 50 and\
        len(front_left) > 0 and\
        len(front_right) > 0 and\
        abs(front_left[-1] - front_middle[0]) > thresh and\
        abs(front_middle[-1] - front_right[0]) > thresh
    return res



