_l2 = [
        3.222689628601074,
        3.1874489784240723,
        3.134467840194702,
        3.095015287399292,
        3.062253713607788,
        3.0351147651672363,
        2.9774069786071777,
        2.9634313583374023,
        2.9265425205230713,
        2.905301332473755,
        2.858224630355835,
        2.8529977798461914,
        2.817168951034546,
        2.8101959228515625,
        2.767411708831787,
        2.728180170059204,
        2.7315187454223633,
        2.694674015045166,
        2.686429023742676,
        2.6865174770355225,
        2.702793598175049,
        2.8650076389312744,
        3.033005714416504,
        3.2471683025360107,
        3.448380470275879,
        999,
        999,
        0.6192561984062195,
        0.6290019154548645,
        0.6222659349441528,
        0.6258577704429626,
        0.6260972023010254,
        0.5978096127510071,
        0.6052793264389038,
        0.6191219687461853,
        0.6158697009086609,
        0.6205409169197083,
        0.6119375824928284,
        0.6080930233001709,
        0.6175583600997925,
        0.6091431975364685,
        0.6039846539497375,
        0.6240817308425903,
        0.6190201640129089,
        0.619379997253418,
        0.6099029183387756,
        0.6129356026649475,
        0.6068897843360901,
        0.6139773726463318,
        0.6117121577262878,
        0.6063617467880249,
        0.6139697432518005,
        0.6314571499824524,
        0.6103603839874268,
        0.653363049030304,
        0.6396254301071167,
        2.6371755599975586,
        2.656160593032837,
        2.6835482120513916,
        2.7119393348693848]

# count_l = 0
# count_r = 0
# count_m = 0

# last_l = 0
# # last_m = 0
# # last_r = 0
#
# thresh = 0.7
#
# for i, x in enumerate(l2):
#     # print i, x
#     if x > thresh:
#         # count_l += 1
#         last_l = i
#     else:
#         break
#         # finished_l = True
#
# print l2[:last_l]
# print last_l
# last_m = last_l
# # print l2[last_l+1:]
#
# for i, x in enumerate(l2[last_l+1:]):
#     # print i + last_l + 1, x
#     if x < thresh:
#         # count_m += 1
#         last_m = i + last_l + 1
#     else:
#         break
#
# print last_m
# last_r = last_m
# print l2[last_l+1: last_m]
#
# for i, x in enumerate(l2[last_m+1:]):
#     # print i + last_m + 1, x
#     if x > thresh:
#         # count_r += 1
#         last_r = i + last_m + 1
#     else:
#         break
#
# print last_r
#
# print l2[last_m+1:last_r+1]
#
# print l2[26]
# print l2[55]
# print l2[59]
# print count_l, count_m, count_r


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
    # if res:
        # print "obstacle found!!"
        # print front
        # print last_far_left, last_close, last_far_right
        # print front_left
        # print front_middle
        # print front_right
    return res



# print is_obstacle(_l2)
