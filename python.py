import functools
THING_VALUE = [0, 3000, 3200, 3400, 7100, 7800, 8300, 29000]
    

def workbench_cmp(x, y):
    if THING_VALUE[x] > THING_VALUE[y]:
        return -1
    if THING_VALUE[x] < THING_VALUE[y]:
        return 1
    return 0


a = [1, 2, 3, 4, 5, 6]
a = sorted(a, key=functools.cmp_to_key(workbench_cmp))
print(a)