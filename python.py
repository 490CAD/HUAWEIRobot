n = int(input())
for i in range(1, n+1):
    for j in range(1, i+1):
        print(chr(65 +j-1),end='')
    print(' ' *(2*(n-i)+1), end='')
    for j in range(1,i+1):
        print(chr(65+j-1),end='')
    print()
    # print('\n')