def filter(board, kernel):
    # 传入原始棋盘与卷积核,返回处理后的棋盘
    import copy
    # 棋盘外围补0为卷积做准备
    board = copy.deepcopy(board)
    new_line = []
    for i in range(len(board[0])):
        new_line.append(0)
    board.append(copy.deepcopy(new_line))
    board.insert(0, new_line)
    for j in range(len(board)):
        board[j].insert(0, 0)
        board[j].append(0)
    j = 0
    for i in board:
        print(i)
        j += 1
        if j == len(board):
            print('')

    res = []
    for i in range(len(board) - 2):
        line = []
        for j in range(len(board[0]) - 2):
            value = 0
            for a in range(len(kernel)):
                for b in range(len(kernel[0])):
                    value += board[i + a][j + b] * kernel[a][b]
            line.append(value)
        res.append(line)

    j = 0
    for i in res:
        print(i)
        j += 1
        if j == len(res):
            print('')
    return res

board = [[0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 0]]
kernels = {
    'kernel_four_0': [[0, 0, 0, 0],
                      [1, 1, 1, 1],
                      [0, 0, 0, 0],
                      [0, 0, 0, 0]],
    'kernel_four_1': [[0, 0, 1, 0],
                      [0, 0, 1, 0],
                      [0, 0, 1, 0],
                      [0, 0, 1, 0]],
    'kernel_four_2': [[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]],
    'kernel_four_3': [[0, 0, 0, 1],
                      [0, 0, 1, 0],
                      [0, 1, 0, 0],
                      [1, 0, 0, 0]],
    'kernel_three_0': [[0, 0, 0],
                       [1, 1, 1],
                       [0, 0, 0]],
    'kernel_three_1': [[0, 1, 0],
                       [0, 1, 0],
                       [0, 1, 0]],
    'kernel_three_2': [[0, 0, 1],
                       [0, 1, 0],
                       [1, 0, 0]],
    'kernel_three_3': [[1, 0, 0],
                       [0, 1, 0],
                       [0, 0, 1]],
    'kernel_two_0': [[0, 0],
                     [1, 1]],
    'kernel_two_1': [[1, 0],
                     [1, 0]],
    'kernel_two_2': [[0, 1],
                     [1, 0]],
    'kernel_two_3': [[1, 0],
                     [0, 1]]}
filter(filter(board, kernels['kernel_three_0']), kernels['kernel_three_0'])
# filter(board, kernels['kernel_three_0'])