
def index_of_str(s1, s2):
    lt=s1.split(s2,1)
    if len(lt) == 1:
        return -1
    return len(lt[0])

#读取原始文件
fd = open("lmk04610_original.txt", "r")
result = []
for line in fd.readlines():
    result.append(line.split('\n'))

#提取有效寄存器值
char_1='0x'
result_register_value = []
i = 0
file_handled=open('register_write_file.txt', mode='w', encoding='utf-8')
for item_string in result:
    nPos = index_of_str(item_string[0], char_1)
    result_register_value.append(item_string[0][nPos:])
    print(result_register_value[i])
    file_handled.writelines(result_register_value[i]+','+'\n')
    i += 1

#与之前的寄存器比较，并保存不同值的寄存器信息
fd_compare = open("compare.txt", "r")
register_compare = []
register_compare_temp = []
register_compare_inter_temp = []
j = 0
for line in fd_compare.readlines():
    register_compare_temp.append(line.split('\n'))
    register_compare_inter_temp.append(register_compare_temp[j][0].split(r'//'))
    register_compare.append(register_compare_inter_temp[j][0])
    j += 1

register_diff = []
i = 0
file_diff=open('different_register.txt', mode='w', encoding='utf-8')
file_update=open('updated_register.txt', mode='w', encoding='utf-8')
file_diff.writelines('New REG      Compared REG'+"\n")
for item_string in result:
    # print(register_compare[i])
    # print(result_register_value[i] + ',')
    if (register_compare[i]) != (result_register_value[i]+','):
        file_diff.writelines(result_register_value[i] + '     ' + register_compare_temp[i][0] + '\n')
        if len(register_compare_inter_temp[i]) > 1:
            file_update.writelines(result_register_value[i] + ',' + r'//' + register_compare_inter_temp[i][1] + '\n')
        else:
            file_update.writelines(result_register_value[i] + ',' + '\n')
    else:
        file_update.writelines(register_compare_temp[i][0] + '\n')
    i += 1
