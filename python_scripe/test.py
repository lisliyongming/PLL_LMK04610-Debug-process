
def index_of_str(s1, s2):
    lt=s1.split(s2,1)
    if len(lt)==1:
        return -1
    return len(lt[0])
print(index_of_str('12abc34de5f', 'c34'))
