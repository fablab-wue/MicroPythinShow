print('- Module "hallo" - Anfang')

import gc

test = 42

def welt():
    print('Hallo Jochen')

def ram():
    gc.collect()
    print('RAM:', gc.mem_free(), 'Byte frei')


print('- Module "hallo" - Ende')