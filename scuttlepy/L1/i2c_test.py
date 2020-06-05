# importing the required module 
import timeit 

# code snippet to be executed only once 
mysetup = """from smbus2 import SMBus
bus = SMBus(1)
"""

# code snippet whose execution time is to be measured 
mycode = ''' 
pos = bus.read_i2c_block_data(0x40, 0xFE, 2)
 
'''

# timeit statement 
print(timeit.timeit(setup = mysetup, 
					stmt = mycode, 
					number = 1)) 






# import timeit
# from smbus2 import SMBus

# def test(): 

 
# starttime = timeit.default_timer()
# print("The start time is :",starttime)
# test()
# print("The time difference is :", timeit.default_timer() - starttime)