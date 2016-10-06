import os

# TODO remove IsBroken again as soon as issue #137 is fixed
import platform
IsBroken = platform.node() == 'aslw039050.ethz.ch'

if not IsBroken:
    from libnumpy_eigen import *
else:
    print ("Warning: numpy_eigen is known to be broken here. Not actually loading it.")
