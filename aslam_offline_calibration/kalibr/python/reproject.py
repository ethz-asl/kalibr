import numpy as np
import pylab as pl

reprojectionErrors = np.genfromtxt('../../../../reprojection_error/ReprojErrorcirclecalibvsold1.csv', delimiter=',')
print(reprojectionErrors.shape)
f = pl.figure(1)
f.suptitle("Reprojection Error Distribution")

pl.subplot(121)
pl.hist(reprojectionErrors[:,0], bins=100)
pl.grid('on')
pl.xlabel('Reprojection error in x(pixels)')
pl.ylabel('count')

pl.subplot(122)
pl.hist(reprojectionErrors[:,1], bins=100)
pl.grid('on')
pl.xlabel('Reprojection error in y(pixels)')
pl.ylabel('count')
pl.show()