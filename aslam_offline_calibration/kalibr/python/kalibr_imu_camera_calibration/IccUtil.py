from sm import PlotCollection
import IccPlots as plots
import numpy as np
import pylab as pl
import sys
import subprocess
import yaml
import time
from matplotlib.backends.backend_pdf import PdfPages

def printErrorStatistics(cself, dest=sys.stdout):
    # Reprojection errors
    for cidx, cam in enumerate(cself.CameraChain.camList):
        if len(cam.allReprojectionErrors)>0:
            e2 = np.array([ rerr.evaluateError()for reprojectionErrors in cam.allReprojectionErrors for rerr in reprojectionErrors])
            print >> dest, "Reprojection error squarred (cam{0}):  mean {1}, median {2}, std: {3}".format(cidx, np.mean(e2), np.median(e2), np.std(e2) )
        else:
            print >> dest, "Reprojection error squarred (cam{0}):  no corners".format(cidx)
    
    for iidx, imu in enumerate(cself.ImuList):
        # Gyro errors
        e2 = np.array([ e.evaluateError() for e in imu.gyroErrors ])
        print >> dest, "Gyro error squarred (imu{0}):          mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2))
        # Accelerometer errors
        e2 = np.array([ e.evaluateError() for e in imu.accelErrors ])
        print >> dest, "Accelerometer error squarred (imu{0}): mean {1}, median {2}, std: {3}".format(iidx, np.mean(e2), np.median(e2), np.std(e2))

def printGravity(cself):
    print
    print "Gravity vector: (in target coordinates): [m/s^2]"
    print cself.gravityDv.toEuclidean()
    
def printResults(cself, withCov=False):
    nCams = len(cself.CameraChain.camList)
    for camNr in range(0,nCams):
        T = cself.CameraChain.getResultTrafoImuToCam(camNr)
        print
        print "Transformation T_cam{0}_imu0 (imu0 to cam{0}, T_ci): [m]".format(camNr)
        if withCov and camNr==0:
            print "\t quaternion: ", T.q(), " +- ", cself.std_trafo_ic[0:3]
            print "\t translation: ", T.t(), " +- ", cself.std_trafo_ic[3:]
        print T.T()
        
        if not cself.noTimeCalibration:
            print
            print "cam{0} to imu0 time: [s] (t_imu = t_cam + shift)".format(camNr)
            print cself.CameraChain.getResultTimeShift(camNr),
            
            if withCov:
                print " +- ", cself.std_times[camNr]
            else:
                print
            
def printBaselines(self):
    #print all baselines in the camera chain
    if nCams > 1:
        for camNr in range(0,nCams-1):
            T, baseline = cself.CameraChain.getResultBaseline(camNr, camNr+1)
            
            if cself.CameraChain.camList[camNr+1].T_extrinsic_fixed:
                isFixed = "(fixed to external data)"
            else:
                isFixed = ""
            
            print
            print "Baseline (cam{0} to cam{1}): [m] {2}".format(camNr, camNr+1, isFixed)
            print T.T()
            print baseline, "[m]"
    
   
def generateReport(cself, filename="report.pdf", showOnScreen=True):  
    figs = list()
    plotter = PlotCollection.PlotCollection("Calibration report")
   
    #plot imu stuff (if we have imus)
    for iidx, imu in enumerate(cself.ImuList):
        f = pl.figure(3010+iidx)
        plots.plotAngularVelocities(cself, iidx, fno=f.number, noShow=True)
        plotter.add_figure("imu{0}: angular velocities".format(iidx), f)
        figs.append(f)
        f = pl.figure(3020+iidx)
        plots.plotAccelerations(cself, iidx, fno=f.number, noShow=True)
        plotter.add_figure("imu{0}: accelerations".format(iidx), f)
        figs.append(f)
        f = pl.figure(3030+iidx)
        plots.plotAccelBias(cself, iidx, fno=f.number, noShow=True)
        plotter.add_figure("imu{0}: accelerometer bias".format(iidx), f)
        figs.append(f)
        f = pl.figure(3040+iidx)
        plots.plotAngularVelocityBias(cself, iidx, fno=f.number, noShow=True)
        plotter.add_figure("imu{0}: gyro bias".format(iidx), f)
        figs.append(f)
        f = pl.figure(3050+iidx)
        plots.plotGyroError(cself, iidx, fno=f.number, noShow=True)
        plotter.add_figure("imu{0}: angular velocity error".format(iidx), f)
        figs.append(f)
        f = pl.figure(3060+iidx)
        plots.plotAccelError(cself, iidx, fno=f.number, noShow=True)
        plotter.add_figure("imu{0}: acceleration error".format(iidx), f)
        figs.append(f)
    
    #plot cam stuff
    if cself.CameraChain:        
        for cidx, cam in enumerate(cself.CameraChain.camList):
            f = pl.figure(4000+cidx)
            title="cam{0}: reprojection errors".format(cidx);
            plots.plotReprojectionScatter(cself, cidx, fno=f.number, noShow=True, title=title)
            plotter.add_figure(title, f)
            figs.append(f)

    #write to pdf
    pdf=PdfPages(filename)
    for fig in figs:
        pdf.savefig(fig)
    pdf.close()

    if showOnScreen:
        plotter.show()

def saveResultTxt(cself, filename='cam_imu_result.txt'):
    
    f1=open(filename, 'w')
    
    print >> f1, "Calibration results"
    print >> f1, "==================="   
    printErrorStatistics(cself, f1)
  
    # Calibration results
    nCams = len(cself.CameraChain.camList)
    for camNr in range(0,nCams):
        T = cself.CameraChain.getResultTrafoImuToCam(camNr)
        print >> f1, ""
        print >> f1, "Transformation (cam{0}):".format(camNr)
        print >> f1, "-----------------------"
        print >> f1, "T_ci:  (imu to cam{0}): [m]".format(camNr)   
        print >> f1, T.T()
        print >> f1, ""
        print >> f1, "T_ic:  (cam{0} to imu): [m]".format(camNr)   
        print >> f1, T.inverse().T()
    
        # Time
        print >> f1, ""
        print >> f1, "timeshift cam{0} to imu0: [s] (t_imu = t_cam + shift)".format(camNr)
        print >> f1, cself.CameraChain.getResultTimeShift(camNr)
        print >> f1, ""

    #print all baselines in the camera chain
    if nCams > 1:
        print >> f1, "Baselines:"
        print >> f1, "----------"

        for camNr in range(0,nCams-1):
            T, baseline = cself.CameraChain.getResultBaseline(camNr, camNr+1)
            print >> f1, "Baseline (cam{0} to cam{1}): [m]".format(camNr, camNr+1)
            print >> f1, T.T()
            print >> f1, "baseline norm: ", baseline,  "[m]"
            print >> f1, ""
    
    # Gravity
    print >> f1, ""
    print >> f1, "Gravity vector in target coords: : [m/s^2]"
    print >> f1, cself.gravityDv.toEuclidean()
    
    print >> f1, ""
    print >> f1, ""
    print >> f1, "Calibration configuration"
    print >> f1, "========================="
    print >> f1, ""

    for camNr, cam in enumerate( cself.CameraChain.camList ):
        print >> f1, "cam{0}".format(camNr)
        print >> f1, "-----"
        cam.camConfig.printDetails(f1)
        cam.targetConfig.printDetails(f1)
        print >> f1, ""
    
	print >> f1, ""
    print >> f1, ""
    print >> f1, "IMU configuration"
    print >> f1, "================="
    print >> f1, ""
    cself.ImuList[0].imuConfig.printDetails(f1)
    
