from __future__ import print_function #handle print in 2.x python
import sm
from sm import PlotCollection
from kalibr_common import ConfigReader as cr
import aslam_cv as acv
import aslam_cameras_april as acv_april
import aslam_cv_backend as acvb
import aslam_backend as aopt
import incremental_calibration as ic
import kalibr_camera_calibration as kcc

from matplotlib.backends.backend_pdf import PdfPages
import io
try:
    # Python 2
    from cStringIO import StringIO
except ImportError:
    # Python 3
    from io import StringIO
import matplotlib.patches as patches
import mpl_toolkits.mplot3d.axes3d as p3
import cv2
import numpy as np
import pylab as pl
import math
import gc
import sys

# make numpy print prettier
np.set_printoptions(suppress=True)


def normalize(v):
    return v / np.linalg.norm(v)

def getImageCenterRay(cself, cam_id):
    # Get the ray of the image center
    geometry = cself.cameras[cam_id].geometry
    projection = geometry.projection()
    yc = np.array( [ projection.cu(), projection.cv() ] )
    vc = normalize(geometry.keypointToEuclidean(yc))
    return vc;

class pointStatistics(object):
    pass

def getPointStatistics(cself,view_id,cam_id,pidx):
    view = cself.views[view_id]
    
    #check if the given camera sees the target in the given view (and get the obs)
    #view.rig_observations = list of tuples (cam_id, obs)
    cams_in_view = [obs_tuple[0] for obs_tuple in view.rig_observations]
    if cam_id not in cams_in_view:
        rval = pointStatistics()
        rval.valid = False
        return rval
    obs = view.rig_observations[ cams_in_view.index(cam_id) ][1]
    
    vc = getImageCenterRay(cself, cam_id)
    z = np.array([0,0,1])
    dz = np.dot(z,vc)
    if np.abs(dz - 1.0) > 1e-3:
        print("The point statistics are only valid if the camera points down the z axis. This camera has the image center: [%f, %f, %f]" % (z[0],z[1],z[2]))
    valid, y = obs.imagePoint(pidx)
    geometry = cself.cameras[cam_id].geometry
    rerr = None
    yhat = None
    azumithalAngle = None
    polarAngle = None
    squaredError = None
    e = None
    
    if valid:
        #translate point index to error term index (pidx!=eidx if not all points are observed!)
        pidx_observed = obs.getCornersIdx()
        eidx=list(pidx_observed).index(pidx)
    
        rerr = view.errorTerm(eidx)
        e = rerr.error()
        yhat = y - e
        v = normalize(geometry.keypointToEuclidean(y))
        # Note, these calculations assume that the camera point
        # down the z-axis. If this is wrong...this is bad.
        polarAngle = math.acos(v[2])
        azumithalAngle = math.atan2(v[1],v[0])
        # Make sure to recompute the error just in case
        # The point has been updated
        rerr.evaluateError()
        squaredError = rerr.getRawSquaredError()
    rval = pointStatistics()
    rval.valid = valid
    rval.view_id = view_id
    rval.camid = cam_id
    rval.pidx = pidx
    rval.y = y
    rval.yhat = yhat
    rval.azumithalAngle = azumithalAngle
    rval.polarAngle = polarAngle
    rval.squaredError = squaredError
    rval.e = e
    return rval
    
def getReprojectionErrorStatistics(all_rerrs):
    """
    usage:  all_corners, all_reprojections, all_reprojection_errs = getReprojectionErrors(calibrator, 0)
            mean, std = getReprojectionErrorStatistics(all_reprojections)
    """
    if not len(all_rerrs)>0:
        raise RuntimeError("rerrs has invalid dimension")

    gc.disable() #append speed up
    rerr_matrix=list();
    for view_id, view_rerrs in enumerate(all_rerrs):
        if view_rerrs is not None: #if cam sees target in this view
            for rerr in view_rerrs:
                if not (rerr==np.array([None,None])).all(): #if corner was observed
                    rerr_matrix.append(rerr)
    
    
    rerr_matrix = np.array(rerr_matrix)
    gc.enable()
    
    mean = np.mean(rerr_matrix, 0, dtype=np.float64)
    std = np.std(rerr_matrix, 0, dtype=np.float64)
 
    return mean, std

#return a list (views) of a list (reprojection errors) for a cam
def getReprojectionErrors(cself, cam_id):
    all_corners = list(); all_reprojections = list(); all_reprojection_errs =list()
    
    gc.disable() #append speed up
    for view_id, view in enumerate(cself.views):
        if cam_id in list(view.rerrs.keys()):
            view_corners=list(); view_reprojections=list(); view_reprojection_errs=list()
            for rerr in view.rerrs[cam_id]:
                #add if the corners was observed
                if rerr is not None:
                    corner = rerr.getMeasurement()
                    reprojection = rerr.getPredictedMeasurement()
                    err = corner-reprojection
                else:
                    corner = np.array([None, None])
                    reprojection = np.array([None, None])
                    err = np.array([None, None])
                    
                view_corners.append(corner)
                view_reprojections.append(reprojection)
                view_reprojection_errs.append(err)
                    
            all_corners.append(np.array(view_corners))
            all_reprojections.append(np.array(view_reprojections))
            all_reprojection_errs.append(np.array(view_reprojection_errs))
        else:
            #the camera doesn't see the target in this view
            all_corners.append(None)
            all_reprojections.append(None)
            all_reprojection_errs.append(None)

    gc.enable()
    return all_corners, all_reprojections, all_reprojection_errs
  

#get statistics for one cam over all points
def getAllPointStatistics(cself, cam_id):
    gc.disable() #gc slows down appends... disable it temporarily
    stats = []
    for view_id in range(0,len(cself.views)):
        for p_idx in range(0,cself.target.target.size()):
            stat = getPointStatistics(cself,view_id,cam_id,p_idx)
            if stat.valid:
                stats.append(stat)
    gc.enable()
    return stats

def plotPolarError(cself, cam_id, fno=1, clearFigure=True, stats=None, noShow=False, title=""):
    if stats is None:
        stats = getAllPointStatistics(cself, cam_id)
    angleError = np.array([ [ np.degrees(s.polarAngle), math.sqrt(s.squaredError)] for s in stats ])
    # sort by polar angle
    sae = angleError[ angleError[:,0].argsort() ]
    
    # Now plot
    f = pl.figure(fno)
    if clearFigure:
        f.clf()
    f.suptitle(title)
        
    pl.subplot(121)
    pl.plot(sae[:,0],sae[:,1],'bx-')
    pl.grid('on')
    pl.xlabel('polar angle (deg)')
    pl.ylabel('reprojection error (pixels)')
    pl.subplot(122)
    pl.hist(sae[:,0])
    pl.grid('on')
    pl.xlabel('polar angle (deg)')
    pl.ylabel('count')
    if not noShow:
        pl.show()

def plotAzumithalError(cself, cam_id, fno=1, clearFigure=True, stats=None, noShow=False, title=""):
    if stats is None:
        stats = getAllPointStatistics(cself, cam_id)
    angleError = np.array([ [ np.degrees(s.azumithalAngle), math.sqrt(s.squaredError)] for s in stats ])
    # sort by azimuthal angle
    sae = angleError[ angleError[:,0].argsort() ]
    # Now plot
    f = pl.figure(fno)
    if clearFigure:
        f.clf()
    f.suptitle(title)
    
    pl.subplot(121)
    pl.plot(sae[:,0],sae[:,1],'bx-')
    pl.grid('on')
    pl.xlabel('azimuthal angle (deg)')
    pl.ylabel('reprojection error (pixels)')
    pl.subplot(122)
    pl.hist(sae[:,0])
    pl.grid('on')
    pl.xlabel('azimuthal angle (deg)')
    pl.ylabel('count')
    if not noShow:
        pl.show()

def plotAllReprojectionErrors(cself, cam_id, fno=1, noShow=False, clearFigure=True, title=""):
    # left: observations and projecitons
    # right: scatterplot of reprojection errors
    all_corners, reprojections, rerrs_xy = getReprojectionErrors(cself, cam_id)
    resolution = (cself.cameras[cam_id].geometry.projection().ru(), cself.cameras[cam_id].geometry.projection().rv())

    #create figure
    f = pl.figure(fno)
    if clearFigure:    
        f.clf()
    f.suptitle(title)
    
    values = np.arange(len(cself.views))/np.double(len(cself.views))
    cmap = pl.cm.jet(values,alpha=0.5)
    
    #detected corners plot
    a=pl.subplot(121)
    for view_id, corners in enumerate(all_corners):
        if corners is not None: #if this camerea sees the target in this view
            color = cmap[view_id,:]
            pl.plot(corners[:,0], corners[:,1],'o-', mfc=color, c=color, mec=color)

    #add an empty image to force the aspect ratio
    I=np.zeros((resolution[1], resolution[0]))
    pl.imshow(I, cmap='Greys')

    #reprojection errors scatter plot
    sub = pl.subplot(122)
    for view_id, rerrs in enumerate(rerrs_xy):
        if rerrs is not None: #if this camerea sees the target in this view
            color = cmap[view_id,:]
            pl.plot(rerrs[:,0], rerrs[:,1], 'x', lw=3, mew=3, color=color)

    pl.axis('equal')
    pl.grid('on')
    pl.xlabel('error x (pix)')
    pl.ylabel('error y (pix)')

    SM = pl.cm.ScalarMappable(pl.cm.colors.Normalize(0.0,len(cself.views)), pl.cm.jet)
    SM.set_array(np.arange(len(cself.views)));
    cb = pl.colorbar(SM)
    cb.set_label('image index')
    if not noShow:
        pl.show()

def plotCornersAndReprojection(gridobs, reprojs, fno=1, cornerlist=None, clearFigure=True, plotImage=True, color=None, title=""):
    #create figure
    f = pl.figure(fno)
    if clearFigure:    
        f.clf()
    f.suptitle(title)
    
    plotCorners(gridobs, fno=fno, clearFigure=False, plotImage=True, cornerlist=cornerlist)
    
    #plot reprojection
    if color is None:
        color = [1.0,0.0,0.0,0.5]
        
    # Get the reprojections:
    for pidx, reproj in enumerate(reprojs):
        if (cornerlist is not None) and (pidx in cornerlist):
            if (not np.all(reproj==np.array([None,None]))):
                pl.plot(reproj[0], reproj[1], 'x', lw=3, mew=3, color=color)

    pl.xlim([0,gridobs.imCols()])
    pl.ylim([gridobs.imRows(),0])
    if clearFigure:
        pl.legend(['observation', 'reprojection'])

def recoverCovariance(cself):
    #Covariance ordering (=dv ordering)
    #ORDERING:   N=num cams
    #            1. all baselines (q, t) -> 6 * (N-1)
    #            2. camera -->  sum(sub) * N
    #                a) distortion --> 4
    #                b) projection --> omni:5, pinhole: 4
    #                c) shutter    --> 0
    
    numCams = len(cself.cameras)
    est_stds = np.sqrt(cself.estimator.getSigma2Theta().diagonal())

    #split the variance for baselines
    baseline_cov = est_stds[0:6*(numCams-1)]
    std_baselines = np.array(baseline_cov).reshape(numCams-1,6).tolist()
    
    #split camera cov
    cam_cov = est_stds[6*(numCams-1):]
    std_cameras = list()
    
    offset=0
    for cidx, cam in enumerate(cself.cameras):
        nt = cam.geometry.minimalDimensionsDistortion() +  \
             cam.geometry.minimalDimensionsProjection() +  \
             cam.geometry.minimalDimensionsShutter()
        
        std_cameras.append( cam_cov[offset:offset+nt].flatten().tolist() )
        offset = offset+nt
    
    return std_baselines, std_cameras

def saveChainParametersYaml(cself, resultFile, graph):
    cameraModels = {acvb.DistortedPinhole: 'pinhole',
                    acvb.EquidistantPinhole: 'pinhole',
                    acvb.FovPinhole: 'pinhole',
                    acvb.Omni: 'omni',
                    acvb.DistortedOmni: 'omni',
                    acvb.ExtendedUnified: 'eucm',
                    acvb.DoubleSphere: 'ds'}
    distortionModels = {acvb.DistortedPinhole: 'radtan',
                        acvb.EquidistantPinhole: 'equidistant',
                        acvb.FovPinhole: 'fov',
                        acvb.Omni: 'none',
                        acvb.DistortedOmni: 'radtan',
                        acvb.ExtendedUnified: 'none',
                        acvb.DoubleSphere: 'none'}

    chain = cr.CameraChainParameters(resultFile, createYaml=True)
    for cam_id, cam in enumerate(cself.cameras):
        cameraModel = cameraModels[cam.model]
        distortionModel = distortionModels[cam.model]

        #create new config file
        camParams = cr.CameraParameters(resultFile, createYaml=True)
        camParams.setRosTopic(cam.dataset.topic)

        #set the data
        P = cam.geometry.projection()
        if cameraModel == 'omni':
            camParams.setIntrinsics(cameraModel, [P.xi(), P.fu(), P.fv(), P.cu(), P.cv()] )
        elif cameraModel == 'pinhole':
            camParams.setIntrinsics(cameraModel, [P.fu(), P.fv(), P.cu(), P.cv()] )
        elif cameraModel == 'eucm':
            camParams.setIntrinsics(cameraModel, [P.alpha(), P.beta(), P.fu(), P.fv(), P.cu(), P.cv()] )
        elif cameraModel == 'ds':
            camParams.setIntrinsics(cameraModel, [P.xi(), P.alpha(), P.fu(), P.fv(), P.cu(), P.cv()] )
        else:
            raise RuntimeError("Invalid camera model {}.".format(cameraModel))
        camParams.setResolution( [P.ru(), P.rv()] )
        dist_coeffs = P.distortion().getParameters().flatten()
        camParams.setDistortion( distortionModel, dist_coeffs)

        chain.addCameraAtEnd(camParams)

    for cam_id, cam in enumerate(cself.cameras):
        overlaps = graph.getCamOverlaps(cam_id)
        chain.setCamOverlaps(cam_id, overlaps)
        
    #print all baselines in the camera chain
    for bidx, baseline_dv in enumerate(cself.baselines):
        baseline = sm.Transformation( baseline_dv.T() )
        camNr = bidx+1
        chain.setExtrinsicsLastCamToHere(camNr, baseline)

    chain.writeYaml()


def plotOutlierCorners(cself, removedOutlierCorners, fno=1, clearFigure=True, title=""):
    #create figure
    f = pl.figure(fno)
    if clearFigure:    
        f.clf()
    f.suptitle(title)

    #create subplot for each camera
    for cidx, camera in enumerate(cself.cameras):
        #extract corners for this cam
        corners=list()
        for cam_id, removed_corners in removedOutlierCorners:
            if cam_id==cidx:
                corners.append(removed_corners)
        corners=np.array(corners)
        
        #plot
        subplot_rows = int(np.ceil( np.sqrt(len(cself.cameras)) ))
        subplot_cols = int(np.ceil( np.sqrt(len(cself.cameras)) ))
        pl.subplot(subplot_rows, subplot_cols, cidx+1)
        pl.title("cam{0}".format(cidx))
            
        if len(corners)>0:
            pl.plot(corners[:,0], corners[:,1], 'rx')
            
        #add an empty image to force the aspect ratio
        resolution = (camera.geometry.projection().ru(), camera.geometry.projection().rv())
        I=np.zeros((resolution[1], resolution[0]))
        pl.imshow(I, cmap='Greys')
        

def generateReport(cself, filename="report.pdf", showOnScreen=True, graph=None, removedOutlierCorners=None):
    
    #plotter
    figs = list()
    plotter = PlotCollection.PlotCollection("Calibration report")
    offset = 3010
    
    #Output calibration results in text form.
    sstream = StringIO()
    printParameters(cself, sstream)
    text = [line for line in StringIO(sstream.getvalue())]
    linesPerPage = 35
    
    while True:
        fig = pl.figure(offset)
        offset += 1

        left, width = .05, 1.
        bottom, height = -.05, 1.
        right = left + width
        top = bottom + height
        
        ax = fig.add_axes([.0, .0, 1., 1.])
        # axes coordinates are 0,0 is bottom left and 1,1 is upper right
        p = patches.Rectangle((left, bottom), width, height, fill=False, transform=ax.transAxes, \
                                 clip_on=False, edgecolor="none")
        ax.add_patch(p)
        pl.axis('off')

        printText = lambda t: ax.text(left, top, t, fontsize=7, \
                                     horizontalalignment='left', verticalalignment='top',\
                                     transform=ax.transAxes, wrap=True)
        
        if len(text) > linesPerPage:
            printText("".join(text[0:linesPerPage]))
            figs.append(fig)
            text = text[linesPerPage:]
        else:
            printText("".join(text[0:]))
            figs.append(fig)
            break

    
    #plot graph
    if graph is not None:
        f=pl.figure(1001)
        title="Inter-camera observations graph (edge weight=#mutual obs.)";
        graph.plotGraphPylab(fno=f.number, noShow=True, title=title)
        plotter.add_figure("Obs. graph", f)
        figs.append(f)
        
    #rig geometry
    if len(cself.cameras)>1:
        f=pl.figure(1002)
        title="camera system"
        plotCameraRig(cself.baselines, fno=f.number, clearFigure=False, title=title)
        plotter.add_figure(title, f)
        figs.append(f)
    
    #plot trajectory
    if len(cself.views)>2:
        f=pl.figure(1003)
        title="cam0: estimated poses"
        plotTrajectory(cself, fno=f.number, clearFigure=False, title=title)
        plotter.add_figure(title, f)
        figs.append(f)
        
    #plot for each camera
    for cidx, cam in enumerate(cself.cameras):
        f = pl.figure(cidx*10+1)
        title="cam{0}: polar error".format(cidx)
        plotPolarError(cself, cidx, fno=f.number, noShow=True, title=title)
        plotter.add_figure(title, f)
        figs.append(f)
        f = pl.figure(cidx*10+2)
        title="cam{0}: azimuthal error".format(cidx)
        plotAzumithalError(cself, cidx, fno=f.number, noShow=True, title=title)
        plotter.add_figure(title, f)
        figs.append(f)
        f = pl.figure(cidx*10+3)
        title="cam{0}: reprojection errors".format(cidx)
        plotAllReprojectionErrors(cself, cidx, fno=f.number, noShow=True, title=title)
        plotter.add_figure(title, f)
        figs.append(f)
    
    #plot all removed outlier corners
    if removedOutlierCorners is not None:
        if len(removedOutlierCorners) > 0:
            f=pl.figure(1004)
            title="Location of removed outlier corners"
            plotOutlierCorners(cself, removedOutlierCorners, fno=f.number, title=title)
            plotter.add_figure("Outlier corners", f)
            figs.append(f)
            
    #save to pdf
    pdf=PdfPages(filename)
    for fig in figs:
        pdf.savefig(fig)
    pdf.close()
    
    if showOnScreen:
        plotter.show()  
    
def plotCorners(gridobs, fno=1, cornerlist=None, clearFigure=True, plotImage=True, color=None, subplot=0):
    if color is None:
        color = [0,1,1,0.3];
    f = pl.figure(fno)
    if subplot>0:
        pl.subplot(subplot)
    if clearFigure:
        f.clf()
    if plotImage:
        I = gridobs.getImage()
        if len(I.shape) == 2 and I.shape[0] > 0 and I.shape[1] > 0:
            pl.imshow(I,cmap=pl.cm.gray)

    #only plot certain corners as an option
    if cornerlist is not None:
        P=list()
        for pidx in cornerlist:
            valid, y = gridobs.imagePoint(pidx)
            if valid:
                P.append(y)
            else:
                print("Tried to plot unobserved corner")       
        P=np.array(P)
    else:
        #get all points
        P=gridobs.getCornersImageFrame()
    
    pl.plot(P[:,0], P[:,1],'o-', mfc=color, c=color, mec=color)
    pl.xlim([0,gridobs.imCols()])
    pl.ylim([gridobs.imRows(),0])


def plotTrajectory(cself, fno=1, clearFigure=True, title=""):
    f = pl.figure(fno)
    if clearFigure:
        f.clf()
    f.suptitle(title)

    size = 0.05
    a3d = f.add_subplot(111, projection='3d')
    
    #get the list of camera poses from our problem
    traj_max = np.array([-9999.0, -9999.0, -9999.0])
    traj_min = np.array([9999.0, 9999.0, 9999.0])
    views = sorted(cself.views, key=lambda x: x.timestamp)
    T_target_camera_last = None;
    for view in views:
        # get this view in target frame
        T_target_camera = sm.Transformation(view.dv_T_target_camera.T())
        sm.plotCoordinateFrame(a3d, T_target_camera.T(), size=size)
        # record min max
        traj_max = np.maximum(traj_max, T_target_camera.t())
        traj_min = np.minimum(traj_min, T_target_camera.t())
        # compute relative change between
        if T_target_camera_last != None:
            pos1 = T_target_camera_last.t()
            pos2 = T_target_camera.t()
            a3d.plot3D([pos1[0], pos2[0]],[pos1[1], pos2[1]],[pos1[2], pos2[2]],'k-', linewidth=1)
        T_target_camera_last = T_target_camera;

    #TODO: should also plot the target board here!

    a3d.auto_scale_xyz([traj_min[0]-size, traj_max[0]+size], [traj_min[1]-size, traj_max[1]+size], [traj_min[2]-size, traj_max[2]+size])

def plotCameraRig(baselines, fno=1, clearFigure=True, title=""):
    f = pl.figure(fno)
    if clearFigure:
        f.clf()
    f.suptitle(title)
    
    #convet DVs to trafo
    temp=list()
    for baseline in baselines:
        temp.append(sm.Transformation(baseline.T()))
    baselines=temp
        
    #convert baselines to camera frames in coords of first camera
    camera_frames=list()
    camera_frames.append(sm.Transformation()) #first cam
    for bidx, baseline in enumerate(baselines):
        camera_frames.append( camera_frames[bidx]*baseline.inverse() )
    
    #get the of the coord system (half the shortest baseline)
    sizes=list()
    for baseline in baselines:
        sizes.append( np.linalg.norm(baseline.t()) )
    size=0.5*np.min(sizes)
    
    #plot each frame in coordinates of first camera
    a3d = f.add_subplot(111, projection='3d')
    for i, camera_frame in enumerate(camera_frames):
        sm.plotCoordinateFrame(a3d, camera_frame.T(), size=size, name="cam{0}".format(i))

    #axis equal
    box_sizes=list()
    for camera_frame in camera_frames:
        box_sizes.append( np.linalg.norm(camera_frame.t()) )
    box_size=1.25*np.max(box_sizes)+size
    
    a3d.auto_scale_xyz([-box_size, box_size], [-box_size, box_size], [-box_size, box_size])
    a3d.set_xlabel('x')
    a3d.set_ylabel('y')
    a3d.set_zlabel('z')


def exportPoses(cself, filename):
    
    # Append our header, and select times at IMU rate
    f = open(filename, 'w')
    print("#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []", file=f)
    
    # Times are in nanoseconds -> convert to seconds
    # Use the ETH groundtruth csv format [t,q,p,v,bg,ba]
    views = sorted(cself.views, key=lambda x: x.timestamp)
    for view in views:
        T_target_camera = sm.Transformation(view.dv_T_target_camera.T())
        position = T_target_camera.t()
        orientation = T_target_camera.q()
        print("{:.0f},".format(1e9 * view.timestamp) + ",".join(map("{:.6f}".format, position)) \
               + "," + ",".join(map("{:.6f}".format, orientation)) , file=f)

def saveResultTxt(cself, filename="camera_calibration_result.txt"):
    f1=open(filename, 'w')
    printParameters(cself, f1)

def printParameters(cself, dest=sys.stdout):

    print("Calibration results ", file=dest)
    print("====================", file=dest)

    #get the covariances
    std_baselines, std_cameras = recoverCovariance(cself)

    #print cameras
    print("Camera-system parameters:", file=dest)
    for cidx, cam in enumerate(cself.cameras):
        d = cam.geometry.projection().distortion().getParameters().flatten()
        p = cam.geometry.projection().getParameters().flatten()
        dd = std_cameras[cidx][0:d.shape[0]]
        dp = std_cameras[cidx][d.shape[0]:]
        print("cam{0} ({1}):".format(cidx, cam.dataset.topic), file=dest) 
        print("    type: %s" % ( type(cam.geometry) ), file=dest) 
        print("    distortion: %s +- %s" % (d, np.array(dd)), file=dest)
        print("    projection: %s +- %s" % (p, np.array(dp)), file=dest)
        
        #reproj error statistics
        corners, reprojs, rerrs = getReprojectionErrors(cself, cidx)        
        if len(rerrs)>0:
            me, se = getReprojectionErrorStatistics(rerrs)
            try:
              print("    reprojection error: [%f, %f] +- [%f, %f]" % (me[0], me[1], se[0], se[1]), file=dest)
            except:
              print("    failed printing the reprojection error!", file=dest)
            print(file=dest)

    #print baselines
    for bidx, baseline in enumerate(cself.baselines):
        T = sm.Transformation( baseline.T() )
        dq = std_baselines[bidx][0:3]
        dt = std_baselines[bidx][3:6]
        print("baseline T_{1}_{0}:".format(bidx, bidx+1), file=dest) 
        print("    q: %s +- %s" % (T.q(), np.array(dq)), file=dest)
        print("    t: %s +- %s" % (T.t(), np.array(dt)), file=dest)
        print(file=dest)
    
    print("", file=dest)
    print("", file=dest)
    print("Target configuration", file=dest)
    print("====================", file=dest)
    print("", file=dest)

    cself.cameras[0].ctarget.targetConfig.printDetails(dest)

def printDebugEnd(cself):
    print("")
    print("")
    
    for cidx, cam in enumerate(cself.cameras):
        print("cam{0}".format(cidx))
        print("----------")
        print("")
        print("")
        
        corners, reprojs, rerrs = getReprojectionErrors(cself, cidx)        
        if len(rerrs)>0:
            me, se = getReprojectionErrorStatistics(rerrs)
            print(me[0])
            print(me[1])
            print(se[0])
            print(se[1])
        
        print("")
        p = cam.geometry.projection().getParameters().flatten()
        for temp in p:
            print(temp)
        
        print("")
        d = cam.geometry.projection().distortion().getParameters().flatten()
        for temp in d:
            print(temp)
            
        if cidx>0:
            bidx=cidx-1
            T = sm.Transformation( cself.baselines[bidx].T() )
            for temp in T.t():
                print(temp)
    
            for temp in T.q():
                print(temp)
    
    print("")
    print("")
