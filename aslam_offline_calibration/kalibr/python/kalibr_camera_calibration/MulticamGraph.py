import sm
import aslam_backend as aopt
import aslam_cv as cv
import kalibr_camera_calibration as kcc

import numpy as np
import collections
import igraph
import itertools
import sys
import pylab as pl
import Image
import time

# make numpy print prettier
np.set_printoptions(suppress=True)


class MulticamCalibrationGraph(object):
    def __init__(self, obs_db):
        #observation database
        self.obs_db = obs_db
        self.numCams = self.obs_db.numCameras()
        
        #initialize the graph
        self.initializeGraphFromObsDb(self.obs_db)
    
    def initializeGraphFromObsDb(self, obs_db):        
        t0 = time.time()

        #create graph and label the vertices
        G = igraph.Graph(self.numCams)
        for id, vert in enumerate(G.vs):
            vert["label"] = "cam{0}".format(id)
        
        #go through all times
        for timestamp in self.obs_db.getAllViewTimestamps():
            #cameras that have a target view at this timestamp instant
            cam_ids_at_timestamp = set( obs_db.getCamIdsAtTimestamp(timestamp) )
            
            #go through all edges of the graph and check if we have common corners
            possible_edges = itertools.combinations(cam_ids_at_timestamp, 2)
            
            for edge in possible_edges:
                cam_id_A = edge[0]
                cam_id_B = edge[1]
                
                #and check them against the other cams for common corners (except against itself...)
                corners_A = self.obs_db.getCornerIdsAtTime(timestamp, cam_id_A)
                obs_id_A = self.obs_db.getObsIdForCamAtTime(timestamp, cam_id_A)
                corners_B = self.obs_db.getCornerIdsAtTime(timestamp, cam_id_B)
                obs_id_B = self.obs_db.getObsIdForCamAtTime(timestamp, cam_id_B)
                
                common_corners = corners_A & corners_B
                
                #add graph edge if we found common corners
                if common_corners:
                    #add edege if it isn't existing yet
                    try:
                        edge_idx = G.get_eid(cam_id_A, cam_id_B)
                    except:
                        G.add_edges([(cam_id_A, cam_id_B)])
                        edge_idx = G.get_eid(cam_id_A, cam_id_B)
                        G.es[edge_idx]["obs_ids"] = list()
                        G.es[edge_idx]["weight"] = 0
                    
                    #store the observation of the camera if the lower id first on the edge
                    G.es[edge_idx]["weight"] += len(common_corners)
                    G.es[edge_idx]["obs_ids"].append( (obs_id_A, obs_id_B) if cam_id_A<cam_id_B else (obs_id_B, obs_id_A) )
        
        #store the graph  
        self.G = G
        
        #timing
        t1 = time.time()
        total = t1-t0
        sm.logDebug("It took {0}s to build the graph.".format(total))
    
#############################################################
## SYSTEM PROPERTIES
#############################################################    
    #check if all cams are connected through observations
    def isGraphConnected(self):
        #check if all vertices are connected
        return self.G.adhesion()
        
    #returns the list of cam_ids that share common view with the specified cam_id
    def getCamOverlaps(self, cam_id):
        overlap_vertices=self.G.vs[cam_id].neighbors()
        
        overlaps=list()
        for vert in overlap_vertices:
            overlaps.append(vert.index)
            
        return overlaps
        
#############################################################
## INITIAL GUESS STUFF
#############################################################    
    
    #returns: 
    #        baselines:    list of baselines starting from cam0 to camN
    #                      direction: baseline_O => cam0 to cam1 (T_c1_c0)
    def getInitialGuesses(self, cameras):
        
        if not self.G:
            raise RuntimeError("Graph is uninitialized!")
        
        #################################################################
        ## STEP 0: check if all cameras in the chain are connected
        ##         through common target point observations
        ##         (=all vertices connected?)
        #################################################################
        if not self.isGraphConnected():
            sm.logError("The cameras are not connected through mutual target observations! " 
                        "Please provide another dataset...")
            
            self.plotGraph()
            sys.exit(0)
        
        #################################################################
        ## STEP 1: get baseline initial guesses by calibrating good 
        ##         camera pairs using a stereo calibration
        ## 
        #################################################################

        #first we need to find the best camera pairs to obtain the initial guesses
        #--> use the pairs that share the most common observed target corners
        #The graph is built with weighted edges that represent the number of common
        #target corners, so we can use dijkstras algorithm to get the best pair
        #configuration for the initial pair calibrations
        weights = [1.0/commonPoints for commonPoints in self.G.es["weight"]]

        #choose the cam with the least edges as base_cam
        outdegrees = self.G.vs.outdegree()
        base_cam_id = outdegrees.index(min(outdegrees))

        #solve for shortest path  (=optimal transformation chaining)
        edges_on_path = self.G.get_shortest_paths(0, weights=weights, output="epath")
        
        self.optimal_baseline_edges = set([item for sublist in edges_on_path for item in sublist])
        
        
        #################################################################
        ## STEP 2: solve stereo calibration problem for the baselines
        ##         (baselines are always from lower_id to higher_id cams!)
        #################################################################
        
        #calibrate all cameras in pairs
        for baseline_edge_id in self.optimal_baseline_edges:

            #get the cam_nrs from the graph edge (calibrate from low to high id)
            vertices = self.G.es[baseline_edge_id].tuple
            if vertices[0]<vertices[1]:
                camL_nr = vertices[0]
                camH_nr = vertices[1]
            else:
                camL_nr = vertices[1]
                camH_nr = vertices[0]
            
            print "\t initializing camera pair ({0},{1})...  ".format(camL_nr, camH_nr)          

            #run the pair extrinsic calibration
            obs_list = self.obs_db.getAllObsTwoCams(camL_nr, camH_nr)
            success, baseline_HL = kcc.stereoCalibrate(cameras[camL_nr], 
                                                       cameras[camH_nr], 
                                                       obs_list,
                                                       distortionActive=False)
            
            if success:
                sm.logDebug("baseline_{0}_{1}={2}".format(camL_nr, camH_nr, baseline_HL.T()))
            else:
                sm.logError("initialization of camera pair ({0},{1}) failed  ".format(camL_nr, camH_nr))
                sm.logError("estimated baseline_{0}_{1}={2}".format(camL_nr, camH_nr, baseline_HL.T()))
        
            #store the baseline in the graph
            self.G.es[ self.G.get_eid(camL_nr, camH_nr) ]["baseline_HL"] = baseline_HL
        
        #################################################################
        ## STEP 3: transform from the "optimal" baseline chain to camera chain ordering
        ##         (=> baseline_0 = T_c1_c0 | 
        #################################################################
        
        #construct the optimal path graph
        G_optimal_baselines = self.G.copy()
        
        eid_not_optimal_path = set(range(0,len(G_optimal_baselines.es)))
        for eid in self.optimal_baseline_edges:
            eid_not_optimal_path.remove(eid)
        G_optimal_baselines.delete_edges( eid_not_optimal_path )
        
        #now we convert the arbitary baseline graph to baselines starting from 
        # cam0 and traverse the chain (cam0->cam1->cam2->camN)
        baselines = []
        for baseline_id in range(0, self.numCams-1):
            #find the shortest path on the graph
            path = G_optimal_baselines.get_shortest_paths(baseline_id, baseline_id+1)[0]
            
            #get the baseline from cam with id baseline_id to baseline_id+1
            baseline_HL = sm.Transformation()
            for path_idx in range(0, len(path)-1):
                source_vert = path[path_idx]
                target_vert = path[path_idx+1]
                T_edge = self.G.es[ self.G.get_eid(source_vert, target_vert) ]["baseline_HL"]
            
                #correct the direction (baselines always from low to high cam id!)
                T_edge = T_edge if source_vert<target_vert else T_edge.inverse()
            
                #chain up
                baseline_HL = T_edge * baseline_HL
            
            #store in graph
            baselines.append(baseline_HL)
 
        #################################################################
        ## STEP 4: refine guess in full batch
        #################################################################
        success, baselines = kcc.solveFullBatch(cameras, baselines, self)
        
        if not success:
            sm.logWarn("Full batch refinement failed!")
    
        return baselines
    
    def getTargetPoseGuess(self, timestamp, cameras, baselines_HL=[]):
        #go through all camera that see this target at the given time
        #and take the one with the most target points        
        camids = list()
        numcorners = list()
        for cam_id in self.obs_db.getCamIdsAtTimestamp(timestamp):
            camids.append(cam_id)
            numcorners.append( len(self.obs_db.getCornerIdsAtTime(timestamp, cam_id)) )

        #get the pnp solution of the cam that sees most target corners
        max_idx = numcorners.index(max(numcorners))
        cam_id_max = camids[max_idx]        
        
        #solve the pnp problem
        camera_geomtry = cameras[cam_id_max].geometry
        success, T_t_cN = camera_geomtry.estimateTransformation(self.obs_db.getObservationAtTime(timestamp, cam_id_max))
               
        if not success:
            sm.logWarn("getTargetPoseGuess: solvePnP failed with solution: {0}".format(T_t_cN))
        
        #transform it back to cam0 (T_t_cN --> T_t_c0)
        T_cN_c0 = sm.Transformation()
        for baseline_HL in baselines_HL[0:cam_id_max]:
            T_cN_c0 = baseline_HL * T_cN_c0
        
        T_t_c0 = T_t_cN * T_cN_c0
        
        return T_t_c0
    
    #get all observations between two cameras
    def getAllMutualObsBetweenTwoCams(self, camA_nr, camB_nr):
        #get the observation ids
        try:
            edge_idx = self.G.get_eid(camA_nr, camB_nr)
        except:
            sm.logError("getAllMutualObsBetweenTwoCams: no mutual observations between the two cams!")
            return [], []
        
        observations = self.G.es[edge_idx]["obs_ids"]
        
        #extract the ids
        obs_idx_L = [obs_ids[0] for obs_ids in observations]
        obs_idx_H = [obs_ids[1] for obs_ids in observations]
        
        #the first value of the tuple always stores the obsvervations for camera
        #with the lower id
        obs_idx_A = obs_idx_L if camA_nr<camB_nr else obs_idx_H
        obs_idx_B = obs_idx_H if camA_nr<camB_nr else obs_idx_L
        
        #get the obs from the storage using idx
        obs_A = [self.obs_db.observations[camA_nr][idx] for idx in obs_idx_A]
        obs_B = [self.obs_db.observations[camB_nr][idx] for idx in obs_idx_B]

        return obs_A, obs_B
         
#############################################################
## PLOTTING AND PRINTING
#############################################################            
    def plotGraph(self, noShow=False):        
        layout = self.G.layout("kk")
        
        #plot the optimal baselines for pair calibration
        try:
            edgewidth=[]
            for edge_idx, edge in enumerate(self.G.es):
                if edge_idx in self.optimal_baseline_edges:
                    edgewidth.append(10)
                else:
                    edgewidth.append(2)
        except AttributeError:
            edgewidth = 2
        
        if not noShow:
            target = None
        else:
            target = "/tmp/graph.png"

        plot = igraph.plot(self.G, 
                           layout=layout, 
                           rescale=False, 
                           add=False,
                           target=target,
                           vertex_size=50,
                           edge_label=self.G.es["weight"],
                           edge_width=edgewidth,
                           margin = 50)
        
        if not noShow:
            return plot
        else:
            return target

    def plotGraphPylab(self, fno=0, noShow=True, clearFigure=True, title=""):
        target = self.plotGraph(noShow=True)
        
        #create figure
        f = pl.figure(fno)
        if clearFigure:    
            f.clf()
        f.suptitle(title)
        
        img = Image.open(target)
        pl.imshow(np.array(img))
        pl.axis('off')
        
        if not noShow:
            pl.show()
