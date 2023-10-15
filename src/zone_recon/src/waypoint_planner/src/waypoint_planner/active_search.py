"""
Code for the work:

`` Stealthy Terrain-Aware Multi-Agent Active Search``,
Nikhil Angad Bakshi and Jeff Schneider
Robotics Institute, Carnegie Mellon University

(C) Nikhil Angad Bakshi 2023 (nabakshi@cs.cmu.edu)
Please cite the following paper to use the code:


@inproceedings{
bakshi2023stealthy,
title={Stealthy Terrain-Aware Multi-Agent Active Search},
author={Nikhil Angad Bakshi and Jeff Schneider},
booktitle={7th Annual Conference on Robot Learning},
year={2023},
url={https://openreview.net/forum?id=eE3fsO5Mi2}
}
"""
#!/usr/bin/env python

from __future__ import print_function

import numpy as np
from math import erf
import random
import math
import os
import pickle as pkl
import scipy
import scipy.stats as ss
import copy
from scipy.stats import invgauss
import time
from scipy.sparse.linalg import inv as sparse_inv
from scipy.sparse import csc_matrix, hstack
from waypoint_planner_utils import is_within_search_region, grid2map, transform_pose, Pose, rospy, plt, SendMessage, \
    init_searchpolygon_marker, Marker, GetMessage, map2grid, euler_from_quaternion, TrackDataVector, start2grid, \
    costmap2slice, OccupancyGridUpdate, OccupancyGrid, ColorRGBA, MarkerArray, bresenham_line_gen, rgb2gray, grid2grid, \
    compute_probability_prior
from collections import Counter
from zone_recon_msgs.msg import ThreatVector
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped
import json, yaml
from std_msgs.msg import Float32MultiArray
from PIL import Image

# import sys
# np.set_printoptions(threshold=sys.maxsize)
import matplotlib.pyplot as plt
from matplotlib import cm, image
import shutil
import cProfile
import pstats

# TODO: 1. Change the zone clearing parameter to a search style like random, coverage, or nats
# TODO: 2. See all locations where that comes into play and delete the temporary zone clearing paramter in nats_params.yaml
# TODO: 3. Current best value for distance cost is 0.3. THis should automatically be set for ZC behaviour regardless of what
# TODO: 3 (contd). is user specified. Better yet consider killing that parameter altogether? Or is it useful for viewshed
# TODO: 4 Update logging config for the new visibility related topics



class NATS(object):

    def __init__(self, beta, n1, mu, noise_vec, lmbd, EMitr, n_agents, trl, search_polygon, costmap):
        # theta2 # signal variance to create nonzero entries of vector beta
        self.err = 0.05 # hyperparameter for RSI algorithm
        self.k = rospy.get_param("~num_targets") # number of targets, used in RSI math and for threat simulation for the everythin
        self.mu = mu # signal intensity to create nonzero entries of vector beta, this parameter is not used for estimation
        self.beta = beta.reshape((-1,1)) # ground truth
        self.lmbd = lmbd
        self.EMitr = EMitr # number of iterations for the Expectation-Maximization estimator
        self.trl = trl
        self.n_agents = n_agents
        self.gamma = lmbd**2
        self.search_polygon = search_polygon
        self.current_pose = None
        self.costmap = costmap
        self.n = beta.shape[0]
        self.n1 = n1 # length n1 of matrix beta # keep small
        self.n2 = int(self.n/self.n1)
        self.L = int(self.n)
        self.M = int(self.n / self.L)
        self.points_dict = {'X': [], 'Y': [], 'par': [] }
        self.rng = np.random.RandomState(trl)
        self.marker_publisher = init_searchpolygon_marker(SendMessage("candidate_waypoint", Marker))
        self.marker_publisher.msg.type = Marker.LINE_STRIP
        self.marker_publisher.msg.colors = []
        # self.marker_publisher.msg.scale.z = 5.0
        # self.marker_publisher.msg.scale.x = 5.0
        self.marker_publisher.msg.points = []
        # self.marker_array_publisher = SendMessage(topic="dummy_threats", msg_type=MarkerArray)
        self.sig_beta_pub = SendMessage(topic=rospy.get_param("~coverage_variance_topic"), msg_type=OccupancyGrid)
        self.beta_hat_pub = SendMessage(topic=rospy.get_param("~threats_belief_topic"), msg_type=OccupancyGrid)
        # self.track_subscriber = TrackSubscriber(self.points_dict, topic=rospy.get_param("~track_topic"), msg_type=TrackDataVector, verbose=True)
        self.polygon_memory = None # variable that remembers latest list of points that specify search polygon
        self.allowable_grid_locations = []
        self.current_waypoint = []
#        self.err = err
        # sigma2 is noise variance on observations: # noise variance on observations
        self.noise_vec = noise_vec # np.append(np.append(np.array(1*[sigma2,sigma2]), np.repeat(4*sigma2,4)),np.repeat(9*sigma2,6))
        # self.working_dir = "/home/unicorn/src/zone_recon/"
        self.working_dir = "/home/"+str(os.environ['USER'])+"/src/zone_recon/"
        if "thesis" in rospy.get_param("~map_type"):
            self.robogrid = pkl.load(open(self.working_dir + "robogrid_"+rospy.get_param("~location")+rospy.get_param("~map_type")[-2:]+"_dict_gs"+str(int(rospy.get_param("~cell_size")))+"_python2.pkl", "rb"))
        else:
            self.robogrid = pkl.load(open(self.working_dir + "robogrid_"+rospy.get_param("~location")+"_dict_gs"+str(int(rospy.get_param("~cell_size")))+"_python2.pkl", "rb"))
        self.rg_xlim, self.rg_ylim = sorted(self.robogrid.keys(), reverse=True)[0]
        self.rg_xlim += 1 # because of zero indexing
        self.rg_ylim += 1
        print("Max extent of robogrid is ", self.rg_xlim, self.rg_ylim)
        self.fov_setting = rospy.get_param("~fov_setting")
        self.step = 0
        self.oldh, self.oldl = None, None
        self.ground_truth_threats = None
        self.robot_name = rospy.get_param("~robot_name")
        if rospy.get_param("~logging"):
            algorithm = "STAR" + str(rospy.get_param("~visibility_cost_weightage")) if (rospy.get_param("~visibility_cost_weightage") and rospy.get_param("~search_mode") == "nats") else rospy.get_param("~search_mode").replace("nats", "guts").upper()
            algorithm += "-ZC" + str(rospy.get_param("~distance_cost_weightage")) if (rospy.get_param("~distance_cost_weightage") and rospy.get_param("~zone_clearing")) else ""
            experiment = "-" + rospy.get_param("~threat_sampling")[0]
            experiment += "-k_" + str(rospy.get_param("~num_targets"))
            experiment += "-" + rospy.get_param("~map_type")[-2:]

            if rospy.has_param("/" + self.robot_name + "/simulate_threats/threat_breakdown"):
                if rospy.get_param("/" + self.robot_name + "/simulate_threats/threat_breakdown"):
                    experiment += "-threat_breakdown"
            if not rospy.get_param("~simulate_crosspose"):
                experiment += "-crosspose_false"

            self.logging_path = rospy.get_param("~logging_path")[:-1] + experiment + "/" + algorithm + "/" + self.robot_name + "/" + str(time.asctime( time.localtime(time.time()) )) + "/"
            rospy.loginfo("Zone Recon Metrics Logging path: " + str(self.logging_path))
            if rospy.get_param("~autonomy_faker"):
                rospy.set_param("/"+self.robot_name+"/autonomy_faker/logging_path", self.logging_path)
            if not os.path.exists(self.logging_path):
                os.makedirs(self.logging_path)
            
            self.start_time = rospy.get_time()
            self.start_wall_time = time.time()
            print("Starting time is: ", self.start_time)
            self.logs = []

            shutil.copy(self.working_dir + "src/waypoint_planner/config/nats_params.yaml", self.logging_path)
            shutil.copy(self.working_dir + "src/waypoint_planner/config/default_parameters.yaml", self.logging_path)

            param_dict = {}
            for p in ["~visibility_cost_weightage", "~distance_cost_weightage", 
                      "~zone_clearing", "~robot_name", "~search_mode", "~random_seed",
                      "~location", "~map_type", "/" + self.robot_name + "/navigation_manager/ompl_global_planner/state_cost_weight",
                      "~threat_sampling", "~simulate_crosspose", "/" + self.robot_name + "/simulate_threats/threat_breakdown",
                      "~autonomy_faker", "~num_targets"]:
                if rospy.has_param(p):
                    param_dict[p[1:]] = rospy.get_param(p)

            json_str = json.dumps(yaml.load(str(param_dict)), indent=4)
            with open(self.logging_path + "run_settings_.json", 'wb') as f:
                f.write(json_str)

            threat_file = self.working_dir + "src/waypoint_planner/config/threat_locs.pkl"
            if rospy.get_param("~use_threat_locs") and os.path.isfile(threat_file):
                shutil.copy(threat_file, self.logging_path)
                
                with open(threat_file, 'rb') as f:
                    self.ground_truth_threats = set(pkl.load(f))


        # enables visualization of belief posterior and reward using matplotlib
        self.visualize = True
        self.robot_goals = []
        self.robot_path = []
        self.robot_path_dict = {}
        self.robot_path_timestamps = []
        self.robot_path_timestamps_dict = {}
        self.last_loc = None

        # for discreet search
        self.visibility_costmap_publisher = SendMessage(topic=rospy.get_param("~viz_costmap_topic_lowres"),
                                                        msg_type=Float32MultiArray,
                                                        latched=True)
        self.visibility_prior = image.imread(self.working_dir + "assets/"+ \
                                             rospy.get_param("~location")+"/visibility_cost_"+ \
                                             rospy.get_param("~map_type")+".png")
        self.visibility_prior = rgb2gray(self.visibility_prior)
        print("Visibility prior dimensions", self.visibility_prior.shape)
        self.visibility_prior /= np.max(self.visibility_prior)
        # self.probability_prior = image.imread(self.working_dir + "assets/"+ \
        #                                      rospy.get_param("~location")+"/probability_map_"+ \
        #                                      rospy.get_param("~map_type")+".png")
        # print("Probability prior dimensions", self.probability_prior.shape)
        # self.probability_prior = rgb2gray(self.probability_prior)
        # self.probability_prior /= np.max(self.probability_prior)
        self.probability_prior = compute_probability_prior(self.visibility_prior, self.costmap, self.n1, self.n2)

        # print(self.probability_prior_2, self.probability_prior)
        Image.fromarray(255.0 * self.probability_prior).convert("L").save(open(self.working_dir+"probability_heightmap_path.png", "wb"))
        # assert (self.probability_prior.shape == self.probability_prior_2.shape)
        # assert np.array_equal(self.probability_prior, self.probability_prior_2)
        self.publish_grid(rospy.get_param("~threat_prior_inside_sp") * np.ones((self.n2, self.n1)).flatten(), self.visibility_costmap_publisher, viz_cost=True)

        if rospy.get_param("~robot_type") == "ugv":
            x_full, _, _ = self.create_directional_sensor(self.n1//2, self.n2//2, "ugv")
            self.max_viewshed = x_full.shape[0]
        else:
            rospy.loginfo("Discreet search not implemented for uas")
            exit()

        # for evaluation
        self.threat_publisher = SendMessage(topic = self.robot_name+"/detected_threats", msg_type=ThreatVector)

    def save_log(self, log, Sig_beta, beta_hat, beta_hat_vis):
        self.logs.append(log)

        with open(self.logging_path+"log.pkl", 'wb') as f:
            pkl.dump(self.logs, f)
        
        with open(self.logging_path+"readable_log.txt", 'a') as f:
            print(log, file=f)
        
        t = (log['runtime']//0.001)*0.001

        np.save(self.logging_path+ "belief_var_" + str(t), Sig_beta)
        np.save(self.logging_path+ "belief_mu_" + str(t), beta_hat)
        np.save(self.logging_path+ "belief_beta_vis_" + str(t), beta_hat_vis)

        return

    def set_beta(self,beta):
        # don't need or use this
        self.beta = beta

    def sample_from_prior(self, num_samples):
        # Code doesn't use this
        self.rng = np.random.RandomState(self.trl)
        beta_tilde = np.zeros((num_samples, self.n, 1))

        for idx in range(num_samples):

            beta_tilde[idx] = self.rng.laplace(scale=1/self.lmbd,size=(self.n,1))

        return beta_tilde

    def sample_from_prior_per_worker(self, recv_time):
        # This enforces a diff random seed per worker
        self.rng = np.random.RandomState(self.trl+recv_time)

        beta_tilde = np.maximum(self.rng.laplace(scale=1/self.lmbd,size=(self.n,1)),np.zeros((self.n,1)))

        return beta_tilde

    def get_near_pd_v2(self, A):
        C = (A + A.T) / 2

        eigval, eigvec = np.linalg.eig(C)
        eigval[np.real(eigval) <= 1e-8] = 1e-8
        return np.real(eigvec.dot(np.diag(eigval)).dot(eigvec.T))

    def get_multivariate_normal(self, mean, cov, rng):
        # from np.dual import svd
        mean = np.array(mean)
        cov = np.array(cov)
        shape = []
        final_shape = list(shape[:])
        final_shape.append(mean.shape[0])
        x = rng.standard_normal(final_shape).reshape(-1, mean.shape[0])
        cov = cov.astype(np.double)
        # (u, s, v) = np.linalg.svd(cov, hermitian=True)

        (u, s, v) = scipy.linalg.svd(cov, lapack_driver='gesvd')

        x = np.dot(x, np.sqrt(s)[:, None] * v)
        x += mean
        x.shape = tuple(final_shape)

        return x

    def getPosterior(self, X, Y, pos, recv_time):
        # posterior (beta| data) = N(mu, V)
        # V = sig_beta
        # mu = beta_hat_tmp
        # X    (d*num_obs) x N , d can vary with observations
        a = 0.1
        b = 1
        n = X.shape[1]
        self.rng = np.random.RandomState(self.trl+recv_time)
        # print(recv_time)
        if self.n != X.shape[1]:
            print("You have entered the matrix: ", self.n, X.shape[1])
        beta_hat = np.zeros((n,1))
        # Sig_beta = np.zeros((self.n,self.n))+1e8
        idx = []
        gamma = np.zeros((n,1)) +1e+8  # why is this such a big number? Want to be very uncertain to start
        for j in range(self.EMitr): # try 10 to start: number of iterations for the Expectation-Maximization estimator
            # can also try ending using a tolerance for the sigma_beta or beta_hat
            t0 = time.time()
            Gamma = np.diag(np.squeeze(gamma)) # squeeze makes it a vector
            inv_y_one_undiag = 1. / np.squeeze(Y[:, 1])
            if isinstance(inv_y_one_undiag, float):
                inv_y_one = np.array([[inv_y_one_undiag]])
            else:
                inv_y_one = np.diag(inv_y_one_undiag)  # depth-aware noise corresponding to each of the y's
            # sig_beta= (X^T * inv(y1) * X + diag(gammas))^(-1) # NxN
            # print("inv_y_one in get posterior: ", inv_y_one)
            # if X.shape[0] > X.shape[1]:
                # Inversion of a size of grid order matrix
            ginv_xtsx = np.matmul(np.matmul(np.transpose(X),inv_y_one),X) + np.diag(1/np.diag(Gamma))

            if rospy.get_param("~robot_type") != "uas":
                assert np.array_equal(np.linalg.inv(Gamma) , np.diag(1/np.diag(Gamma)))
                divergence = np.count_nonzero(ginv_xtsx - np.diag(np.diagonal(ginv_xtsx)))
                if divergence:
                    rospy.loginfo("ZERO IF DIAGONAL MATRIX (ensure is zero on robot): %d" % divergence)

            Sig_beta = np.diag(1/np.diag(ginv_xtsx))
            # rospy.loginfo("Diag inverse time: %f" % (time.time() - t_diag_inv))

            # print("Are the results equal? conv v sparse: ", np.array_equal(Sig_beta, Sig_beta_s))
            # print("Are the results equal? conv v diag: ", np.array_equal(Sig_beta, Sig_beta_d))
            # else:
                # Inversion of a number of observations size matrix
                # try:
                #     Sig_beta = Gamma - np.matmul(np.matmul(Gamma, np.transpose(X)), np.matmul(np.linalg.inv(np.diag(np.array([np.squeeze(Y[:, 1])])) + np.matmul(np.matmul(X, Gamma), np.transpose(X))), np.matmul(X, Gamma)))
                # except:
                #     Sig_beta = np.linalg.inv(np.matmul(np.matmul(np.transpose(X), inv_y_one), X) + np.linalg.inv(Gamma))
                #     print("There was a failure in inversion")
                    # pickle.dump([np.diag(np.array([np.squeeze(Y[:, 1])])), X, Gamma], open(rospy.get_param("~logging_path") + "debug_inverse_" + str(time.time())+ ".pkl", "wb"))
                    # print(np.diag(np.array([np.squeeze(Y[:, 1])])) + np.matmul(np.matmul(X, Gamma), np.transpose(X)))

            # beta_hat = sig_beta * X^T inv(y1) y0   # Nx1

            beta_hat_tmp = np.matmul(np.matmul(Sig_beta, np.transpose(X)), np.matmul(inv_y_one,
                                           np.reshape(Y[:,0],(-1,1)))) # mean of posterior
            
            t1 = time.time()
            rospy.loginfo("One time posterior calc: %f" % (t1 - t0))
            
            t0 = time.time()
            for ii in range(n): #n=N , length of beta
                gamma[ii,0] = (2*b+beta_hat_tmp[ii,0]**2+Sig_beta[ii,ii])/(1+2*a)
                if(ii not in idx):
                    beta_hat[ii,0] = beta_hat_tmp[ii]
                if(gamma[ii,0]<1e-8):
                    gamma[ii,0] = 1e-8
                    beta_hat[ii,0] = 0.0
                    idx.append(ii)
            t1 = time.time()
            rospy.loginfo("Potentially vectorizable bit in posterior: %f" % (t1 - t0))


        # TODO: Experiment and log (smarter way of doing previous 3 lines)
        # We dont want negative eigenvalues bec the covariance mtrix can never have negative eignevalues
        # numerical errors due to round off etc lead to negative values sometimes
        # print("Sig_beta in get posterior: ", Sig_beta)
        t0 = time.time()
        # min_eig = np.amin(np.real(np.linalg.eigvals(Sig_beta)))
        # if min_eig < 0:
        #     Sig_beta -= 1.1* min_eig * np.eye(*Sig_beta.shape) #What's the deal here?
        #
        min_eig = np.amin(np.real(np.linalg.eigvals(Sig_beta)))
        if min_eig <= 1e-8:
            Sig_beta = self.get_near_pd_v2(Sig_beta)
        t1 = time.time()
        rospy.loginfo("Ensuring positive semi def in posterior: %f" % (t1 - t0))

        t0 = time.time()
        # beta_tilde = np.maximum(np.reshape(self.rng.multivariate_normal(np.squeeze(beta_hat), Sig_beta), (self.n, 1)),
        #                         np.zeros((self.n, 1)))
        # print("this where beta_tilde")
        # print(np.count_nonzero(beta_tilde))
        beta_tilde = np.maximum(np.reshape(self.get_multivariate_normal(np.squeeze(beta_hat),Sig_beta, self.rng),(n,1)),np.zeros((n,1)))
        # print(np.count_nonzero(beta_tilde))
        t1 = time.time()
        rospy.loginfo("Taking thompson sample in posterior: %f" % (t1 - t0))
        '''
        # even sparser belief of signals
        floorsample = ((beta_tilde)>(np.amax(beta_tilde)/2)) # The 2 just ensures sparsity
        # this^ is a heuristic
        # TODO: try without it
        beta_tilde = np.zeros((self.n,1))
        # self.mu could very well be 1.0
        beta_tilde[floorsample] = self.mu # the signal intensity for non-zeros values of beta_tilde is preselected
        '''
        return beta_tilde,Gamma,Sig_beta,beta_hat,pos, ginv_xtsx, inv_y_one#pi_0,beta_rsi

    def getPosteriorRSI(self, X, Y, polygon_masker, recv_time):  # rng, n, X, Y, EMitr=1):
        # profiler = cProfile.Profile()
        # profiler.enable()

        n = X.shape[1]
        ######### posterior computation RSI #########
        pi_0 = np.ones((n, 1)) * 1. / n
        beta_hat_rsi = np.zeros((n, 1))
        #        print('X shape: ',X.shape)

        beta_rsi = copy.deepcopy(self.beta)

        t_pi0_init = 0
        t_pi0_second = 0
        for i in range(X.shape[0]):
            t0 = time.time()
            b = np.repeat(copy.deepcopy(beta_hat_rsi), repeats=n, axis=1)

            # pi_0_copy = copy.deepcopy(pi_0)
            for j in range(n):
            #     b = np.zeros((n, 1))
                b[j, j] = self.mu
                # print('X[i] shape: ', X[i].shape)
                # assert X[i].shape == (1,32)
                # print('shape: ', Y[i].shape)
                # assert(np.dot(X[i], b) == self.mu*X[i,j])
                # pi_0[j] = np.float64(pi_0[j] * ss.norm(0, np.sqrt(Y[i, 1])).pdf(Y[i, 0] - np.dot(X[i], b)))
                # pi_0[j] = np.float64(pi_0[j] * ss.norm(0, np.sqrt(Y[i, 1])).pdf(Y[i, 0] - self.mu*X[i,j]))
            # assert(np.array_equal(pi_0, np.float64(np.multiply(pi_0_copy, ss.norm(0, np.sqrt(Y[i,1])).pdf(Y[i,0] - self.mu*X[i].reshape(-1,1))))))
            # pi_0 = np.float64(pi_0 * ss.norm(0, np.sqrt(Y[i, 1])).pdf(Y[i, 0] - self.mu*X[i]).reshape(-1, 1))
            pi_0 = np.float64(pi_0 * ss.norm(0, 1).pdf(Y[i, 0] - np.dot(b, X[i])).reshape(-1, 1))/np.sqrt(Y[i, 1])
            # print(pi_0)
            # print(pi_0_copy)
            if np.sum(pi_0)>0:
                pi_0 /= np.sum(pi_0)
            t_pi0_init += time.time() - t0
            # return pi_0, pi_0, beta_rsi, None, None
            # print(i, Y[i])
            if np.amax(pi_0) == 0.:
                print('1-process ', os.getpid(), 'with recv_time', recv_time, ' would raise ValueError')
                break
                # continue
                # raise ValueError('pi_0 max value 0.!')
            maxidxs = np.argwhere(pi_0 == np.amax(pi_0))
            # print("Maxidxs: ", len(maxidxs), "max val: ", np.amax(pi_0))
            eps = 0.
            for ids in maxidxs:
                eps += 1 - pi_0[ids][0]
            # print("eps: ", eps)
            if (eps < self.err):
                # idxs = []
                for m in maxidxs:
                    beta_hat_rsi[m[0], :] = self.mu

                    beta_rsi[m[0], :] = 0.

                detected = np.count_nonzero(beta_hat_rsi)
                if (detected >= self.k):
                    print("so far detected: ",detected," left:",np.count_nonzero(beta_rsi))
                    break
                if np.count_nonzero(beta_rsi) == 0:
                    print("Breaking because beta_rsi uniformly zero" )
                    break
                # pi_0 /= np.sum(pi_0)

                idxs = [ii for ii in range(n) if ii not in np.nonzero(beta_hat_rsi)[0]]
                print("Num nonzero idxs: ", len(idxs))

                pi_0 = np.ones((n,1))
                pi_0[np.nonzero(beta_hat_rsi)[0],:] = 0.

                t0 = time.time()
                for t in range(i + 1):
                    b = np.repeat(copy.deepcopy(beta_hat_rsi), repeats=len(idxs), axis=1)  # N x b
                    for loc, idx in enumerate(idxs):
                        b[idx, loc] = self.mu
                        # pi_0[idx] = 1. / n
                    pi_0[idxs, :] = np.float32(
                            pi_0[idxs, :] * ss.norm(0, 1).pdf(Y[t, 0] - np.matmul(b.T, X[t])))/np.sqrt(Y[t, 1])
                    if np.sum(pi_0) > 0:
                        pi_0 /= np.sum(pi_0)
                t_pi0_second += time.time() - t0

                if (np.amax(pi_0) == 0):
                    print('2-process ', os.getpid(), ' would raise ValueError')
                    print(recv_time)
                    break

                if (math.isinf(np.amax(pi_0)) or math.isnan(np.sum(pi_0))):
                    pi_0 = np.nan_to_num(pi_0)
                    print('inf or nan value for pi_0')

                pi_0 /= np.nansum(pi_0)


        print('returning beta_hat')
        print("\n\nInitial pi_0 calculation: ", t_pi0_init)
        print("\n\nSecondary pi_0 calculation: ", t_pi0_second)

        _ = 1
        # pi_0_full = np.zeros((self.n, 1), dtype=np.float64)
        # np.put(pi_0_full, polygon_masker, pi_0)
        # beta_hat_rsi_full = np.zeros((self.n, 1))
        # np.put(beta_hat_rsi_full, polygon_masker, beta_hat_rsi)

        # profiler.disable()
        # stats = pstats.Stats(profiler).sort_stats('ncalls')
        # stats.print_stats()
        print()
        return _, pi_0, beta_rsi, beta_hat_rsi, _

    def get_visibility_cost(self, Sig_beta, beta_hat, X, polygon_masker):
        seen_cells = (X.sum(axis=0) != 0).astype(int)
        print("Sig_beta and beta_hat dimensions: ", Sig_beta.shape, beta_hat.shape)
        beta_vis = np.zeros_like(beta_hat) + rospy.get_param("~threat_prior_inside_sp")
        for i in range(len(beta_vis)):
            if seen_cells[i] and beta_hat[i]!=0:
                beta_vis[i] = np.clip(np.sqrt(Sig_beta[i,i]) * np.sqrt(2 / np.pi) * np.exp( - beta_hat[i] ** 2 / (2 * Sig_beta[i,i])) + \
                              beta_hat[i] * (1 - 2 * self.phi(-beta_hat[i]/np.sqrt(Sig_beta[i,i]))), 0, 1)
            elif seen_cells[i] and beta_hat[i]==0:
                beta_vis[i] = np.clip(
                    np.sqrt(Sig_beta[i, i]) * np.sqrt(2 / np.pi) * np.exp(- beta_hat[i] ** 2 / (2 * Sig_beta[i, i])) + \
                    beta_hat[i] * (1 - 2 * self.phi(-beta_hat[i] / np.sqrt(Sig_beta[i, i]))), 0, rospy.get_param("~threat_prior_inside_sp"))
        beta_vis_full = np.zeros(self.n) + rospy.get_param("~threat_prior_outside_sp")
        seen_cells_full = np.zeros(self.n)
        viz_cost_full = np.zeros((self.n2, self.n1, 4))  # similar as above for loss
        print("shape of seen cells: ", seen_cells.shape, "shape of seen cells full", seen_cells_full.shape)
        np.put(seen_cells_full, polygon_masker, seen_cells)
        np.put(beta_vis_full, polygon_masker, beta_vis)
        d = 0
        for i in range(len(seen_cells_full)):#np.flatnonzero(seen_cells_full):
            h = i // self.n1
            l = i % self.n1
            # self.robogrid[]
            if h < self.rg_xlim and l < self.rg_ylim:
                viz_cost_full[h, l, d] += beta_vis_full[int(h*self.n1 + l)]
                visibility = {item: self.robogrid[h, l]["viewshed"][subdictkey][item] for subdictkey in
                              self.robogrid[h, l]["viewshed"] for item in
                              self.robogrid[h, l]["viewshed"][subdictkey]}  # this is a dict
                # TODO: What do I want?
                # visibility is location in scalar value but in robogrid dimensions
                # for every location () in visibility I want to multiply that by the beta_vis_full for the corresponding h,l
                # in the outer loop and add it to the viz_cost_full at that location
                for loc_rg, vis in visibility.items():
                    rgh = loc_rg % self.rg_xlim
                    rgl = int(loc_rg / self.rg_xlim)
                    # print("ROOBGRID: ", rgh, rgl, d, "REAL WORLD: ", h, l)
                    if rgh < self.n2 and rgl < self.n1:
                        viz_cost_full[rgh, rgl, d] += vis * beta_vis_full[int(h*self.n1 + l)]

        return beta_vis, viz_cost_full, beta_vis_full

    def phi(self, x):
        # 'Cumulative distribution function for the standard normal distribution'
        return (1.0 + erf(x / np.sqrt(2.0))) / 2.0
    
    def publish_detected_threats(self):
        points_dict = self.points_dict
        polygon_masker = np.ravel_multi_index(np.roll(np.array(self.inpolygon_grid_locations).T, 1, axis=0), (self.n2, self.n1))
        recv_time = int(time.time())

        X = points_dict['X'][:, polygon_masker]
        Y = points_dict['Y']
        [_,pos] = points_dict['par']
        beta_tilde, Gamma, Sig_beta, beta_hat, _, ginv_xtsx, inv_y_one = self.getPosterior(X,Y,pos,recv_time)

        threat_locs = [self.inpolygon_grid_locations[j] for j in np.nonzero(beta_hat.squeeze())[0]]

        if len(threat_locs) > 0:
            threats = ThreatVector()
            threats.header.stamp = rospy.Time().now()
            threats.header.frame_id = "grid"
            threats.robot_name = self.robot_name
            threats.num_waypoints = len(self.robot_goals)
            for i in range(len(threat_locs)):
                print("Threats suspected at: ", threat_locs[i])
                threats.threat_list.append(Point(x=threat_locs[i][1], y=threat_locs[i][0]))
            
            self.threat_publisher.msg = threats 
            self.threat_publisher.publish()
        
        return


    def normalize_ptp(self, loss):
        if np.ptp(loss) != 0:
            return (loss-np.min(loss))/np.ptp(loss)
        else:
            return loss

    # returns selected waypoint
    def ActiveSearch(self, search_mode=None): #beta,mu,theta2,sigma2,lmbd,EMitr,T,trl=np.random.randint(1000),X,Y):
        if search_mode is None:
            search_mode = rospy.get_param("~search_mode")
        start_planning_time = rospy.get_time()
        grid_cell_size = rospy.get_param("~cell_size")
        search_polygon_msg = self.search_polygon.get_copy()
        robot_type = rospy.get_param("~robot_type")
        costmap_vacancy_threshold = rospy.get_param("~costmap_vacancy_threshold")
        points_dict = self.points_dict
        recv_time = int(time.time())
        self.rng = np.random.RandomState(self.trl+recv_time)
        wid = 0 #qinfo.worker_id   # may still be useful when when distributedly computing on different nodes

        # for viz
        self.marker_publisher.msg.points = []
        self.marker_publisher.msg.colors = []
        if robot_type == "ugv":
            cmap_arr = np.array(self.costmap.msg.data).reshape((self.costmap.msg.info.height, self.costmap.msg.info.width))

        # cache allowable_grid_locations if search_polygon is not updated  
        # allowable_grid_locations contains traversable grid cells within the search polygon
        # inpolygon_grid_locations contains all grid cells within search polygon
        new_polygon = False
        if self.polygon_memory != self.search_polygon.get_copy().polygon:
            new_polygon = True
            t = ((time.time() - self.start_wall_time) // 0.001) * 0.001
            json_str = json.dumps(yaml.load(str(self.search_polygon.get_copy())), indent=4)
            #json_str = json_message_converter.convert_ros_message_to_json(self.search_polygon.get_copy())
            with open(self.logging_path + "search_polygon_" + str(t) + ".json", 'wb') as f:
                f.write(json_str)
            self.polygon_memory = self.search_polygon.get_copy().polygon
            self.allowable_grid_locations = []
            self.inpolygon_grid_locations = []
            
        if search_mode == "coverage" or search_mode == "random" or len(points_dict["X"]) == 0:
            Sig_beta, Sig_beta_full, beta_hat, beta_hat_full, cells_this_round, count, dm, hm, lm, pos = self.coverage_mode(cmap_arr,
                                                                                                             costmap_vacancy_threshold,
                                                                                                             grid_cell_size,
                                                                                                             new_polygon,
                                                                                                             points_dict,
                                                                                                             robot_type,
                                                                                                             search_mode,
                                                                                                             search_polygon_msg)
            beta_hat_vis_full = beta_hat_vis = np.zeros_like(beta_hat) + rospy.get_param("~threat_prior_inside_sp")
            if self.current_pose:
                oldh, oldl = start2grid(self.current_pose, grid_cell_size)
                # print("Inside NATS: ", (oldl, oldh))

            else:
                # Assumes perfection hence not good code
                oldl = pos[wid] % self.n1 # current length position
                oldh = (pos[wid]-oldl)/self.n1 # current height position
        else:
            t0 = time.time()

            # stores traversable grid locations (inside the search polygon) in allowable_grid_locations
            # we don't need to compute NATS rewards for other locations
            if new_polygon:
                self.set_allowable_grid_locs(cmap_arr, costmap_vacancy_threshold, grid_cell_size, robot_type,
                                             search_polygon_msg)
                if self.ground_truth_threats is None:
                    self.set_ground_truth_threats()

            t1 = time.time()
            rospy.loginfo("One time check search polygon total time: %f" % (t1 - t0))
            rospy.loginfo("Number of points to consider on this go round: %d" % int(round(rospy.get_param("~nats_cell_selection_prob")*len(self.allowable_grid_locations))))
            cells_this_round = [self.allowable_grid_locations[i] for i in sorted(random.sample(range(len(self.allowable_grid_locations)), int(round(rospy.get_param("~nats_cell_selection_prob")*len(self.allowable_grid_locations)))))]

            # print(np.roll(np.array(cells_this_round).T, 1, axis=0), (self.n2, self.n1))
            # masker = np.ravel_multi_index(np.roll(np.array(self.allowable_grid_locations).T, 1, axis=0), (self.n2, self.n1))
            # This limits the scope/dimension of the matrices to the searcg region only so that computation is quicker
            # the reason this can be done with no issues is that NATS treats neighbouring cells as independent

            masker = np.ravel_multi_index(np.roll(np.array(cells_this_round).T, 1, axis=0), (self.n2, self.n1))
            polygon_masker = np.ravel_multi_index(np.roll(np.array(self.inpolygon_grid_locations).T, 1, axis=0), (self.n2, self.n1))
            
            if len(points_dict['X']): # set when receviing first message about track info
                X, Y, pos = self.extract_and_regularize_points_dict(points_dict, polygon_masker)

                t0 = time.time()
                if search_mode == "rsi":
                    _, pi_0, beta_rsi, beta_hat_full, _ = self.getPosteriorRSI(points_dict["X"], points_dict["Y"], polygon_masker, recv_time)
                    Sig_beta_full = (points_dict['X'].sum(axis=0) == 0).astype(float) * 100
                    Sig_beta_full += (points_dict['X'] * points_dict['Y'][:, 1].reshape(-1, 1)).sum(axis=0)
                    Sig_beta = np.diag(Sig_beta_full.reshape(self.n2, self.n1).T[
                        np.array(self.inpolygon_grid_locations)[:, 0], np.array(self.inpolygon_grid_locations)[:, 1]])
                    beta_hat_full = (points_dict['X'] * points_dict['Y'][:, 0].reshape(-1, 1)).sum(axis=0)
                    beta_hat = beta_hat_full.reshape(self.n2, self.n1).T[
                        np.array(self.inpolygon_grid_locations)[:, 0], np.array(self.inpolygon_grid_locations)[:,
                                                                       1]]  # duplicate variable for compatibility with nats
                    # beta_hat = beta_hat_full.reshape(self.n2, self.n1).T[np.array(self.inpolygon_grid_locations)[:,0], np.array(self.inpolygon_grid_locations)[:,1]]
                else:
                    #guts or star
                    beta_tilde, Gamma, Sig_beta, beta_hat, _, ginv_xtsx, inv_y_one = self.getPosterior(X,Y,pos,recv_time)

                Sig_beta_full = np.zeros(self.n) + 1e+8
                np.put(Sig_beta_full, polygon_masker, np.diag(Sig_beta))
                # Sig_beta_full[Sig_beta_full > 1e6] = 10 # all the max uncertainty cells are dark
                # for the non zero ones, closer to zero should be brighter
                # Sig_beta_full[Sig_beta_full !=0] -= 10
                Sig_beta_full = np.diag(Sig_beta_full)
                # Sig_beta_full[Sig_beta_full]
                # Sig_beta_full = np.diag((points_dict['X'].sum(axis=0)==0).astype(int))
                beta_hat_full = np.zeros(self.n)
                np.put(beta_hat_full, polygon_masker, beta_hat)
                # beta_hat_full = (points_dict['X'] * points_dict['Y'][:, 0].reshape(-1, 1)).sum(axis=0)
                # import ipdb; ipdb.set_trace()
                t1 = time.time()
                rospy.loginfo("Get posterior total time: %f" % (t1 - t0))

                t0 = time.time()
                beta_hat_vis, visibility_cost_full, beta_hat_vis_full = self.get_visibility_cost(Sig_beta, beta_hat.flatten(), X, polygon_masker)
                t1 = time.time()
                rospy.loginfo("Build visibility posterior total time: %f" % (t1 - t0))
                t0 = time.time()
                self.publish_grid(visibility_cost_full[:, :, 0].flatten(), self.visibility_costmap_publisher,
                                  viz_cost=True)
                t1 = time.time()
                rospy.loginfo(" visibility posterior publish total time: %f" % (t1 - t0))
                # print(beta_hat_vis.shape)
                # print(list(beta_hat_vis))
                # print("Visibility Beta statistics: ", Counter(list(beta_hat_vis)))
                # print("Visibility Cost statistics: ", Counter(list(visibility_cost_full[:,:,0].flatten())))



            else:#this branch True on initial evaluations for agents with samples from prior
                print("No observations!")
                beta_tilde = self.sample_from_prior_per_worker(recv_time)
                beta_hat = np.zeros((self.n,1))
                beta_hat_full = np.zeros((self.n,1))
                Sig_beta = np.diag(100*np.ones((self.n)))
                Sig_beta_full = np.diag(100*np.ones((self.n)))
                beta_rsi = self.beta
                pos = np.zeros((self.n_agents)) # sounds like this
                visibility_cost_full = np.ones((self.n2,self.n1,4))

            if search_mode != "rsi":
                # print("Sigma Beta statistics: ", Counter(list(np.diag(Sig_beta_full))))
                self.publish_grid(10 * np.diag(Sig_beta_full), self.sig_beta_pub)
            # threat_idxs = np.array(np.nonzero(np.array(beta_tilde).reshape((self.n2, self.n1))))
            # for num in range(len(threat_idxs[0])):
            #     print("Threats suspected at (beta_tilde): ", threat_idxs[0][num], threat_idxs[1][num] )
            threat_idxs = np.array(np.nonzero(np.array(beta_hat_full).reshape((self.n2, self.n1))))
            for num in range(len(threat_idxs[0])):
                print("Threats suspected at: ", threat_idxs[0][num], threat_idxs[1][num] )

            max_reward = float("-inf")
            k = np.count_nonzero(self.beta)
# <<<<<<< HEAD
            beta_bar = np.zeros((len(beta_hat),1)) # what is beta_bar?
# =======
#             beta_bar = np.zeros((len(self.inpolygon_grid_locations),1)) # what is beta_bar?
# >>>>>>> tejus/evaluate_multirobot
            if self.current_pose:
                if rospy.get_param("~zone_clearing"):
                    # rospy.loginfo("Zone clearing behaviour with fixed start point: %d %d" % (self.oldh, self.oldl))
                    oldh, oldl = self.oldh, self.oldl
                else:
                    oldh, oldl = start2grid(self.current_pose, grid_cell_size)
                # print("Inside NATS: ", (oldl, oldh))
            else:
                # Assumes perfection hence not good code
                oldl = pos[wid] % self.n1 # current length position
                oldh = (pos[wid]-oldl)/self.n1 # current height position

            dist = np.zeros((self.n2,self.n1,4))+1e10 # distance bw current and candidate position placeholder
            loss = np.zeros((self.n2,self.n1,4))+1e10 # similar as above for loss
            avg_loss = 0.001
            avg_dist = 0
            avg_viz = 0
            count = 0
            #put some limits on l and h so that this works faster
            # Search polygon plus some nearness constraints

            if len(points_dict['X']):
                t_sig = 0
                t_b = 0
                t_xTb = 0
                t_bht = 0
                if search_mode != "rsi":
                    X_sparseT = csc_matrix(np.transpose(X))
                    y_inv_one_sparse = csc_matrix(inv_y_one)
                    t_s = time.time()
                    XTyinv_sparse = X_sparseT * y_inv_one_sparse
                    rospy.loginfo("Sparse Matrix multi: %f" % (time.time() - t_s))

            for (l, h) in cells_this_round:
                d = 0
                # Viz for candidate point to see if it is respecting search_polygon
                self.viz_compute_progress(count, h, l)
                # noise_var is something I will have to guess as as I don't have location uncertainty
                # to smooth me over
                if not self.current_waypoint:
                    self.current_waypoint = [oldh, oldl, 0]
                if rospy.get_param("~take_drone_path_into_account"):
                    x_full, nonzero_idx, noise_var = self.create_directional_sensor(l,h,robot_type, l_iam=self.current_waypoint[1], h_iam=self.current_waypoint[0]) # this gives candidate X_t
                else:
                    x_full, nonzero_idx, noise_var = self.create_directional_sensor(l,h,robot_type) # this gives candidate X_t
                # print("l,h: ", l, h, "NONZERO IDX: ", nonzero_idx)

                x = x_full[:, polygon_masker]
                # print(nonzero_idx)
                if len(points_dict['X']): # is true if we have atleast one obs
                    if search_mode == "rsi":
                        # RSI information Gain:
                        p_0 = np.sum(pi_0[[ll for ll in range(self.n) if ll not in nonzero_idx]])
                        p_1 = np.sum(pi_0[nonzero_idx])
                        lamda = self.mu * 1.
                        IG = 0
                        if (np.sum(pi_0) != 0):
                            IG += -(p_0 * np.log(
                                p_0 * ss.norm(0, 1).pdf(0) + p_1 * ss.norm(0, 1).pdf(-lamda)))
                            IG += -(p_1 * np.log(
                                p_0 * ss.norm(0, 1).pdf(lamda) + p_1 * ss.norm(0, 1).pdf(0)))

                        loss[h, l, d] = -IG
                        dist[h, l, d] = (np.absolute(oldl - l) ** 2 + np.absolute(oldh - h) ** 2)

                        avg_loss += loss[h, l, d]
                        avg_dist += dist[h, l, d]
                        count += 1
                    else:
                        #star or guts
                        # Xt = np.append(X,x ,axis=0)
                        # print("Xt dimensions: ", Xt.shape)
                        # Sig_beta including candidate x
                        # Sig = np.linalg.inv(np.matmul(np.matmul(np.transpose(Xt),
                        #                                              np.diag(1./np.append(np.squeeze(Y[:,1]),noise_var))),Xt)+np.linalg.inv(Gamma))
                        # print("Gamma inside Active search: ", Gamma)
                        t0 = time.time()
                        # t_multi = time.time()
                        ginv_xtsx_t = ginv_xtsx + np.matmul(np.matmul(np.transpose(x), np.diag(1. / np.array(noise_var))),x)
                        # rospy.loginfo("Multiplication time: %f" % (time.time() - t_multi))
                        Sig = np.diag(1 / np.diag(ginv_xtsx_t))
                        t1 = time.time()
                        t_sig+=t1 - t0
                        #
                        t_matmul = time.time()
                        b = csc_matrix(Sig) * hstack([XTyinv_sparse, csc_matrix(np.matmul(np.transpose(x), np.diag(1. / np.array(noise_var))))])
                        # b = np.matmul(np.matmul(Sig,np.transpose(Xt)),
                        #                      np.diag(1./np.append(np.squeeze(Y[:,1]),noise_var))) # true y's are missing, this is not exactly beta_hat
                        t_b += time.time() - t_matmul
                        # print("x.shape[0]: ", x.shape[0])
                        b1 = b[:,:-1*x.shape[0]] # this is the portion of b associated with obs we already have
                        b2 = b[:,-1*x.shape[0]:] # this is hte portion of b associated with the candidate x
                        # print("This: ", b1.shape, " and this: ", np.reshape(Y[:,0],(-1,1)).shape, " should be multipliable and of dimension: ", beta_tilde.shape)
                        t_matmul = time.time()
                        xTb = np.matmul(x,beta_tilde) # assume this is the y we will receive
                        t_xTb += time.time() - t_matmul

                        # # norm 2 loss:
                        # b1Y_b = np.matmul(b1,np.reshape(Y[:,0],(-1,1)))-beta_tilde
                        # loss[h,l,d] = (np.linalg.norm(b1Y_b)**2+\
                        #               (np.sum(noise_var)+np.linalg.norm(xTb)**2)*(np.linalg.norm(b2)**2)+\
                        #               np.ndarray.item(2*np.matmul(np.matmul(np.transpose(b1Y_b),b2),xTb)))\
                        #               /(np.linalg.norm(np.matmul(b1,np.reshape(Y[:,0],(-1,1))))**2+\
                        #                             (np.sum(noise_var)+np.linalg.norm(xTb)**2)*(np.linalg.norm(b2)**2)+\
                        #                             np.ndarray.item(2*np.matmul(np.matmul(np.transpose(b1Y_b),b2),xTb)))

                        # loss with beta_expectation:
                        t = time.time()
                        beta_hat_t = np.matmul(b1.todense(),np.reshape(Y[:,0],(-1,1)))+np.matmul(b2.todense(),xTb) #finally we get beta_hat candidate
                        t_bht += time.time() - t
                        est_t = (np.round(beta_hat_t)>(np.amax(beta_hat_t)/2)) #
                        real_t = (np.round(beta_tilde)>(np.amax(beta_tilde)/2))
                        # changed norm 2 loss
                        loss[h,l,d] = 1*(np.linalg.norm(beta_hat_t-beta_tilde,2)+0.01*int(~np.all(est_t==real_t)))
                        # print("Loss at position: ", [h,l,d], " is ", loss[h,l,d])
                        # # norm 1 loss:
                        # xTb = np.matmul(x,beta_tilde)
                        # Si = np.reshape(noise_var,(-1,1))
                        # E_abs_y = np.sqrt(2/math.pi)*np.multiply(np.sqrt(Si),np.exp(-(xTb**2/(2*Si))))\
                        #     +np.multiply(xTb,ssp.erf(xTb/np.sqrt(2*Si)))
                        # loss[h,l,d] = np.linalg.norm(b1Y_b,1)+np.sum(np.matmul(np.absolute(b2),E_abs_y))
                        # print("Old l: ", oldl, " Old h: ", oldh)
                        dist[h,l,d] = np.sqrt(np.absolute(oldl-l)**2+np.absolute(oldh-h)**2)
                        # print("Dist at position: ", [h,l,d], " is ", dist[h,l,d])
                else:
                    # Can default to random waypoint planner behaviour here bec it is not very important
                    Vinv = np.eye(polygon_masker.shape[0])*1e3
                    tmp = np.linalg.inv(np.diag(noise_var)+np.matmul(np.matmul(x,Vinv),np.transpose(x)))
                    b = np.matmul(np.matmul(Vinv,np.transpose(x)),tmp)
                    b2 = b
                    b1Y_b = -beta_tilde
                #                        #norm 2 loss:
                #                        reward = -1*(np.linalg.norm(beta_tilde)**2+
                #                                     0.01*(self.sigma2*x.shape[0]+np.linalg.norm(xTb)**2)*(np.linalg.norm(b)**2)-
                #                                     np.ndarray.item(2*np.matmul(np.matmul(np.transpose(beta_tilde),b),xTb)))

                    # loss with expectation:
                    xTb = np.matmul(x,beta_tilde[polygon_masker,:])
                    beta_hat_t = np.matmul(b,xTb)-np.matmul(b,np.matmul(x,beta_bar))+beta_bar
                    est_t = (np.round(beta_hat_t)>(np.amax(beta_hat_t)/2))
                    real_t = (np.round(beta_tilde)>(np.amax(beta_tilde)/2))
                    loss[h,l,d] = (np.linalg.norm(beta_hat_t-beta_tilde[polygon_masker,:],1)+0.01*int(~np.all(est_t==real_t)))
                    dist[h,l,d] = 0.001
                    # print("Loss at position: ", [h,l,d], " is ", loss[h,l,d])
                    # print("Dist at position: ", [h,l,d], " is ", dist[h,l,d])

                avg_loss += loss[h,l,d]
                avg_dist += dist[h,l,d]
                avg_viz += visibility_cost_full[h,l,d]
                count += 1

            if len(points_dict['X']):
                rospy.loginfo("b time: %f" % (t_b))
                rospy.loginfo("sig inverse total time: %f" % (t_sig))
                rospy.loginfo("xTb time: %f" % (t_xTb))
                rospy.loginfo("beta_hat_t time: %f" % (t_bht))

            # print("Num of candidate points checked (shouldn't vary): ", count)
            avg_loss /= count
            avg_dist /= count
            avg_viz /= count

            cells_this_round_arr = np.array(cells_this_round)

            # heuristically determine the correct distance weightage
            # reward = - loss/avg_loss - rospy.get_param("~distance_cost_weightage")*dist/avg_dist - rospy.get_param("~visibility_cost_weightage")*visibility_cost_full/avg_viz
            reward = np.zeros((self.n2,self.n1,4))-1e10
            reward_lite = - self.normalize_ptp(loss[cells_this_round_arr[:,1],cells_this_round_arr[:,0],0]) - \
                     rospy.get_param("~distance_cost_weightage") * self.normalize_ptp(dist[cells_this_round_arr[:,1],cells_this_round_arr[:,0], 0]) - \
                     rospy.get_param("~visibility_cost_weightage") * self.normalize_ptp(visibility_cost_full[cells_this_round_arr[:,1],cells_this_round_arr[:,0], 0])

            # np.ravel_multi_index(np.concatenate([np.roll(np.array(cells_this_round).T, 1, axis=0), np.zeros((1,len(cells_this_round_arr)))], axis=0), (self.n2, self.n1, 4))
            reward_tmp = reward[:,:,0]
            np.put(reward_tmp, masker, reward_lite)
            reward[:,:,0] = reward_tmp
            # print("NATS loss statistics: ", Counter(list(self.normalize_ptp(loss[cells_this_round_arr[:,1],cells_this_round_arr[:,0], 0]).flatten())))
            # print("NATS OG loss statistics: ", Counter(list((loss[cells_this_round_arr[:,1],cells_this_round_arr[:,0], 0]/avg_loss).flatten())))
            # print("Dist loss statistics: ", Counter(list(self.normalize_ptp(dist[cells_this_round_arr[:,1],cells_this_round_arr[:,0], 0]).flatten())))
            # print("Dist OG loss statistics: ", Counter(list((dist/avg_dist)[cells_this_round_arr[:,1],cells_this_round_arr[:,0], 0].flatten())))
            # print("Visibility loss statistics: ", Counter(list(self.normalize_ptp(visibility_cost_full[cells_this_round_arr[:,1],cells_this_round_arr[:,0], 0]).flatten())))
            # print("Visibility OG loss statistics: ", Counter(list((visibility_cost_full/avg_viz)[cells_this_round_arr[:,1],cells_this_round_arr[:,0], 0].flatten())))


            # print("Reward statistics: ", Counter(list(-reward.flatten())))

            # print(loss/avg_loss)
            # print(dist/avg_dist)

            print("Shape of reward: ", reward.shape)
            [hm,lm,dm] = np.unravel_index(reward.argmax(), reward.shape)
            # [hm, lm] = cells_this_round_arr[reward.argmax()]
            # dm = 0

        threat_locs = [self.inpolygon_grid_locations[j] for j in np.nonzero(beta_hat.squeeze())[0]]

        if rospy.get_param("~logging"):
            precision, recall = None, None
            # if self.ground_truth_threats:
            #     detections = set(threat_locs)
            #     precision = 1.0*len(ground_truth & detections)/len(detections)
            #     recall = 1.0*len(ground_truth & detections)/len(ground_truth)

            self.save_log(
                {
                    'robot_name': self.robot_name,
                    'time': time.time() - self.start_wall_time,
                    'runtime': rospy.get_time() - self.start_time,
                    'detected_threats': threat_locs,
                    'num_waypoints': len(self.robot_goals),
                    'path': copy.deepcopy(self.robot_path),
                    'path_dict': copy.deepcopy(self.robot_path_dict),
                    'path_timestamps': copy.deepcopy(self.robot_path_timestamps),
                    'path_timestamps_dict': copy.deepcopy(self.robot_path_timestamps_dict),
                    'precision': precision,
                    'recall': recall,
                    'waypoint_selection_time': rospy.get_time() - start_planning_time
                }, np.diag(Sig_beta_full), beta_hat_full, beta_hat_vis_full
            )

        if len(threat_locs) > 0:
            threats = ThreatVector()
            threats.header.stamp = rospy.Time().now()
            threats.header.frame_id = "grid"
            threats.robot_name = self.robot_name
            threats.num_waypoints = len(self.robot_goals)
            for i in range(len(threat_locs)):
                print("Threats suspected at: ", threat_locs[i])
                threats.threat_list.append(Point(x=threat_locs[i][1], y=threat_locs[i][0]))

            self.threat_publisher.msg = threats
            self.threat_publisher.publish()

        rospy.loginfo("Number of allowable cells in the search region: "+ str(len(self.allowable_grid_locations)))
        rospy.loginfo("Best location in grid based on reward is: " + str([hm,lm,dm]))
        self.current_waypoint = [hm,lm,dm]
        bestx,bestidx,best_noise_var = self.create_directional_sensor(lm,hm,dm)
        pos[wid] = float(hm*self.n1+lm) # position in the unrolled vector of the map

        best_x_map, best_y_map = grid2map((lm, hm), rospy.get_param("~cell_size"))
        best_pose = Pose()
        best_pose.position.x = best_x_map
        best_pose.position.y = best_y_map
        # best_pose_start = transform_pose(best_pose,
        #                                  rospy.get_param("~robot_name") + rospy.get_param("~map_frame"),
        #                                  rospy.get_param("~robot_name") + rospy.get_param("~start_frame"))
        color = ColorRGBA()
        color.r = count % 2
        color.g = (count + 1) % 2
        self.marker_publisher.msg.colors.append(color)
        self.marker_publisher.msg.points.extend([best_pose.position])
        self.marker_publisher.publish()

        if len(points_dict['X']):
            print("Beta hat statistics: ", Counter(list(beta_hat_full)))
            self.publish_grid((100*beta_hat_full/np.max(beta_hat_full)).astype(int), self.beta_hat_pub)
        self.step +=1
        if not len(points_dict['X']):
            result = {'x':[bestx], 'grid_pos':[hm, lm, dm], 'par': [beta_hat,pos], 'pre-eval':True}
        else:
            result = {'x':bestx,'grid_pos':[hm, lm, dm], 'par':[beta_hat,pos]}

        print("Previoous loc from current pose or something: ", (oldl, oldh), "Previous loc saved in last_loc member", self.last_loc)
        if (oldl,oldh) == self.last_loc:
            if search_mode == "rsi":
                print("IS OMPL GIVING YOU A PLANNING ERROR????")
                result['grid_pos'][1] += 1
                result['grid_pos'][0] += 1
            del self.robot_goals[-1]
        
        self.last_loc = (oldl, oldh)
        self.robot_goals.append((result['grid_pos'][1], result['grid_pos'][0]))

        # visualizes belief mean and variance, NATS loss and total reward for each cell in the grid

        if rospy.get_param("~logging") and (search_mode == "random" or search_mode == "coverage" or len(points_dict["X"]) == 0):
            dist = loss = reward = visibility_cost_full = np.random.randn(*(self.n2, self.n1, 4))
            # print(Sig_beta.shape, beta_hat.shape, beta_hat_vis.shape, cells_this_round, dist.shape, loss.shape, reward.shape,
            #                  visibility_cost_full.shape)
            self.logging_viz(Sig_beta, beta_hat.reshape(-1,1), beta_hat_vis, cells_this_round, dist, loss, reward,
                             visibility_cost_full)

            #result['grid_pos'] = [4, 13, 0] # set goal close to spawn location on Gascola map; useful for debugging
        elif rospy.get_param("~logging"):
            #star guts or rsi
            # print(Sig_beta.shape, beta_hat.shape, beta_hat_vis.shape, cells_this_round, dist.shape, loss.shape, reward.shape,
            #                  visibility_cost_full.shape)
            # try:
            self.logging_viz(Sig_beta, beta_hat.reshape(-1,1), beta_hat_vis, cells_this_round, dist, loss, reward,
                                 visibility_cost_full)
            # except Exception as e:
            #     print("Error with logging")
            #     print(e)
        #Convert best x to desired waypoint goal
        return result

    def set_ground_truth_threats(self):
        self.ground_truth_threats = []
        k = self.k
        self.probs_for_threats = np.array(self.probs_for_threats) / np.sum(self.probs_for_threats)
        # median = np.median(self.probs_for_threats)
        # self.probs_for_threats *= self.probs_for_threats > median
        # self.probs_for_threats = self.probs_for_threats / np.sum(self.probs_for_threats)

        trl = rospy.get_param("~random_seed")
        print("NATS RANDOM SEED RECEIVED IS: ", trl, "Numebr in poly grid locs: ", len(self.inpolygon_grid_locations))
        rng = np.random.RandomState(trl)
        if rospy.get_param("~threat_sampling") == "stealth":
            print("WEEE USING THE STEALTH MODE \n\n\n")
            idx = rng.choice(len(self.inpolygon_grid_locations), k, replace=False, p=self.probs_for_threats)
        else:
            idx = rng.choice(len(self.inpolygon_grid_locations), k, replace=False)
        mu = 1
        # beta = np.zeros((self.n, 1))
        print("Sampled threats at:")
        self.threat_locations_ground_truth = []
        for i in idx:
            self.beta[self.inpolygon_grid_locations[i][1] * self.n1 + self.inpolygon_grid_locations[i][0], 0] = mu
            print("%d,%d" % (self.inpolygon_grid_locations[i][0], self.inpolygon_grid_locations[i][1]))
            self.ground_truth_threats.append((self.inpolygon_grid_locations[i][0], self.inpolygon_grid_locations[i][1]))
        pkl.dump(self.ground_truth_threats, open(self.logging_path + "ground_truth_threats.pkl", "wb"))

        # hidden_prior = 1 - self.visibility_prior
        # plt.imshow(hidden_prior)
        # print("\n\n LOOOOOOK HERE:    \n\n")
        # for i in range(1, 110):
        #     print(i)
        #     rng = np.random.RandomState(i)
        #     # idx = rng.choice(len(self.inpolygon_grid_locations), k, replace=False)
        #     idx = rng.choice(len(self.inpolygon_grid_locations), k, replace=False, p=self.probs_for_threats)
        #     for id in idx:
        #         plt.scatter(120 * self.inpolygon_grid_locations[id][1]+10*np.abs(np.random.randn())+60,
        #                     np.abs(hidden_prior.shape[0] - 120 * self.inpolygon_grid_locations[id][0])+60+10*np.abs(np.random.randn()),
        #                     s=1)
        # plt.savefig("/home/nabakshi/src/zone_recon/hidden_prior.png")
        # plt.close()

    def logging_viz(self, Sig_beta, beta_hat, beta_hat_vis, cells_this_round, dist, loss, reward, visibility_cost_full):
        viridis = cm.get_cmap('viridis')
        fig, ax = plt.subplots(2, 4)  # , figsize=(12, 4))
        ax = ax.flatten()
        vis = np.zeros((self.n1, self.n2, 3))
        for i, j in self.allowable_grid_locations:
            vis[i, j, :] = [0.5625, 0.9296, 0.5625]
        for i, j in self.robot_path:
            if (i < self.n1 and j < self.n2) and (i>0 and j>0):
                vis[i, j, :] = [1, 0.64, 0]
        for i, j in self.robot_goals:
            vis[i, j, :] = [1, 0, 0]
        for i, j in self.ground_truth_threats:
            vis[i, j, :] = [1, 0.64, 1]

        ax[0].imshow(np.flipud(vis))
        ax[0].set_title("robot path")
        belief_viz = np.zeros((self.n1, self.n2, 3))
        belief_var_viz = np.zeros((self.n1, self.n2, 3))
        loss_viz = np.zeros((self.n1, self.n2, 3))
        reward_viz = np.zeros((self.n1, self.n2, 3))
        visibility_viz = np.zeros((self.n1, self.n2, 3))
        viz_loss_viz = np.zeros((self.n1, self.n2, 3))
        dist_loss_viz = np.zeros((self.n1, self.n2, 3))
        loss = np.array([-loss[j, i, 0] for (i, j) in cells_this_round])
        viz_loss = np.array([-visibility_cost_full[j, i, 0] for (i, j) in cells_this_round])
        dist_loss = np.array([-dist[j, i, 0] for (i, j) in cells_this_round])
        reward = np.array([reward[j, i, 0] for (i, j) in cells_this_round])
        belief_var = np.array([Sig_beta[i, i] for i in range(len(self.inpolygon_grid_locations))])
        beta_hat_vis = np.array([beta_hat_vis[i] for i in range(len(self.inpolygon_grid_locations))])
        loss = self.normalize_ptp(loss)
        viz_loss = self.normalize_ptp(viz_loss)
        dist_loss = self.normalize_ptp(dist_loss)
        reward = self.normalize_ptp(reward)
        belief_var = self.normalize_ptp(belief_var)
        beta_hat_vis = self.normalize_ptp(beta_hat_vis)
        k1 = 0
        k2 = 0
        for j in range(self.n2):
            for i in range(self.n1):
                if (i, j) in self.inpolygon_grid_locations:
                    belief_viz[i, j, :] = viridis(beta_hat[k1])[0, :3]
                    belief_var_viz[i, j, :] = viridis(belief_var[k1])[:3]
                    visibility_viz[i, j, :] = viridis(beta_hat_vis[k1])[:3]

                    k1 += 1

                if (i, j) in cells_this_round:
                    loss_viz[i, j, :] = viridis(loss[k2])[:3]
                    viz_loss_viz[i, j, :] = viridis(viz_loss[k2])[:3]
                    dist_loss_viz[i, j, :] = viridis(dist_loss[k2])[:3]
                    reward_viz[i, j, :] = viridis(reward[k2])[:3]

                    k2 += 1
        ax[1].imshow(np.flipud(belief_viz))
        ax[1].set_title("posterior mean")
        ax[2].imshow(np.flipud(belief_var_viz))
        ax[2].set_title("posterior var")
        ax[3].imshow(np.flipud(loss_viz))
        ax[3].set_title("NATS reward")
        ax[4].imshow(np.flipud(viz_loss_viz))
        ax[4].set_title("viz_loss")
        ax[5].imshow(np.flipud(dist_loss_viz))
        ax[5].set_title("dist_loss")
        ax[6].imshow(np.flipud(reward_viz))
        ax[6].set_title("reward")
        ax[7].imshow(np.flipud(visibility_viz))
        ax[7].set_title("visibility_viz")
        plt.savefig(self.logging_path + "viz_" + str(self.step) + ".png", dpi=300)
        plt.close()

    def extract_and_regularize_points_dict(self, points_dict, polygon_masker=None):
        if polygon_masker is not None:
            X = points_dict['X'][:, polygon_masker]
        else:
            X = points_dict["X"]
        Y = points_dict['Y']
        [_, pos] = points_dict['par']
        print("Num observations so far: ", len(X))
        print("Num detection list so far (should match above): ", len(Y))
        print("X shape: ", X.shape, " Y shape: ", Y.shape)
        if len(X) < len(Y):
            rospy.loginfo("More Y observations than X, fixing")
            extra = len(Y) - len(X)
            Y = Y[:-extra, :]
        elif len(X) > len(Y):
            rospy.loginfo("More X observations than Y, fixing")
            extra = len(X) - len(Y)
            X = X[:-extra, :]
        print(X.shape)
        print(Y.shape)
        return X, Y, pos

    def coverage_mode(self, cmap_arr, costmap_vacancy_threshold, grid_cell_size, new_polygon, points_dict, robot_type,
                      search_mode, search_polygon_msg):
        if new_polygon:
            self.set_allowable_grid_locs(cmap_arr, costmap_vacancy_threshold, grid_cell_size, robot_type,
                                         search_polygon_msg)
            self.set_ground_truth_threats()
        cells_this_round = [self.allowable_grid_locations[i] for i in sorted(
            random.sample(range(len(self.allowable_grid_locations)), int(
                round(rospy.get_param("~nats_cell_selection_prob") * len(self.allowable_grid_locations)))))]

        count = 0
        selectable_idxs = []
        X, Y, pos = self.extract_and_regularize_points_dict(points_dict)
        unvisited_idxs = np.array(
            np.nonzero(np.array(np.transpose(np.array(X.sum(axis=0)).reshape((self.n2, self.n1))) == 0)))
        Sig_beta_full = (X.sum(axis=0) == 0).astype(float) * 100
        self.publish_grid(Sig_beta_full, self.sig_beta_pub)
        Sig_beta_full += (X * Y[:, 1].reshape(-1, 1)).sum(axis=0)
        Sig_beta = Sig_beta_full.reshape(self.n2, self.n1).T[np.array(self.inpolygon_grid_locations)[:,0], np.array(self.inpolygon_grid_locations)[:,1]]
        beta_hat_full = (X * Y[:, 0].reshape(-1, 1)).sum(axis=0)
        beta_hat = beta_hat_full.reshape(self.n2, self.n1).T[np.array(self.inpolygon_grid_locations)[:,0], np.array(self.inpolygon_grid_locations)[:,1]]  # duplicate variable for compatibility with nats
        # high_uncertainty_idxs = np.array(np.nonzero(np.array(np.diag(Sig_beta) > 1e06).reshape((self.n2, self.n1))))
        for num in range(len(unvisited_idxs[0])):
            l, h = unvisited_idxs[0][num], unvisited_idxs[1][num]
            # Same old search polygon and hence rely on memory
            if (l, h) not in self.allowable_grid_locations:
                continue
            # print("High uncertainty cell at: ",  l, h)
            selectable_idxs.append((l, h))
            # Viz for candidate point to see if it is respecting search_polygon
            self.viz_compute_progress(count, h, l)
            count += 1
        print("Number of selectable cells: ", len(selectable_idxs))

        if len(selectable_idxs) and search_mode == "coverage":
            lm, hm = random.choice(selectable_idxs)
        else:
            rospy.loginfo("Infinite random search...")
            lm, hm = random.choice(self.allowable_grid_locations)
        dm = 0

        return np.diag(Sig_beta), np.diag(Sig_beta_full), beta_hat, beta_hat_full, cells_this_round, count, dm, hm, lm, pos

    def viz_compute_progress(self, count, h, l):
        color = ColorRGBA()
        color.r = count % 2
        color.b = (count + 1) % 2
        cand_x_map, cand_y_map = grid2map((l, h), rospy.get_param("~cell_size"))
        cand_pose = Pose()
        cand_pose.position.x = cand_x_map
        cand_pose.position.y = cand_y_map
        # cand_pose_start = transform_pose(cand_pose,
        #                                  rospy.get_param("~robot_name") + rospy.get_param("~map_frame"),
        #                                  rospy.get_param("~robot_name") + rospy.get_param("~start_frame"))
        self.marker_publisher.msg.colors.append(color)
        self.marker_publisher.msg.colors = self.marker_publisher.msg.colors[-20:]
        self.marker_publisher.msg.points.extend([cand_pose.position])
        self.marker_publisher.msg.points = self.marker_publisher.msg.points[-20:]
        self.marker_publisher.publish()

    def set_allowable_grid_locs(self, cmap_arr, costmap_vacancy_threshold, grid_cell_size, robot_type,
                                search_polygon_msg):
        # hidden_prior =
        # hidden_prior = image.
        costmap_cell_size = rospy.get_param("~costmap_cell_size")
        self.probs_for_threats = []
        for h in range(0, self.n2):
            for l in range(0, self.n1):
                for d in [0]:  # direction of travel, NSEW, I should use NE,SW,etc.
                    cand_x_map, cand_y_map = grid2map((l, h), grid_cell_size)
                    cand_pose = Pose()
                    cand_pose.position.x = cand_x_map
                    cand_pose.position.y = cand_y_map
                    if not is_within_search_region(cand_pose.position, search_polygon_msg):
                        # applies to ugv and uas
                        # print("Skipping potential point: ", cand_pose_start.position)
                        continue
                    else:
                        self.inpolygon_grid_locations.append((l, h))
                        # ln, hn = grid2grid((l,h), grid_cell_size, costmap_cell_size)
                        # print(h,l, self.n2, self.n1)
                        self.probs_for_threats.append(self.probability_prior[l, h])
                        if robot_type == "ugv":
                            cmap_slice = costmap2slice((l, h), cmap_arr, grid_cell_size)
                            # rospy.loginfo("LOOK here for values: " + str(np.count_nonzero(cmap_slice)/(cmap_slice.shape[0] * cmap_slice.shape[1])) )
                            # print("cmap_slice non-zero entries at ", (l,h), " : ", np.count_nonzero(cmap_slice), "should be MAX 3600")
                            if float(np.count_nonzero(cmap_slice))/(cmap_slice.shape[0] * cmap_slice.shape[1]) > (1 - costmap_vacancy_threshold):
                                # If the non-zero cost cells exceed a certain threshold a.k.a not freespace cells
                                continue
                    self.allowable_grid_locations.append((l, h))
                    # Same old search polygon and hence rely on memory
                    if (l, h) not in self.allowable_grid_locations:
                        continue

    def publish_grid(self, data_vector, publisher, viz_cost=False):
        heatmap = OccupancyGrid()
        if not viz_cost:
            heatmap.data = list(np.clip(np.array(np.transpose(data_vector.reshape((self.n2, self.n1)))).reshape(-1, 1), 0, 100).flatten().astype(int))
            # print("heatmap max: ", max(heatmap.data), " min: ", min(heatmap.data), "heatmap stats: ", Counter(heatmap.data))
            print(publisher)
            heatmap.info.height = self.n1
            heatmap.info.width = self.n2
            heatmap.info.resolution = rospy.get_param("~cell_size")

        else:
            msg = Float32MultiArray()
            if data_vector is not None:
                msg.data = data_vector.tolist()
            else:
                msg.data = np.zeros((self.n2, self.n1)).flatten().tolist()
            publisher.msg = msg
            publisher.publish()
            return
            # heatmap.info.height = self.n1
            # heatmap.info.width = self.n2
            # heatmap.info.resolution = rospy.get_param("~cell_size")
        # heatmap.data = list(np.repeat(np.array(np.clip(np.diag(Sig_beta.astype(int)), 0, 100)), int(rospy.get_param("~cell_size")/rospy.get_param("~costmap_cell_size"))))[:19 * 47**3]
        # print(heatmap.data.reshape(47**2, 47*19))
        origin = Pose()
        origin.position.x = 0
        origin.position.y = 0
        origin.position.z = 0
        origin.orientation.x = 0
        origin.orientation.y = 0
        origin.orientation.z = 0
        origin.orientation.w = 1
        heatmap.info.origin = origin
        heatmap.header.stamp = rospy.Time.now()
        heatmap.header.frame_id = self.robot_name + rospy.get_param("~map_frame")
        publisher.msg = heatmap
        publisher.publish()

    def create_directional_sensor(self,l,h,d, robot_type=None, l_iam=None, h_iam=None):
        """
        We are assuming that the map is aligned with cardinal directions, that means for
        gascola the map looks something like:

         ________
        |   |   |
        |   |   |   North up^
        |   ^   |
        |  / \  |
        | |  |  |
        | `-'   |
        --------
        @param l: This is the y coordinate in coarse grid space, in this case it is the longer dimension and is hence l
        @param h: This is the x coordinate in coarse grid space, in this case it is the shorter dimension and hence h
        @param d: This the direction of facing, to be change in the future, for now 0=East, 1=North, 2=West, 3=South,
                  The reason for the strangeness of the convention is that previously East was taken to be the 'UP' direction
                  This can also be overloaded with the robot_type instead and it will still work. This is done at planning
                  time when the omniview assumption is considered, a.k.a the information from all four views are considered
                  when selecting the next waypoint
        @param robot_type: "uas" or "ugv"
        @param l_iam: Only relevant for the drone, this explicitly provides the starting location of the drone so that
                      information along the path can be accounted for. Y-coordinate in grid space
        @param h_iam: Same as above, X-coordinate in grid space
        @return:
        """
        #Incorporate the search polygon information here when computing the candidates
#        l = i % n1
#        h = (i-l)/n1
#         print("l: ", l, "h: ", h)
        if not robot_type:
            robot_type = rospy.get_param("~robot_type")
        if robot_type == "uas":
            # drone
            # print("drone viewshed")
            if l_iam is None and h_iam is None:
                l_iam = l
                h_iam = h
            view = bresenham_line_gen((h_iam, l_iam), (h, l))
            #Very slow viz code
            # count = 0
            # for (x, y) in view:
            #     if count:
            #         color = ColorRGBA()
            #         color.r = count % 2
            #         color.b = (count + 1) % 2
            #         cand_x_map, cand_y_map = grid2map((y, x), rospy.get_param("~cell_size"))
            #         cand_pose = Pose()
            #         cand_pose.position.x = cand_x_map
            #         cand_pose.position.y = cand_y_map
            #         # cand_pose_start = transform_pose(cand_pose,
            #         #                                  rospy.get_param("~robot_name") + rospy.get_param("~map_frame"),
            #         #                                  rospy.get_param("~robot_name") + rospy.get_param("~start_frame"))
            #
            #         self.marker_publisher.msg.colors.append(color)
            #         self.marker_publisher.msg.points.extend([cand_pose.position])
            #         self.marker_publisher.publish()
            #     count+=1
            viewsheds = {"l":[[point[1] for point in view] for idx in range(4)],
                         "h":[[point[0] for point in view] for idx in range(4)]} # looking straight down
        else:
            # ground robot
            # print("ugv viewshed")
            ######## ----- COnvservative FOV ------ ##########
            if self.fov_setting == "conservative":
                viewsheds = {"l":[[l, l], [l+1, l+2], [l, l], [l-1, l-2]],
                             "h":[[h+1, h+2], [h, h], [h-1, h-2],[h, h]]} # up, right, down, left
            elif self.fov_setting == "viewshed":
                viewsheds = {"l": [[l, l-1, l, l+1,   l-2, l-1, l, l+1, l+2,   l-3, l-2, l-1, l, l+1, l+2, l+3],
                                   [l, l+1,l+1, l+1,  l+2,l+2,l+2,l+2,l+2,     l+3,l+3,l+3,l+3,l+3,l+3,l+3],
                                   [l, l-1,l,l+1,l-2,l-1,l,l+1,l+2,l-3,l-2,l-1,l,l+1,l+2,l+3],
                                   [l, l-1,l-1,l-1,l-2,l-2,l-2,l-2,l-2,l-3,l-3,l-3,l-3,l-3,l-3,l-3]],

                             "h": [[h, h+1,h+1,h+1,h+2,h+2,h+2,h+2, h+2, h+3,h+3,h+3,h+3,h+3,h+3, h+3],
                                   [h, h+1,h, h-1,   h+2,h+1,h,h-1,h-2,   h+3,h+2,h+1,h,h-1,h-2, h-3],
                                   [h, h-1,h-1,h-1,h-2,h-2,h-2,h-2,h-2,h-3,h-3,h-3,h-3,h-3,h-3,h-3],
                                   [h, h-1,h,h+1,h-2,h-1,h,h+1,h+2,h-3,h-2,h-1,h,h+1,h+2,h+3]]}  # up, right, down, left
            else:
                rospy.loginfo("Who do you, Who do you, Who do you, WHo do you think you are? ha ha ha wrong fov_setting in nats_param.yaml file")

        if d in [0, 1, 2, 3]:
            ls = np.array(viewsheds["l"][d])
            hs = np.array(viewsheds["h"][d])
            # ROBOGRID is consistent with above weird convention
            # print(self.robogrid[h, l]["viewshed"])
            # visibility = self.robogrid[h, l]["viewshed"][(d- 1) % 4] # this is a dict
            # print("Visibility in direction, ", (d- 1) % 4, " is ", visibility)
            if h < self.rg_xlim and l < self.rg_ylim:
                visibility = self.robogrid[h, l]["viewshed"][d] # this is a dict
            # print("Visibility in direction, ", d, " is ", visibility)
        elif robot_type == "ugv":
            ######## ----- COnvservative FOV ------ ##########
            #             [ ]
            #             [ ]
            #       [ ][ ] X [ ][ ]
            #             [ ] `\_ Robot loc
            #             [ ]
            ######## ----- Viewshed FOV ------ ##########
            #       [ ][ ][ ][ ][ ]
            #    [ ]   [ ][ ][ ]   [ ]
            #    [ ][ ]   [ ]   [ ][ ]
            #    [ ][ ][ ] X [ ][ ][ ]
            #    [ ][ ]   [ ]   [ ][ ]   X = Robot loc
            #    [ ]   [ ][ ][ ]   [ ]
            #       [ ][ ][ ][ ][ ]

            #    [=][ ][ ][ ][ ][ ][=]
            #    [ ][=][ ][ ][ ][=][ ]
            #    [ ][ ][=][ ][=][ ][ ]
            #    [ ][ ][ ] X [ ][ ][ ]
            #    [ ][ ][=][ ][=][ ][ ]   X = Robot loc
            #    [ ][=][ ][ ][ ][=][ ]   = is double counted
            #    [=][ ][ ][ ][ ][ ][=]
            # looking off the edges of the map get handled at end of this function
            ls = np.array([item for sublist in viewsheds["l"] for item in sublist])
            hs = np.array([item for sublist in viewsheds["h"] for item in sublist])
            if h < self.rg_xlim and l < self.rg_ylim:
                visibility = {item:self.robogrid[h, l]["viewshed"][subdictkey][item] for subdictkey in self.robogrid[h, l]["viewshed"] for item in self.robogrid[h, l]["viewshed"][subdictkey]} # this is a dict
            # print("Ls, hs: ", ls, hs)
            # print("Omni visibility", visibility)

        elif robot_type == "uas":
            # print("drone position")
            #
            #             [ X ]
            #                 `\_ Robot Location
            # Looking straight down below the robot, sees roughly a 22x22m cell
            ls = np.array(viewsheds["l"][0])
            hs = np.array(viewsheds["h"][0])
        else:
            print('wrong d parameter')

        non_zero_idx = []
        non_zero_2idx = []
        noise_var = []
        count = 0
        for ii in range(0, len(ls)):
            if(ls[ii]<self.n1 and ls[ii]>=0 and hs[ii]<self.n2 and hs[ii]>=0):
                pos = int(hs[ii]*self.n1+ls[ii])
                non_zero_2idx.append((hs[ii], ls[ii]))
                non_zero_idx.append(pos)
                # noise_var.append(self.noise_vec[robot_type][ii % len(self.noise_vec[robot_type])])
                noise_var.append(self.noise_vec[robot_type][max(abs(ls[ii]-l), abs(hs[ii]-h))])
                count = count+1
        x = np.zeros((count, self.n))

        # visibility_picker = lambda viewshed, idx:
        if robot_type == "uas" or self.fov_setting == "conservative":
            for jj in range(count):
                x[jj,non_zero_idx[jj]] = 1 # * self.robogrid[h,l]["viewshed"] #need to convert non_zero_idx[jj]
#        print(i,d)
        else:
            to_keep_idxs = []
            for jj in range(count):
                # It's important to do this because the robogrid space has the same origin but can have a
                # different extent than the grid space
                rgx, rgy = non_zero_2idx[jj]
                # print("loooking for visibility at position: ", rgx, rgy, " aka ", rgx+rgy*self.rg_xlim)
                x[jj, non_zero_idx[jj]] = 1
                if rgx < self.rg_xlim and rgy < self.rg_ylim:
                    if visibility.get(rgx+rgy*self.rg_xlim, None) is None and ((h, l) != (rgx, rgy)):
                        rospy.loginfo("[WARNING] Robogrid coordinates out of bounds: " + str((rgx, rgy)) + " Bounds: " +
                                      str((self.rg_xlim, self.rg_ylim)))
                    if visibility.get(rgx+rgy*self.rg_xlim, 1) != 0:
                        noise_var[jj] /= visibility.get(rgx+rgy*self.rg_xlim, 1)**2
                        to_keep_idxs.append(jj)
                    # else:
                        # noise_var[jj] /= 0.001 # zero visibility -> very low visibility -> very high noise
            x = x[to_keep_idxs]
            non_zero_idx = [non_zero_idx[ix] for ix in to_keep_idxs]
            noise_var = [noise_var[ix] for ix in to_keep_idxs]
        # print("l,h is: ", (l,h), "x shape at this time is: ", x.shape)
        # print("Corresponding to this the pos in gridspace is: ", h*self.n1+l)
        # print("Corresponding to this the pos in robogridspace is: ", h*self.rg_xlim+l)
        # self.robogrid[h,l]["viewshed"]
        # print(np.array(non_zero_idx), np.array(noise_var), non_zero_2idx)

        return x, np.array(non_zero_idx), np.array(noise_var)
