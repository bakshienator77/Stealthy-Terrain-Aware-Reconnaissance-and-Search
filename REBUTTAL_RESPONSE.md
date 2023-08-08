# Meta comment
Thank you for taking the time to review our paper. We shall briefly make our case for the relevance of this paper to CoRL and responding to specific questions by directly responding to the reviewer queries below.

> Submissions will be evaluated based on the significance and novelty of the results, either theoretical or empirical. Results will be judged on the degree to which they have been objectively established and/or their potential for scientific and technological impact, as well as their relevance to robotic learning. Submissions should focus on a core robotics problem and demonstrate the relevance of proposed models, algorithms, datasets, and benchmarks to robotics. Authors are encouraged to report real-robot experiments or provide convincing evidence that simulation experiments are transferable to real robots.

- We strongly encourage the reviewers to review our 3.5 min supplementary video that defines the problem, showcases our physical system, motivates the use of our realistic unity based simulation and depicts a full run of STAR in the simulator while explaining crucial steps in the algorithm.
- Our supplementary material also contains several engineering details such as average computation time, hardware used on the physical platforms, details on visibility map computation, etc. which the reviewers were curious about.
- The novelty in this paper, lies in the algorithmic achievements, while the engineering on the physical platform was a feat in itself, we wanted this paper to fill the gap between theoretical research on adversarial search and realistic robotic search scenarios. 
- Our algorithm is novel, runs on physical hardware, is inherently roboust to communication and hardware failures, and has been extensively ablated against varying team sizes, varying fidelity of communication, map type, and adversarial/non-adversarial targets. In all cases the STAR algorithm emerged superior on both sample complexity (efficiency of search decisions) and the stealth penalty (low-risk search)over existing baselines and state-of-the-art search methods [1-2]. 
- The problem of (adversarial) search has long been studied in theoretical settings and even today most applications of robotics to search missions are through teleoperation. Multi-agent multi-target active search is NP-complete in the general case and therefore no optimal solutions exist [3-4]. We have provided our anonymized code base (linked in supplimentary material) and will make this publicly available upon acceptance of the paper so that we may encourage greater adoption of and research in fully autonomous robotic search. The code base is written in ROS and a more extensive version of it is exactly what we deploy on our physical systems.
- On all points in the CoRL call for papers we believe we meet the threshold for relevance to the CoRL research community and we urge the reviewers to re-review this paper during the rebuttal period with fresh eyes.

[1] Y. Ma, R. Garnett, and J. Schneider. Active search for sparse signals with region sensing.
Proceedings of the AAAI Conference on Artificial Intelligence, 31(1), Feb. 2017

[2] N. A. Bakshi, T. Gupta, R. Ghods, and J. Schneider. Guts: Generalized uncertainty-aware
thompson sampling for multi-agent active search. In IEEE International Conference on
Robotics and Automation (ICRA), 2023

[3] Richard Borie, Craig Tovey, and Sven Koenig. Algo-
rithms and complexity results for pursuit-evasion prob-
lems. In Proceedings of the 21st International Joint
Conference on Artificial Intelligence, IJCAI’09, page
59–66, San Francisco, CA, USA, 2009. Morgan Kauf-
mann Publishers Inc.

[4] Hiroyuki Sato and Johannes O Royset. Path optimization
for the resource-constrained searcher. Nav. Res. Logist.,
pages NA–NA, 2010



# Reviewer One: WA

## Response

We thank the reviewer for carefully reading the paper and providing constructive feedback. Here we will address your open questions and will update the manuscript accordingly as indicated. We would encourage the reviewer to review our supplementary materials as many of the clarifications may be found there.

## Notational/Math Questions

> Notation is at times ambiguous. The sensing matrix for target and robots use the same symbol while the subscript has different meaning in the two settings. The subscript represents time in one instance, and a target index in the other. See equation (8) as an example. 

Thanks for pointing this out, it should be in superscript here. We have made the correction.
> -In Sec.3, can a component of vector v_t be zero? If it does, then it would lead to division by zero in equation (2).

The physical interpretation of an entry of v_t being zero is that the cell is completely obstructed/invisible to the robot (See Fig. 2-a). Such entries are simply omitted, i.e. no observation is added for them as they are not viewable. The undefined case does not occur.

> It is unclear if agents know the targets’ sensing matrices. In case they do not, then how the visibility objective in equation (8) is computed. The approach only samples the targets’ locations. Does it also sample the targets’ sensing matrices?

The robots assume target sensing matrix is depicted in Fig. 2-b. More details have been provided in Sec. 7.5 and Sec 7.6 in the supplementary material provided. 


## Engineering/Physical system questions

We wanted to present an algorithm-forward paper where the reader can think of the merits of the algorithm and apply it to their own multi-robot system. To this end the main paper was designed to focus on algorithmic details, experiments and performance, and the accompanying video and appendix covered details specific to the physical system. That being said, please find responses to your questions below.

> It is not clear how agent motion constraints are captured by the model. The agents’ motion model is not even described.

In the supplementary material a video of the real hardware platform and a full run in our realistic simulation environment are provided. The vehicles have an Ackerman drive and are modelled as such in the realistic simulator. The movement model is greatly simplified in the simple simulator. Planning is described in Sec 7.6 in the supplementary materials.

> Is the computational effort growing over time for the EM and action selection procedures?

Yes and no. The computational time is polynomial in the size of the search space (number of columns in X) or the number of observations (number of rows in X). The code picks whichever is lower. So initially the time does increase while the number of observations increases to match the search space but it is constant after that. Therefore we thinking of it as constant: bounded by the size of the search space. The next section will further clarify.

> What is the runtime performance of the algorithms per agent action? 

In the supplementary material we have provided a characteristic curve on real hardware to show computation time against size of the search space (directly related to the number of columns in X). But as explained in line 412-414 in Sec. 7.2 in the supplementary material this doesn’t impact real world search performance as the robot may simply start computing the next action when it is a minute or so away from it's current goal. 

> How does the communication impact performance as a function of update frequency (number of received packages from other agents per time unit)?

There is a fixed buffer to the incoming messages so it does not affect computation speed. This can be verified in the anonymized code link provided in Sec. 7.1 of the supplementary material.

> In the communication model, do agents swap raw observation data? Are data packages sufficiently small for the approach to be practical?

In line 108-110 in the main paper we define that, in our communication model, agents share their location with each other (high frequency) and positive observations whenever new one is made (low frequency) if communication is feasible. The network easily handles the traffic as there is no sharing of posteriors/planned paths.

> Why is the runtime budget 1 hour and 15 minutes?

4 sq. km is a large space and the robots have a max speed of 5 m/s. The budget needed to be longer enough that robots can actually search the space and short enough that this algorithm could be adopted in a real-world search and rescue or reconnaissance missions. This clarification was in the main text but was cut due to space restrictions. We shall add it back.

> It is unclear from the last sentence if zero-risk search is possible using the proposed method.

 In the current formulation with symmetric sensing, zero-risk or zero-penalty search is impossible, however in the asymmetric case, it can be possible. Please refer to Sec. 7.6 in the supplementary material for more details.

>The paper considers an important and interesting problem in robotics. It presents a practical approach in the form of a decentralized active exploration method. The paper requires clarification of several technical aspects and improve notation for clarity.

We are grateful to the reviewer for their comments. We hope we have addressed all outstanding points and welcome further feedback if any questions remain open.


# Reviewer Two: WR
## Experiments requested

## Major changes to manuscript

## Minor changes to manuscript

## Response
We thank the reviewer for carefully reading the paper and providing constructive feedback. Here we will address your open questions and will update the manuscript accordingly as indicated. We would strongly encourage the reviewer to review our supplementary materials, especially the 3.5 min accompanying video, as it will provide a fundamental understanding of the paper to a first-time reader.

> well I'm not an expert in the domain of multi-agent search. To me, it seems the ablation study should be conducted to see how much the prior helps the proposed problem. The reported ablation study over different map types, reliability of communications, number of search agents, and whether the targets are placed adversarially seems more like evaluating the proposed algorithm over other baselines in different settings. Please use appropriate words to describe the results.

[ADD DETAILS TO RESULTS AND EXPERIMENTS]

> Also, it is not clear to me what this paper's contribution is. If using the prior terrain information is novel, then has previous research used this information in other papers? If so, how is the proposed algorithm compared to those? If not, then the authors should state that this is the first time that terrain information is used for a stealthy target search algorithm.

- Please do review our comment at the top of this page addressing the contributions of this paper and its relevance to to CoRL community.
- The novel contribution of this paper is two-fold:
    - The problem formulation which uses realistic assumptions for large scale search operations, this includes robustness to intermittent communication, decentralized on board computation and realistic depth and terrain aware noise modelling.
    - The penalty term devised using the terrain prior in the multi objective optimisation. This stealth term informs search and evasion behaviour and is the key difference between STAR and GUTS [line 171]. Therefore in every ablation when STAR outperforms GUTS, it is evidence toward the vital contribution of the penalty term in improving search efficiency and reducing risk.


> Again, I'm not an expert in this domain. My recommendation is based on my limited understanding of this paper and the existing work.

We are grateful to the reviewer for their time and effort in understanding our paper. We hope that after reviewing the video, our meta comment at the top of this page and our above responses we have clarified all open questions in the reviewer's mind.

# Reviewer Three: SR
## Experiments requested

## Major changes to manuscript

We thank the reviewer for carefully reading the paper and providing constructive feedback. Here we will address your open questions and will update the manuscript accordingly as indicated. We would strongly encourage the reviewer to review our supplementary materials, especially the 3.5 min accompanying video, as it will provide an understanding of the paper.

Kindly do review our comment at the top of this page addressing the contributions of this paper and its relevance to to CoRL community.

> Is the method correct? There are confusing setups and missing assumptions. For example, in Sec 3 beta  is defined to be vector of 0s and 1s (line 112), but then it is modeled with a zero-mean gaussian in Sec 4.1 (line 152).

You have correctly pointed out that the first reference is the definition and the second is how it is being modelled/estimated on each robot. In the first mention of it in Sec 4.1 it has been referred to as $\beta$ instead of $\hat{\beta}$. We shall make the correction.

> As another example, the paper assumes decentralized control, but Eq (4) contains stacked measurements from all agents in Sec 4.1 (line 162) without stating when and how those agents exchange information. Both examples (and there are others) lead me to doubt the correctness of the method
- The fifth bullet in the problem formulation (line 108) mentions that robot locations and observations will be communicated to each other when possible. Line 162 only mentions ‘stacking all measurements from $(X_t, y_t)$ in $D_t^j$’. In the very next sentence Line 164 it is written: ‘Each robot estimates $p(\beta/D^j_t)$ on-board using its partial dataset $D^j_t$’. If the robot only has access to its own measurements, it will stack those, if it has received communications from other robots, it will stack those too. It is clearly a decentralized setup. 
- We also have experiments in Fig. 4 that ablate the performance of the system with and without the ability to communicate between agents. In either scenario STAR outperforms other methods and efficiency increases with increasing number of agents.

> How generalizable is the method? In Sec 2, the paper argues that "... it is not clear if these RL methods will generalize to different topographies". However, the paper does not give results on different terrains either.
- Fig. 4 vs Fig. 5 has different topographies but no change to the algorithm other than providing a different prior. This shows the generalizability of our algorithm.
- In the sentence just prior to the one quoted by the reviewer (Line 83) we also wrote that RL methods are extremely sample inefficient, that combined with overfitting on the terrain prior it was trained on makes RL impractical for time critical tasks like search and rescue where there is no time to train a model on the terrain prior before deploying the robots.
- More generally speaking, our algorithm is appliable to any environment that could be mathematically thought of as a 2-D grid, regardless of type of terrain, number of agents, ability to coordinate and adversarial nature of the targets. We have outlined the limitations in Sec. 6.

> How useful is this paper to robot learning researchers? In general, this paper can be presented as proposing a multi-agent exploration task in a grid world, and the stealthy search is one of the applications. A CoRL reader may expect more robot and real-world related components which are currently missing in the reviewer's opinion. For example, depending on the terrain, a vehicle may not be able to visit all the area and may turn over so that it loses mobility, will this affect agent's planning behavior? 
- We would strongly encourage the reviewer to watch our 3.5 min supplementary video. We show our physical test platforms, explain the STAR algorithm and show a run in our realistic simulator.
- A vehicle failing due experiencing any kind of hardware failure is inherently handled by the system. Due to the parallelized thompson sampling framework, there is to explicit subdivision of the space and therefore if any of the robots becoms unavailable, the team as a whole will still complete the task. This has been demonstrated in Bakshi et. al. on physical systems [refernce], and Ghods et. al. in simulation [reference].

> Lastly, I find the statement in the introduction "... Our motivation for only presenting simulation results in this paper is to ablate and assess the performance of the STAR algorithm ..." not convincing.

- The novelty in this paper lies in the algorithm not in the engineering of the physical test platforms, due to limitations in space we chose to focus on a well-ablated empirical study of the superiority of the algorithm in a realistic simulator.
- To the best of our knowledge there is no algorithm in current literature that can outperform state-of-the-art methods regardless of adversarial/non-adversarial nature of targets, with or without communication, subject to different map types, regardless of number of agents, in large scale environments and that is designed with realistic considerations for viewshed computations, detector noise and actually runs on physical systems.
- In the supplementary video we have provided we showcase our physical platform which is the basis for our ROS-Unity simulation experiments.
- In the appendix you will find statistics from runs on the physical systems at the testing site similar to the one recreated in Unity, as well as other hardware specific and implementation specific details. 
- In line 56, just prior to the statement quoted above by the reviewer we have encourage the reviewer to review the demo video showcasing the physical system.
- These are large-scale experiments on full size vehicles in outdoor environments, it is not possible to collect data in sufficient quantities to demonstrate statistically significant superiority of our algorithm against 4 other algorithms, varying the terrain, the number of agents, the availability of communications and the adversarial nature of the targets. Therefore we think our statement of presenting simulation results to 'ablate and assess the performance of the STAR' algorithm to be fair, whilst we are simultaneously providing a demo video and an appendix containing hardware related details and benchmarks.

## Minor changes to manuscript
> The paper uses a large number of notations and equations that are cluttered and redundant. E.g., Eq (3) and Eq (8) are the same thing.

They are not the same. Eqn 3 is how we evaluate model performance in our experiments and it assumes ground truth knowledge of target locations. Eqn (8) is the stealth penalty from a robots perspective where a posterior over the robot locations (Eqn. 7) informs the stealth penalty for decision making. Both these equations are necessary for understanding the algorithm and it's evaluation in the graphs in Fig.4 and Fig. 5.

> Figures need to be modified and rearranged. Most of the captions and legends are too small to be viewed clearly.
- We shall improve the ordering for clarity. The images are embedded in a lossless PDF and will be ditributed in an electronic form only, we encourage you to zoom in as necessary to see the figures.


We would like to reiterate, we are grateful to the reviewer for taking the time and providing us with constructive feedback to improve the manuscript and it's clarity.


# Reviewer Four: WR
## Experiments requested

## Major changes to manuscript

## Minor changes to manuscript
> (i) Although the number of targets is unknown a priori, it seems that the algorithm needs (i-a) a prior on the targets' location, and (i-b) knowledge of their sensing matrix. I would discuss in the paper how (i-a) and (i-b) can be obtained.
- No prior is needed, as stated in section 4.1, a zero mean gaussian prior is assumed for each possible grid location and sparsity is enforced using a conjugate inverse gamma prior. The inverse gamma prior is responsible for dynamically setting some variances of the gaussian priors in the grid to well-above zero. The exact number of non-zero entries is learned online based on observations, therefore our method is robust to not having an informative prior over target locations AND not having a prior on the number of targets either.
- Yes the sensing matrix is assumed to be an omnidirectional version of a robots own sensing capability (Fig. 2) and it is hence symmetric. In Sec. 7.4-7.6 of the appendix you will find more information on this. 
> Also, I would discuss these assumptions in the introduction and in the problem formulation to help guide the reader on what the problem and its limitations are.
- [Don't take GUTS constraints for granted]
- [TODO]

>(ii) To my understanding, the algorithm also needs a topological map that quantifies the risk for the robots to be visible from a target. Similarly to the previous comment, I would explain how such a map can be obtained when the targets are unknown a priori.
- This is defined based on the current posterior $\hat{\beta}$ in Sec. 4.2, Lines 183-186: "However, we don't know the ground truth locations of the targets, we only have the posterior $\hat{\beta}$. We use the folded normal distribution to determine a separate posterior mean $\hat{\mu}_{vis} \in \mathbb{R}^M$ for visibility risk that accounts for the mean and variance of the posterior $\hat{\beta}$ as defined in Eqn. 7 and the penalty function is defined in Eqn. 8"
- The intuition for this choice is explained in Line 190-194: "In Eqn. 8, $\hat{\mu}^i_{vis}$ behaves as a weighting scalar for each location $i$ in the map where a threat may be located. When the posterior variance for a location $i$ is close to zero then $\hat{\mu}^i_{vis}$ will tend to the posterior mean $\mu^i$, and when the variance is high but the mean is zero (as it is at the beginning of the run) the weighting factor will still be non-zero as it will be governed by the variance."
> I would define some used terms/notions:

> (i) I would elaborate on what "information leakage" means.

Information leakage from the paper’s perspective has been defined as the stealth penalty. More generally, it is when an attacker 

> (ii) In eq. (1), I would define what the clip operator does immediately after eq. (1).
- We have moved its explanation from line 128 closer to Eqn. 1.

> (iii) I would define what a "one-hot vector" is.

> (iv) I would define what "recall" means in Figure 6.


> I would elaborate on design choices, to help the reader understand the practicality and potential impact of the algorithm:

> (i) I would explain why the observation model is clipped to be between 0 and 1.
- Per the definition of $\beta$ (Line 112) observations must be in the range, the equation to which you are referring is Line 114, the relation is implicit. The additive noise can violate these bounds hence the clipping is necessary.

> (ii) I would discuss what is the intuition behind the reward in eq. (6).
- [TODO]
> (iii) I would discuss under what assumptions there are performance guarantees.
- 

>I would discuss in more detail the differences of this work from existing works, in particular [27, 28, 29]. In lines 87-89, the comparison is through the statement "However, these approaches focus on path planning but not on a competing search objective." I understand from this that the difference between [27, 28, 29] and this paper is that this paper uses a different objective function. I would present the comparison from a more qualitative perspective: Will the algorithms in [27, 28, 29] not perform well in the proposed setting in this paper, and why? In what scenarios they will not be able to be applied but the proposed algorithm will be?
- [27, 28, 29] attempt to solve path planning with adveraries. The goal is to avoid known targets but not to seek them. It is a single objective problem. 
- In Sec 7.6 in the supplementary material, we describe the off-the-shelf planning tool we use on the risk objective computed in Eqn. 8. This is our solution to the equivalent problem.
- We have amended lines 87-89 to now read as follows: "Terrain-aware path planning with adversarial targets is well-studied in the context of military operations [27, 28, 29] and in the context of stealth-based video games [30, 31]. However, these approaches focus on path planning but not on a competing search objective, that is, they assume that
the adversary locations are known and need to be avoided, or are unknown but need to be evaded if encountered en route to the goal."

> The paper proposes an excellent problem and a method that appears promising. In my opinion, the current version of the paper would benefit from discussing the practicality of the assumptions and from elaborating on the rigorous performance of the proposed algorithm given the design choices.

We are grateful to the excellent constructive feedback provided by the reviewer to improve the clarity of our paper. We hope that the changes proposed and clarifications provided above have addressed all open questions in the reviewer's mind.



TODO:
1. Correct Fig. 4-d ka y axis
2. Define T, J, C, M in the graphs
3. Don't take GUTS assumptions for granted. Assume reader does not know. 
    - What is the problem?
    - Why is it important?
    - Why is it hard?
    - what have others done?
    - what's missing?
    - what's our big idea/contribution?
    - How do we evaluate against SotA?
4. Quantify improvements of STAR over baselines?