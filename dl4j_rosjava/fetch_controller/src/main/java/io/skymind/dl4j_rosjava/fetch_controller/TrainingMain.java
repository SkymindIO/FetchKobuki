/*-
 *
 *  * Copyright 2017 Skymind,Inc.
 *  *
 *  *    Licensed under the Apache License, Version 2.0 (the "License");
 *  *    you may not use this file except in compliance with the License.
 *  *    You may obtain a copy of the License at
 *  *
 *  *        http://www.apache.org/licenses/LICENSE-2.0
 *  *
 *  *    Unless required by applicable law or agreed to in writing, software
 *  *    distributed under the License is distributed on an "AS IS" BASIS,
 *  *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  *    See the License for the specific language governing permissions and
 *  *    limitations under the License.
 *
 */

package io.skymind.dl4j_rosjava.fetch_controller;

import java.io.File;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.deeplearning4j.nn.multilayer.MultiLayerNetwork;
import org.deeplearning4j.rl4j.learning.async.a3c.discrete.A3CDiscrete;
import org.deeplearning4j.rl4j.learning.async.a3c.discrete.A3CDiscreteDense;
import org.deeplearning4j.rl4j.learning.async.nstep.discrete.AsyncNStepQLearningDiscrete;
import org.deeplearning4j.rl4j.learning.async.nstep.discrete.AsyncNStepQLearningDiscreteDense;
import org.deeplearning4j.rl4j.learning.sync.qlearning.QLearning;
import org.deeplearning4j.rl4j.learning.sync.qlearning.discrete.QLearningDiscreteDense;
import org.deeplearning4j.rl4j.network.ac.ActorCriticFactorySeparateStdDense;
import org.deeplearning4j.rl4j.network.dqn.DQN;
import org.deeplearning4j.rl4j.network.dqn.DQNFactoryStdDense;
import org.deeplearning4j.rl4j.policy.DQNPolicy;
import org.deeplearning4j.rl4j.util.DataManager;
import org.deeplearning4j.util.ModelSerializer;

/**
 *
 * @author saudet
 */
public class TrainingMain {
    private static final Log log = LogFactory.getLog(PlayingMain.class);

    public static final String POLICY_FILENAME = "fetch_policy.model";

    public static QLearning.QLConfiguration QL_CONF =
            new QLearning.QLConfiguration(
                    123,    //Random seed
                    100,    //Max step By epoch
                    100000, //Max step
                    100000, //Max size of experience replay
                    32,     //size of batches
                    100,    //target update (hard)
                    10,     //num step noop warmup
                    0.1,    //reward scaling
                    0.9,    //gamma
                    10.0,   //td-error clipping
                    0.1f,   //min epsilon
                    1000,   //num step for eps greedy anneal
                    true    //double DQN
            );

    public static DQNFactoryStdDense.Configuration QL_NET_CONF =
            DQNFactoryStdDense.Configuration.builder()
                    .learningRate(1e-3)
                    .l2(1e-4)
                    .numHiddenNodes(16)
                    .numLayer(3).build();

    public static AsyncNStepQLearningDiscrete.AsyncNStepQLConfiguration NSTEP_CONF =
            new AsyncNStepQLearningDiscrete.AsyncNStepQLConfiguration(
                    123,     //Random seed
                    100,     //Max step By epoch
                    100000,  //Max step
                    1,       //Number of threads
                    5,       //t_max
                    100,     //target update (hard)
                    10,      //num step noop warmup
                    0.1,     //reward scaling
                    0.9,     //gamma
                    10.0,    //td-error clipping
                    0.1f,    //min epsilon
                    1000     //num step for eps greedy anneal
            );

    public static DQNFactoryStdDense.Configuration NSTEP_NET_CONF =
            DQNFactoryStdDense.Configuration.builder()
                    .learningRate(1e-3)
                    .l2(1e-4)
                    .numHiddenNodes(16)
                    .numLayer(3).build();

    private static A3CDiscrete.A3CConfiguration A3C_CONF =
            new A3CDiscrete.A3CConfiguration(
                    123,            //Random seed
                    100,            //Max step By epoch
                    100000,         //Max step
                    1,              //Number of threads
                    5,              //t_max
                    10,             //num step noop warmup
                    0.1,            //reward scaling
                    0.9,            //gamma
                    10.0            //td-error clipping
            );

    private static final ActorCriticFactorySeparateStdDense.Configuration A3C_NET_CONF =
            ActorCriticFactorySeparateStdDense.Configuration.builder()
                    .learningRate(1e-3)
                    .l2(1e-4)
                    .numHiddenNodes(16)
                    .numLayer(3).build();

    public static void main(String[] args) throws Exception {
        if (args.length < 1) {
            System.err.println("Please specify the data directory.");
            System.exit(1);
        }
        String dataDir = args[0];

        //get a detector, if a model is available
        KobukiDetector kobukiDetector = new KobukiDetector(dataDir);

        //record the training data in rl4j-data in a new folder
        DataManager manager = new DataManager(true);

        //define the mdp from rosjava
        SimpleMDP mdp = new SimpleMDP(kobukiDetector, 0, 1235);

        File policyFile = new File(dataDir, POLICY_FILENAME);
        MultiLayerNetwork mln = policyFile.exists() ? ModelSerializer.restoreMultiLayerNetwork(policyFile) : null;

        //define the training
        QLearningDiscreteDense<SimpleMDP.Observation> dql = mln != null
                ? new QLearningDiscreteDense<>(mdp, new DQN(mln), QL_CONF, manager)
                : new QLearningDiscreteDense<>(mdp, QL_NET_CONF, QL_CONF, manager);
//        AsyncNStepQLearningDiscreteDense<SimpleMDP.Observation> dql = mln != null
//                ? new AsyncNStepQLearningDiscreteDense<>(mdp, new DQN(mln), NSTEP_CONF, manager)
//                : new AsyncNStepQLearningDiscreteDense<>(mdp, NSTEP_NET_CONF, NSTEP_CONF, manager);
//        A3CDiscreteDense<SimpleMDP.Observation> dql = new A3CDiscreteDense<>(mdp, A3C_NET_CONF, A3C_CONF, manager);

        //start the training
        dql.train();

        //get the final policy
        DQNPolicy<SimpleMDP.Observation> pol = dql.getPolicy();

        //serialize and save
        log.info("Saving " + policyFile);
        pol.save(policyFile.getPath());

        //close the mdp (http connection)
        mdp.close();

        //also train the detector if it doesn't have a model already
        if (kobukiDetector.getModel() == null) {
            kobukiDetector.train();
        }
    }
}
