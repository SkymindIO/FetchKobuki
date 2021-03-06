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
import org.deeplearning4j.rl4j.network.dqn.DQN;
import org.deeplearning4j.rl4j.policy.DQNPolicy;
import org.deeplearning4j.rl4j.policy.Policy;
import org.deeplearning4j.util.ModelSerializer;

/**
 *
 * @author saudet
 */
public class PlayingMain {
    private static final Log log = LogFactory.getLog(PlayingMain.class);

    public static void main(String[] args) throws Exception {
        if (args.length < 1) {
            System.err.println("Please specify the data directory.");
            System.exit(1);
        }
        String dataDir = args[0];

        //get a detector, if a model is available
        KobukiDetector kobukiDetector = new KobukiDetector(dataDir);

        //define the mdp from rosjava
        SimpleMDP mdp = new SimpleMDP(kobukiDetector, 10, 1235);

        //load the previous agent
        MultiLayerNetwork mln = ModelSerializer.restoreMultiLayerNetwork(new File(dataDir, TrainingMain.POLICY_FILENAME));
        Policy<SimpleMDP.Observation, Integer> pol = new DQNPolicy<SimpleMDP.Observation>(new DQN(mln));

        //evaluate the agent
        int episodes = 100;
        int failures = 0;
        int steps = 0;
        int terminations = 0;
        double rewards = 0;
        for (int i = 0; i < episodes; i++) {
            double reward = pol.play(mdp);
            if (reward < 0) {
                failures++;
            }
            if (mdp.getStep() >= SimpleMDP.MAX_STEPS) {
                terminations++;
            }
            steps += mdp.getStep();
            rewards += reward;
            log.info("Steps: " + mdp.getStep() + " Reward: " + reward);
        }

        log.info("Failures: " + failures + " Terminations: " + terminations
                + " Averate steps: " + (float)steps / episodes + " Average reward: " + (float)rewards / episodes);

        //close the mdp (http connection)
        mdp.close();
    }
}
