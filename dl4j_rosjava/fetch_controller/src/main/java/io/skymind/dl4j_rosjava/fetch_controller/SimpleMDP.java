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

import java.text.DecimalFormat;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.deeplearning4j.gym.StepReply;
import org.deeplearning4j.rl4j.mdp.MDP;
import org.deeplearning4j.rl4j.space.ArrayObservationSpace;
import org.deeplearning4j.rl4j.space.DiscreteSpace;
import org.deeplearning4j.rl4j.space.Encodable;
import org.deeplearning4j.rl4j.space.ObservationSpace;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

/**
 *
 * @author saudet
 */
public class SimpleMDP implements MDP<SimpleMDP.Observation, Integer, DiscreteSpace> {

    public static class Observation implements Encodable {
        protected final int NUM_RANGES = 15;

        protected double[] array;

        public Observation(double linearVelocity, double angularVeocity, float[][] ranges) {
            array = new double[2 + 2 * NUM_RANGES];
            array[0] = linearVelocity;
            array[1] = angularVeocity;
            for (int i = 0; i < NUM_RANGES; i++) {
                // try to map the ranges symmetrically
                int j = (int)Math.round((i + (double)i / (NUM_RANGES - 1)) * (ranges[0].length - 1) / NUM_RANGES);
                array[2 + i] = ranges[0][j];
                array[2 + i + NUM_RANGES] = ranges[0][j] - ranges[1][j];
            }
        }

        @Override
        public double[] toArray() {
            return array;
        }
    }

    protected final double[] LINEAR_VELOCITIES = {0.0, 1.0, /*-1.0*/}; // m/s
    protected final double[] ANGULAR_VELOCITIES = {0.0, Math.PI, -Math.PI}; // rad/s
    protected final double MIDDLE_MAX_DISTANCE = 1; // m
    protected final double KOBUKI_MIN_DISTANCE = 0.5; // m
    protected final double WALL_MIN_DISTANCE = 0.5; // m
    protected final long LATENCY = 500; // ms
    protected final int MAX_STEPS = 100;

    protected ControllerNode controllerNode;
    protected NodeConfiguration nodeConfiguration;
    protected NodeMainExecutor nodeMainExecutor;
    protected List<int[]> actions;
    protected DiscreteSpace discreteSpace;
    protected ObservationSpace<Observation> observationSpace;
    protected boolean done;
    protected long randomSeed;
    protected Random random;
    protected int step;
    protected double lastKobukiDistance;

    private final Log log = LogFactory.getLog(SimpleMDP.class);
    private final DecimalFormat df = new DecimalFormat("0.00");

    public SimpleMDP(long seed) {
        controllerNode = new ControllerNode();
        nodeConfiguration = NodeConfiguration.newPublic("0.0.0.0");
        nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(controllerNode, nodeConfiguration);

        float[][] ranges = controllerNode.getLaserRanges();
        while (ranges == null || ranges[1] == null) {
            try {
                Thread.sleep(LATENCY);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            ranges = controllerNode.getLaserRanges();
        }
        discreteSpace = new DiscreteSpace(LINEAR_VELOCITIES.length * ANGULAR_VELOCITIES.length);
        Observation o = new Observation(0, 0, ranges);
        observationSpace = new ArrayObservationSpace<Observation>(new int[] {o.toArray().length});
        done = false;
        randomSeed = seed;
        random = new Random(seed);
        step = 0;
    }

    @Override
    public ObservationSpace<Observation> getObservationSpace() {
        return observationSpace;
    }

    @Override
    public DiscreteSpace getActionSpace() {
        return discreteSpace;
    }

    protected double[] randomPose() {
        double[] p = controllerNode.getMiddlePose();
        return new double[] {p[0] + 2 * MIDDLE_MAX_DISTANCE * (random.nextDouble() - 0.5),
                             p[1] + 2 * MIDDLE_MAX_DISTANCE * (random.nextDouble() - 0.5),
                           2 * Math.PI * random.nextDouble()};
    }

    @Override
    public Observation reset() {
        Observation o;
        do {
            double[] fetchPose = randomPose();
            double[] kobukiPose = randomPose();
            double dx = fetchPose[0] - kobukiPose[0];
            double dy = fetchPose[1] - kobukiPose[1];
            while (dx * dx + dy * dy <= KOBUKI_MIN_DISTANCE * KOBUKI_MIN_DISTANCE) {
                fetchPose = randomPose();
                kobukiPose = randomPose();
                dx = fetchPose[0] - kobukiPose[0];
                dy = fetchPose[1] - kobukiPose[1];
            }
            controllerNode.setFetchPose(fetchPose);
            controllerNode.setKobukiPose(kobukiPose);
            done = false;
            step = 0;
            o = step(0).getObservation();
        } while (done);

        return o;
    }

    @Override
    public void close() {
        nodeMainExecutor.shutdown();
        nodeMainExecutor = null;
    }

    @Override
    public StepReply<Observation> step(Integer a) {
        if (nodeMainExecutor == null) {
            return null;
        }
        double linearVelocity = LINEAR_VELOCITIES[a % LINEAR_VELOCITIES.length];
        double angularVelocity = ANGULAR_VELOCITIES[a / LINEAR_VELOCITIES.length];
        controllerNode.setLinearVelocity(linearVelocity);
        controllerNode.setAngularVelocity(angularVelocity);

        try {
            Thread.sleep(LATENCY);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        float[][] ranges = controllerNode.getLaserRanges();
        double kobukiDistance = controllerNode.getKobukiDistance();
        double wallDistance = controllerNode.getWallDistance();
        double reward = kobukiDistance < lastKobukiDistance ? 0.01 : 0;
        lastKobukiDistance = kobukiDistance;
        if (kobukiDistance <= KOBUKI_MIN_DISTANCE) {
            reward = 1;
            done = true;
        } else if (wallDistance <= WALL_MIN_DISTANCE) {
            reward = -1;
            done = true;
        } else if (step >= MAX_STEPS) {
            done = true;
        }
        log.info("step: " + step + " action: " + a + " (" + df.format(linearVelocity)+ ", " + df.format(angularVelocity) + ")"
                + " kobukiDistance: " + df.format(kobukiDistance) + " wallDistance: " + df.format(wallDistance)
                + " reward: " + reward + " done: " + done);
        step++;
        return new StepReply(new Observation(linearVelocity, angularVelocity, ranges), reward, done, null);
    }

    @Override
    public boolean isDone() {
        return done;
    }

    @Override
    public MDP<Observation, Integer, DiscreteSpace> newInstance() {
        // shut down this node to avoid interference with the single simulator
        nodeMainExecutor.shutdown();
        nodeMainExecutor = null;
        return new SimpleMDP(randomSeed);
    }

}
