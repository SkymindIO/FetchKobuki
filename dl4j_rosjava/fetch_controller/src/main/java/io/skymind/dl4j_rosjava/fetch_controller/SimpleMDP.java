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

import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
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
import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

/**
 *
 * @author saudet
 */
public class SimpleMDP implements MDP<SimpleMDP.Observation, Integer, DiscreteSpace> {

    public class Observation implements Encodable {
        public static final int NUM_RANGES = 5; // subsampled from 662 values
        public static final double MAX_RANGE = 5.0; // for normalization
        public static final double MIN_OVERLAP = 0.1; // of the target with the window

        protected double[] array;

        public Observation(float[] ranges, double kobukiRange, int kobukiIndex1, int kobukiIndex2) {
            // in the simulator, we know kobukiRange and kobukiIndex, but
            // in the real world, these values need to be estimated by some other means
            array = new double[2 * NUM_RANGES];
            kobukiIndex1 += ranges.length * MIN_OVERLAP / NUM_RANGES;
            kobukiIndex2 -= ranges.length * MIN_OVERLAP / NUM_RANGES;
            int n1 = kobukiIndex1 < 0 ? -1 : kobukiIndex1 * NUM_RANGES / ranges.length;
            int n2 = kobukiIndex2 < 0 ? -1 : kobukiIndex2 * NUM_RANGES / ranges.length;
            boolean facingTarget = n2 >= 0 && n1 < NUM_RANGES;

            double[] subranges = new double[ranges.length / NUM_RANGES + 1];
            boolean hasModel = kobukiDetector.getModel() != null;

            for (int i = 0; i < NUM_RANGES; i++) {
                boolean target = i >= n1 && i <= n2;
                double minRange = MAX_RANGE;
                int start = i * (subranges.length - 1);
                for (int j = 0; j < subranges.length - 1; j++) {
                    double r = ranges[start + j];
                    subranges[j] = r / MAX_RANGE;
                    if (minRange > r) {
                        minRange = r;
                    }
                }
                subranges[subranges.length - 1] = (double)i / NUM_RANGES;

                if (hasModel) {
                    target = kobukiDetector.predict(subranges);
                    kobukiRange = minRange; //assume target is closest object
                } else if (facingTarget && kobukiRangesWriter != null) {
                    String line = "";
                    for (int j = 0; j < subranges.length; j++) {
                        line += subranges[j] + ",";
                    }
                    line += (target ? 1 : 0) + "\n";

                    try {
                        kobukiRangesWriter.append(line);
                    } catch (IOException ex) {
                        log.error(ex);
                    }
                }

                // try to map the ranges symmetrically
                int j = (int)Math.round((i + (double)i / (NUM_RANGES - 1)) * (ranges.length - 1) / NUM_RANGES);
                array[2 * i    ] = target ? 1.0 : ranges[j] / MAX_RANGE;
                array[2 * i + 1] = target ? kobukiRange / MAX_RANGE : 1.0;
            }
        }

        @Override
        public double[] toArray() {
            return array;
        }
    }

    public static final double[][] FETCH_VELOCITIES = { {1.0, 0.0}, {0.1, 1.0}, {0.1, -1.0} }; // { m/s, rad/s }
    public static final double MIDDLE_MAX_DISTANCE = 1; // m
    public static final double KOBUKI_MIN_DISTANCE = 0.5; // m
    public static final double WALL_MIN_DISTANCE = 0.5; // m
    public static final long LATENCY = 500; // ms
    public static final int MAX_STEPS = 100;

    protected ControllerNode controllerNode;
    protected NodeConfiguration nodeConfiguration;
    protected NodeMainExecutor nodeMainExecutor;
    protected KobukiDetector kobukiDetector;
    protected Writer kobukiRangesWriter;
    protected Observation dummyObservation;
    protected DiscreteSpace discreteSpace;
    protected ObservationSpace<Observation> observationSpace;
    protected boolean done;
    protected long randomSeed;
    protected Random random;
    protected int step;
    protected double lastKobukiDistance;

    private final Log log = LogFactory.getLog(SimpleMDP.class);
    private final DecimalFormat df = new DecimalFormat("0.00");

    public SimpleMDP(KobukiDetector detector, long seed) {
        controllerNode = new ControllerNode();
        nodeConfiguration = new CommandLineLoader(Arrays.asList(ControllerNode.class.getName())).build();
        nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(controllerNode, nodeConfiguration);

        kobukiDetector = detector;
        if (kobukiDetector.getModel() == null) {
            // accumulate data only if we do not already have a model
            Path p = kobukiDetector.getDataPath().resolve(KobukiDetector.RANGES_FILENAME);
            try {
                kobukiRangesWriter = Files.newBufferedWriter(p, StandardCharsets.UTF_8);
            } catch (IOException ex) {
                log.error(ex);
            }
        }

        float[][] ranges = controllerNode.getLaserRanges();
        while (ranges == null || ranges[1] == null) {
            try {
                Thread.sleep(LATENCY);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
            ranges = controllerNode.getLaserRanges();
        }
        discreteSpace = new DiscreteSpace(FETCH_VELOCITIES.length);
        dummyObservation = new Observation(ranges[0], 0, 0, 0);
        observationSpace = new ArrayObservationSpace<Observation>(new int[] {dummyObservation.toArray().length});
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
        if (nodeMainExecutor == null) {
            return dummyObservation;
        }

        try {
            if (kobukiRangesWriter != null) {
                kobukiRangesWriter.flush();
            }
        } catch (IOException ex) {
            log.error(ex);
        }

        Observation o;
        controllerNode.setLinearVelocity(0);
        controllerNode.setAngularVelocity(0);

        try {
            Thread.sleep(LATENCY);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

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
            o = step(-1).getObservation();
        } while (done);

        return o;
    }

    @Override
    public void close() {
        if (nodeMainExecutor != null) {
            nodeMainExecutor.shutdown();
        }
        nodeMainExecutor = null;

        try {
            if (kobukiRangesWriter != null) {
                kobukiRangesWriter.close();
            }
        } catch (IOException ex) {
            log.error(ex);
        } finally {
            kobukiRangesWriter = null;
        }
    }

    @Override
    public StepReply<Observation> step(Integer a) {
        if (nodeMainExecutor == null) {
            return new StepReply(dummyObservation, 0, done, null);
        }
        double linearVelocity = a < 0 ? 0 : FETCH_VELOCITIES[a][0];
        double angularVelocity = a < 0 ? 0 : FETCH_VELOCITIES[a][1];
        controllerNode.setLinearVelocity(linearVelocity);
        controllerNode.setAngularVelocity(angularVelocity);

        try {
            Thread.sleep(LATENCY);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

        float[][] ranges = controllerNode.getLaserRanges();
        double[] angles = controllerNode.getLaserAngles();
        double kobukiDistance = controllerNode.getKobukiDistance();
        double kobukiAngle = controllerNode.getKobukiAngle();
        double wallDistance = controllerNode.getWallDistance();
        double reward = lastKobukiDistance - kobukiDistance;
        lastKobukiDistance = kobukiDistance;
        if (kobukiDistance <= KOBUKI_MIN_DISTANCE) {
            reward = 10;
            done = true;
        } else if (wallDistance <= WALL_MIN_DISTANCE) {
            reward = -10;
            done = true;
        } else if (step >= MAX_STEPS) {
            done = true;
        }

        // the laser is located ~28cm at the front of the base...
        double x = kobukiDistance * Math.cos(kobukiAngle) - 0.28;
        double y = kobukiDistance * Math.sin(kobukiAngle);
        double r = Math.sqrt(x * x + y * y);
        double angle = Math.atan2(y, x);

        // the kobuki is about 35 cm in diameter
        double dangle = Math.atan2(0.35 / 2, kobukiDistance);
        double angle1 = angle - dangle;
        double angle2 = angle + dangle;

        int n = (int)Math.round((angle - angles[0]) * ranges[0].length / (angles[1] - angles[0]));
        int n1 = (int)Math.round((angle1 - angles[0]) * ranges[0].length / (angles[1] - angles[0]));
        int n2 = (int)Math.round((angle2 - angles[0]) * ranges[0].length / (angles[1] - angles[0]));

        log.info("step: " + step + " action: " + a + " (" + df.format(linearVelocity)+ ", " + df.format(angularVelocity) + ")"
                + " kobukiDistance: " + df.format(kobukiDistance) + " wallDistance: " + df.format(wallDistance)
                + " reward: " + df.format(reward) + " done: " + done);
        step++;
        return new StepReply(new Observation(ranges[0], r, n1, n2), reward, done, null);
    }

    @Override
    public boolean isDone() {
        return done;
    }

    public int getStep() {
        return step;
    }

    @Override
    public MDP<Observation, Integer, DiscreteSpace> newInstance() {
        // shut down this node to avoid interference with the single simulator
        nodeMainExecutor.shutdown();
        nodeMainExecutor = null;
        return new SimpleMDP(kobukiDetector, randomSeed);
    }

}
