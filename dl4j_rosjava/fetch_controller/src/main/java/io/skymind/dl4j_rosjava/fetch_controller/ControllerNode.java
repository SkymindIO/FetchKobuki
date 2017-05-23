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

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;
import java.text.DecimalFormat;
import java.util.Arrays;
import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

/**
 *
 * A simple {@link Subscriber} and {@link Publisher} {@link NodeMain} to control a Fetch robot.
 *
 * @author saudet
 */
public class ControllerNode extends AbstractNodeMain {

    private static final DecimalFormat df = new DecimalFormat("0.00");

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("dl4j_rosjava/fetch_controller");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        final Log log = connectedNode.getLog();
        Subscriber<sensor_msgs.LaserScan> subscriber = connectedNode.newSubscriber("base_scan", sensor_msgs.LaserScan._TYPE);
        subscriber.addMessageListener(new MessageListener<sensor_msgs.LaserScan>() {
            @Override
            public void onNewMessage(sensor_msgs.LaserScan message) {
                String ranges = "[";
                for (float range : message.getRanges()) {
                    if (ranges.length() > 1) {
                        ranges += ", ";
                    }
                    ranges += df.format(range);
                }
                ranges += "]";
                log.info("ranges: " + ranges);
            }
        });

        Subscriber<gazebo_msgs.ModelStates> subscriber2 = connectedNode.newSubscriber("gazebo/model_states", gazebo_msgs.ModelStates._TYPE);
        subscriber2.addMessageListener(new MessageListener<gazebo_msgs.ModelStates>() {
            @Override
            public void onNewMessage(gazebo_msgs.ModelStates message) {
                int count = 0, wallCount = 0;
                int fetchIndex = -1, kobukiIndex = -1;
                int[] wallIndices = new int[4];
                String names = "";
                for (String name : message.getName()) {
                    if (name.equals("fetch")) {
                        fetchIndex = count;
                    } else if (name.equals("kobuki")) {
                        kobukiIndex = count;
                    } else if (name.contains("wall")) {
                        wallIndices[wallCount] = count;
                        wallCount++;
                    }
                    names += name + ", ";
                    count++;
                }
                log.info("names: " + names);

                count = 0;
                Pose fetchPose = null, kobukiPose = null;
                Pose[] wallPoses = new Pose[4];
                String poses = "";
                for (Pose pose : message.getPose()) {
                    if (count == fetchIndex) {
                        fetchPose = pose;
                    } else if (count == kobukiIndex) {
                        kobukiPose = pose;
                    } else for (int i = 0; i < wallPoses.length; i++) {
                        if (count == wallIndices[i]) {
                            wallPoses[i] = pose;
                        }
                    }
                    Point p = pose.getPosition();
                    Quaternion q = pose.getOrientation();
                    if (poses.length() > 1) {
                        poses += ", ";
                    }
                    poses += "[" + df.format(p.getX()) + ", " + df.format(p.getY()) + ", " + df.format(p.getZ())
                          + "; " + df.format(q.getX()) + ", " + df.format(q.getY()) + ", " + df.format(q.getZ())
                          + ", " + df.format(q.getW()) + "]";
                    count++;
                }
                log.info("poses: " + poses);

                double x1 = fetchPose.getPosition().getX();
                double y1 = fetchPose.getPosition().getY();
                double x2 = kobukiPose.getPosition().getX();
                double y2 = kobukiPose.getPosition().getY();
                double kobukiDist = Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
                log.info("kobukiDist: " + df.format(kobukiDist));

                count = 0;
                double[] wallDists = new double[4];
                org.ros.rosjava_geometry.Vector3 referenceAxis
                        = new org.ros.rosjava_geometry.Vector3(0.0, 1.0, 0.0);
                for (Pose wallPose : wallPoses) {
                    Point p = wallPose.getPosition();
                    org.ros.rosjava_geometry.Vector3 p2
                            = new org.ros.rosjava_geometry.Vector3(p.getX(), p.getY(), p.getZ());
                    Quaternion q = wallPose.getOrientation();
                    org.ros.rosjava_geometry.Quaternion q2
                            = new org.ros.rosjava_geometry.Quaternion(q.getX(), q.getY(), q.getZ(), q.getW());
                    org.ros.rosjava_geometry.Vector3 n = q2.rotateAndScaleVector(referenceAxis);

                    // compute minimum distance from Fetch to the line of the wall
                    double a = n.getX();
                    double b = n.getY();
                    double c = n.getZ(); // should be pretty close to 0 for a wall
                    double d = -n.dotProduct(p2);
                    wallDists[count] = Math.abs(a * x1 + b * y1 + d);// / Math.sqrt(a * a + b * b);
                    count++;
                }
                log.info("wallDists: [" + df.format(wallDists[0]) + ", " + df.format(wallDists[1])
                                 + ", " + df.format(wallDists[2]) + ", " + df.format(wallDists[3]) + "]");

            }
        });

        final Publisher<geometry_msgs.Twist> publisher =
                connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
        // This CancellableLoop will be canceled automatically when the node shuts
        // down.
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            private int sequenceNumber;

            @Override
            protected void setup() {
                sequenceNumber = 0;
            }

            @Override
            protected void loop() throws InterruptedException {
                geometry_msgs.Twist msg = publisher.newMessage();
                geometry_msgs.Vector3 lin = msg.getLinear();
                lin.setX(10.0 / sequenceNumber);
                publisher.publish(msg);
                sequenceNumber++;
                Thread.sleep(1000);
            }
        });
    }
}
