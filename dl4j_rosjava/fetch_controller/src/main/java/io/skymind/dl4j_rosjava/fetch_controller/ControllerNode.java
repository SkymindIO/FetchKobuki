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
import org.ros.concurrent.WallTimeRate;
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

    private final DecimalFormat df = new DecimalFormat("0.00");

    double linearVelocity;
    double angularVelocity;
    float[][] laserRanges;
    double[] laserAngles;

    double kobukiDistance, kobukiAngle, wallDistance;

    double[] middlePose, fetchPose, kobukiPose;
    double[] lastFetchPose, lastKobukiPose;

    void setLinearVelocity(double linearVelocity) {
        this.linearVelocity = linearVelocity;
    }
    void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
    }
    float[][] getLaserRanges() {
        return laserRanges;
    }
    double[] getLaserAngles() {
        return laserAngles;
    }

    double getKobukiDistance() {
        return kobukiDistance;
    }
    double getKobukiAngle() {
        return kobukiAngle;
    }
    double getWallDistance() {
        return wallDistance;
    }

    double[] getMiddlePose() {
        return middlePose;
    }
    void setFetchPose(double... pose) {
        fetchPose = pose;
    }
    void setKobukiPose(double... pose) {
        kobukiPose = pose;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("dl4j_rosjava/fetch_controller");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        final Log log = connectedNode.getLog();
        Subscriber<sensor_msgs.LaserScan> subscriber = connectedNode.newSubscriber("base_scan", sensor_msgs.LaserScan._TYPE);
        subscriber.addMessageListener(new MessageListener<sensor_msgs.LaserScan>() {
            long lastTime = System.currentTimeMillis();

            @Override
            public void onNewMessage(sensor_msgs.LaserScan message) {
                // we get no time stats from this message, but we know Fetch is publishing at 15 Hz
                long time = System.currentTimeMillis();
                log.trace("scan times: " + message.getTimeIncrement() + " " + message.getScanTime() + " " + (time - lastTime));
                lastTime = time;

                laserAngles = new double[] { message.getAngleMin(), message.getAngleMax() };
                laserRanges = new float[][] { message.getRanges(),
                        laserRanges != null && laserRanges[0] != null ? laserRanges[0] : null };

                String ranges = "[";
                for (float range : message.getRanges()) {
                    if (ranges.length() > 1) {
                        ranges += ", ";
                    }
                    ranges += df.format(range);
                }
                ranges += "]";
                log.trace("ranges: " + ranges);
            }
        });

        Subscriber<gazebo_msgs.ModelStates> subscriber2 = connectedNode.newSubscriber("gazebo/model_states", gazebo_msgs.ModelStates._TYPE);
        subscriber2.addMessageListener(new MessageListener<gazebo_msgs.ModelStates>() {
            org.ros.rosjava_geometry.Vector3 referenceAxis
                    = new org.ros.rosjava_geometry.Vector3(0.0, 1.0, 0.0);

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
                log.trace("names: " + names);

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
                log.trace("poses: " + poses);

                double x1 = fetchPose.getPosition().getX();
                double y1 = fetchPose.getPosition().getY();
                double qw = fetchPose.getOrientation().getW();
                double qx = fetchPose.getOrientation().getX();
                double qy = fetchPose.getOrientation().getY();
                double qz = fetchPose.getOrientation().getZ();
                double a1 = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)); // yaw
                double x2 = kobukiPose.getPosition().getX();
                double y2 = kobukiPose.getPosition().getY();
                lastFetchPose = new double[] { x1, y1 };
                lastKobukiPose = new double[] { x2, y2 };

                kobukiDistance = Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
                kobukiAngle = a1 - Math.atan2(y2 - y1, x2 - x1); // clockwise to match LaserScan
                if (kobukiAngle > Math.PI) {
                    kobukiAngle -= 2 * Math.PI;
                } else if (kobukiAngle < -Math.PI) {
                    kobukiAngle += 2 * Math.PI;
                }
                log.trace("kobuki distance: " + df.format(kobukiDistance) + " angle: " + df.format(kobukiAngle));

                count = 0;
                double[] wallDists = new double[4];
                middlePose = new double[2];
                for (Pose wallPose : wallPoses) {
                    Point p = wallPose.getPosition();
                    middlePose[0] += p.getX();
                    middlePose[1] += p.getY();
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
                middlePose[0] /= 4;
                middlePose[1] /= 4;
                log.trace("wallDists: [" + df.format(wallDists[0]) + ", " + df.format(wallDists[1])
                                  + ", " + df.format(wallDists[2]) + ", " + df.format(wallDists[3]) + "]");
                wallDistance = Math.min(wallDists[0], Math.min(wallDists[1], Math.min(wallDists[2], wallDists[3])));

            }
        });

        final Publisher<geometry_msgs.Twist> publisher =
                connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
        // This CancellableLoop will be canceled automatically when the node shuts
        // down.
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            WallTimeRate rate = new WallTimeRate(10);
            private int sequenceNumber;

            @Override
            protected void setup() {
                sequenceNumber = 0;
            }

            @Override
            protected void loop() throws InterruptedException {
                geometry_msgs.Twist message = publisher.newMessage();
                geometry_msgs.Vector3 linear = message.getLinear();
                geometry_msgs.Vector3 angular = message.getAngular();
//                linear.setX(10.0 / sequenceNumber);
//                angular.setZ(10.0 / sequenceNumber);
                linear.setX(linearVelocity);
                angular.setZ(angularVelocity);
                log.trace("velocity linear: " + message.getLinear().getX() + " angular:" + message.getAngular().getZ());
                publisher.publish(message);
                sequenceNumber++;
                rate.sleep();
            }
        });

        final Publisher<gazebo_msgs.ModelState> publisher2 =
                connectedNode.newPublisher("gazebo/set_model_state", gazebo_msgs.ModelState._TYPE);
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            WallTimeRate rate = new WallTimeRate(10);
            org.ros.rosjava_geometry.Vector3 referenceAxis
                    = new org.ros.rosjava_geometry.Vector3(0.0, 0.0, 1.0);

            @Override
            protected void loop() throws InterruptedException {
                if (fetchPose != null) {
                    gazebo_msgs.ModelState message = publisher2.newMessage();
                    message.setModelName("fetch");
                    Pose pose = message.getPose();
                    Point p = pose.getPosition();
                    Quaternion q = pose.getOrientation();
                    p.setX(fetchPose[0]);
                    p.setY(fetchPose[1]);
                    org.ros.rosjava_geometry.Quaternion.fromAxisAngle(referenceAxis, fetchPose[2]).toQuaternionMessage(q);
                    publisher2.publish(message);
                    fetchPose = null;
                } else if (kobukiPose != null) {
                    gazebo_msgs.ModelState message = publisher2.newMessage();
                    message.setModelName("kobuki");
                    Pose pose = message.getPose();
                    Point p = pose.getPosition();
                    Quaternion q = pose.getOrientation();
                    p.setX(kobukiPose[0]);
                    p.setY(kobukiPose[1]);
                    org.ros.rosjava_geometry.Quaternion.fromAxisAngle(referenceAxis, kobukiPose[2]).toQuaternionMessage(q);
                    publisher2.publish(message);
                    kobukiPose = null;
                }
                rate.sleep();
            }
        });
    }
}
