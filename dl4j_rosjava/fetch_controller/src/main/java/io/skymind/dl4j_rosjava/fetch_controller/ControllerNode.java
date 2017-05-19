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
                log.info("ranges: " + Arrays.toString(message.getRanges()));
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
