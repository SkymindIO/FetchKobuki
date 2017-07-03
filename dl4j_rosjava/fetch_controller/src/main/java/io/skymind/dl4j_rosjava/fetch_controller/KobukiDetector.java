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
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.datavec.api.records.reader.RecordReader;
import org.datavec.api.records.reader.impl.csv.CSVRecordReader;
import org.datavec.api.split.FileSplit;
import org.deeplearning4j.datasets.datavec.RecordReaderDataSetIterator;
import org.deeplearning4j.eval.Evaluation;
import org.deeplearning4j.nn.api.OptimizationAlgorithm;
import org.deeplearning4j.nn.conf.MultiLayerConfiguration;
import org.deeplearning4j.nn.conf.NeuralNetConfiguration;
import org.deeplearning4j.nn.conf.Updater;
import org.deeplearning4j.nn.conf.layers.DenseLayer;
import org.deeplearning4j.nn.conf.layers.OutputLayer;
import org.deeplearning4j.nn.multilayer.MultiLayerNetwork;
import org.deeplearning4j.nn.weights.WeightInit;
import org.deeplearning4j.optimize.listeners.ScoreIterationListener;
import org.deeplearning4j.util.ModelSerializer;
import org.nd4j.linalg.activations.Activation;
import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.dataset.DataSet;
import org.nd4j.linalg.dataset.api.iterator.DataSetIterator;
import org.nd4j.linalg.factory.Nd4j;
import org.nd4j.linalg.lossfunctions.LossFunctions.LossFunction;

/**
 *
 * @author saudet
 */
public class KobukiDetector {
    private static final Log log = LogFactory.getLog(KobukiDetector.class);

    public static final String RANGES_FILENAME = "kobuki_ranges.csv";
    public static final String TRAIN_FILENAME = "kobuki_ranges_train.csv";
    public static final String TEST_FILENAME = "kobuki_ranges_test.csv";
    public static final String MODEL_FILENAME = "kobuki_detector.model";

    private Path dataPath;
    private File modelFile;
    private MultiLayerNetwork model;

    private String suffix = "";
    private int seed = 123;
    private double learningRate = 0.01;
    private double l2Regularization = 0.0001;
    private int batchSize = 100;
    private int nEpochs = 20;
    private int numLayers = 3;
    private int numHiddenNodes = 100;

    public Path getDataPath() {
        return dataPath;
    }

    public MultiLayerNetwork getModel() {
        return model;
    }

    public KobukiDetector(String dataDir) throws IOException {
        dataPath = Paths.get(dataDir);
        modelFile = new File(dataDir, MODEL_FILENAME + suffix);
        model = modelFile.exists() ? ModelSerializer.restoreMultiLayerNetwork(modelFile) : null;
    }

    public boolean predict(double[] ranges) {
        INDArray predicted = model.output(Nd4j.create(ranges), false);
        return Nd4j.argMax(predicted, 1).getDouble(0) != 0;
    }

    public void train() throws InterruptedException, IOException {
        //Split data into training and testing sets:
        List<String> allLines = Files.readAllLines(dataPath.resolve(RANGES_FILENAME), StandardCharsets.UTF_8);
        int rows = allLines.size();
        int columns = 1;
        for (char c : allLines.get(0).toCharArray()) {
            columns += c == ',' ? 1 : 0;
        }
        Collections.shuffle(allLines, new Random(seed));
        int splitRow = 80 * rows / 100;
        Files.write(dataPath.resolve(TRAIN_FILENAME), allLines.subList(0, splitRow), StandardCharsets.UTF_8);
        Files.write(dataPath.resolve(TEST_FILENAME), allLines.subList(splitRow, rows), StandardCharsets.UTF_8);

        int numInputs = columns - 1;
        int numOutputs = 2;

        //Load the training data:
        RecordReader rr = new CSVRecordReader();
        rr.initialize(new FileSplit(dataPath.resolve(TRAIN_FILENAME).toFile()));
        DataSetIterator trainIter = new RecordReaderDataSetIterator(rr, batchSize, -1, 2);

        //Load the test/evaluation data:
        RecordReader rrTest = new CSVRecordReader();
        rrTest.initialize(new FileSplit(dataPath.resolve(TEST_FILENAME).toFile()));
        DataSetIterator testIter = new RecordReaderDataSetIterator(rrTest, batchSize, -1, 2);

        NeuralNetConfiguration.ListBuilder listBuilder = new NeuralNetConfiguration.Builder()
                .seed(seed)
                .iterations(1)
                .optimizationAlgo(OptimizationAlgorithm.STOCHASTIC_GRADIENT_DESCENT)
                .learningRate(learningRate)
                .regularization(true).l2(l2Regularization)
                .updater(Updater.NESTEROVS).momentum(0.9)
                .list()
                .layer(0, new DenseLayer.Builder().nIn(numInputs).nOut(numHiddenNodes)
                        .weightInit(WeightInit.XAVIER)
                        .activation(Activation.RELU)
                        .build());

        for (int i = 1; i < numLayers; i++) {
            listBuilder.layer(i, new DenseLayer.Builder().nIn(numHiddenNodes).nOut(numHiddenNodes)
                    .weightInit(WeightInit.XAVIER)
                    .activation(Activation.RELU)
                    .build());
        }

        MultiLayerConfiguration conf = listBuilder
                .layer(numLayers, new OutputLayer.Builder(LossFunction.NEGATIVELOGLIKELIHOOD)
                        .weightInit(WeightInit.XAVIER)
                        .activation(Activation.SOFTMAX)
                        .nIn(numHiddenNodes).nOut(numOutputs).build())
                .pretrain(false).backprop(true).build();

        model = new MultiLayerNetwork(conf);
        model.init();
        model.setListeners(new ScoreIterationListener(100));  //Print score every 100 parameter updates

        for (int n = 0; n < nEpochs; n++) {
            model.fit(trainIter);

            log.info("Evaluating model...");
            Evaluation eval = new Evaluation(numOutputs);
            while (testIter.hasNext()){
                DataSet ds = testIter.next();
                INDArray predicted = model.output(ds.getFeatureMatrix(), false);
                eval.eval(ds.getLabels(), predicted);
            }
            testIter.reset();

            String stats = eval.stats();

            //Print the evaluation statistics
            log.info(stats);

            Files.write(dataPath.resolve("eval_stats_" + n + ".txt" + suffix), stats.getBytes());
        }

        File f = dataPath.resolve(MODEL_FILENAME + suffix).toFile();
        log.info("Saving " + f);
        ModelSerializer.writeModel(model, f, true);
    }

    public static void main(String[] args) throws Exception {
        if (args.length < 1) {
            System.err.println("Please specify the data directory.");
            System.exit(1);
        }

        for (double learningRate : new double[] {0.001, 0.005, 0.01, 0.015, 0.02}) {
            for (double l2Regularization : new double[] {1e-5, 1e-4, 1e-3}) {
                for (int numLayers : new int[] {1, 2, 3, 4}) {
                    for (int numHiddenNodes : new int[] {32, 64, 128, 256, 512}) {
                        KobukiDetector kb = new KobukiDetector(args[0]);
                        kb.suffix = "_" + learningRate + "_" + l2Regularization + "_" + numLayers + "_" + numHiddenNodes;
                        kb.learningRate = learningRate;
                        kb.l2Regularization = l2Regularization;
                        kb.numLayers = numLayers;
                        kb.numHiddenNodes = numHiddenNodes;
                        kb.train();
                    }
                }
            }
        }
    }
}
