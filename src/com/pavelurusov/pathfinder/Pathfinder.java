package com.pavelurusov.pathfinder;

import com.pavelurusov.squaregrid.SquareGrid;
import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.stage.FileChooser;
import javafx.stage.Stage;

import java.io.*;
import java.util.*;

/** @author Pavel Urusov, me@pavelurusov.com
* This is my simple visualisation tool for A* and Dijkstra pathfinding algorithms.
*/

public class Pathfinder extends Application {

    // the main display ;-)
    private SquareGrid board;

    // necessary UI elements
    private BorderPane root;
    private Label originLabel;
    private Label destinationLabel;
    private Label statLabel;
    private Label pathLabel;
    private Button resetButton;
    private Button startButton;
    private Button saveButton;
    private Button loadButton;
    private Button odSwitchButton;
    private RadioButton algoDijkstra;
    private RadioButton algoAstar;
    private RadioButton speedFaster;
    private RadioButton speedSlower;
    private RadioButton quadraticRButton;
    private RadioButton manhattanRButton;
    private RadioButton euclideanRButton;
    private RadioButton diagonalRButton;
    private CheckBox diagonalsCheckBox;
    private CheckBox pathCorrectionCheckBox;

    // grid dimensions
    private final int columns = 75;
    private final int rows = 50;

    // step counter
    private int stepCount = 0;

    // algorithm parameters
    private enum Algorithm { Dijkstra,
                        Astar }

    private enum Heuristic {
                            Manhattan,
                            Quadratic,
                            Euclidean,
                            Diagonal }

    // default values
    private Heuristic heuristic = Heuristic.Euclidean;
    private Algorithm algorithm = Algorithm.Astar;
    private boolean allowDiagonals = true;
    private final double hWeight = 1d; // reserved for future use ;-)

    private boolean pathCorrection = false;

    private boolean isRunning = false;

    // main loop
    private AnimationTimer timer;
    private double interval = 1e8; // default interval = 1/20th of a second

    // origin, destination and current nodes
    private Node origin = null;
    private Node destination = null;
    private Node current = null;

    Set<Node> settledNodes;
    Set<Node> blockedNodes;
    Set<Node> unsettledNodes;

    @Override
    public void start(Stage stage) throws Exception{

        // initialize necessary fields
        unsettledNodes = new HashSet<>();
        settledNodes = new HashSet<>();
        blockedNodes = new HashSet<>();

        // set up the animation loop
        timer = new AnimationTimer() {
            long lastFrameTime;
            @Override
            public void handle(long time) {
                if (time - lastFrameTime >= interval) {
                    tick();
                    lastFrameTime = time;
                }
            }
        };

        // set up the UI and the scene graph
        root = setUI();
        stage.setTitle("jPathfinder");
        stage.setResizable(false);
        stage.setScene(new Scene(root));
        stage.show();
    }

    private void tick() {
        // paint the current state of the map
        visualize();
        // refresh the screen
        board.redraw();
        // update stats
        statLabel.setText("Settled nodes: " + settledNodes.size() +
                ", unsettled nodes: " + unsettledNodes.size() +
                ", total steps: " + stepCount);
        // move to next node
        current = findNext(current);

        // it's done
        if(!isRunning) {
            doStop();
        }
    }

    // finds the next node to move to
    public Node findNext(Node previous) {
        for (int dX = -1; dX <= 1; dX++) {
            for (int dY = -1; dY <= 1; dY++) {
                if (dX == 0 && dY == 0) { // this is the current node itself
                    continue;
                }
                int nextX = previous.getX() + dX;
                int nextY = previous.getY() + dY;

//              calculates costs for the node at nextX, nextY and returns it
                Node tempNode = processSuccessor(nextX, nextY, previous);
//              if it's not null, add it to unsettled
                if (tempNode != null) {
                    unsettledNodes.add(tempNode);
                    stepCount++; // opening a node counts as a step
                }
            }
        }
        // set the current node to the lowest cost unsettled node
        Node next = lowestCostNode();
        stepCount++; // moving into an unsettled node counts as a step

//      On-the-fly path correction:
//      Check each neighbouring unsettled node one by one and if its G-cost is higher than
//      the cost of the current node + distance to the unsettled node, update the costs
//      and set the current node to be the predecessor of the unsettled node.
//      Improves path quality but adds a lot of additional steps.
        if(algorithm == Algorithm.Astar && next != null) {
            if (pathCorrection) {
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        if (dx == 0 && dy == 0) {
                            continue;
                        }
                        if (!allowDiagonals && dx != 0 && dy != 0) {
                            continue;
                        }
                        stepCount++;
                        int nextX = next.getX() + dx;
                        int nextY = next.getY() + dy;
                        // don't cut corners and jump through diagonal fences
                        if((dx != 0) && (dy != 0)) {
                            if (blockedNodes.contains(new Node(nextX,next.getY()))
                                    || blockedNodes.contains(new Node(next.getX(),nextY))) {
                                continue;
                            }
                        }
                        if (unsettledNodes.contains(new Node(nextX, nextY))) {
                            double nextG = next.getGCost() + Math.sqrt(square(dx) + square(dy));
                            Node unsettledNode = getUnsettledNode(new Node(nextX, nextY));
                            if (nextG < unsettledNode.getGCost()) {
                                unsettledNode.setPredecessor(next);
                                unsettledNode.setGCost(nextG);
                                unsettledNode.setFCost(nextG + unsettledNode.getHCost());
                            }
                        }
                    }
                }
            }
        }

        if (next == null) {
            // no path
            isRunning = false;
            pathLabel.setText("No path found!");
            current = null;
            visualize();
            board.redraw();
        } else if(next.equals(destination)) {
            // found the path
            destination.setPredecessor(next.getPredecessor());
            isRunning = false;
            current = null;
            visualize();
            drawPath();
            board.redraw();
        }

//      remove the current node from unsettled
        unsettledNodes.remove(next);

//      add the current node to settled
        settledNodes.add(next);

//      return the current node
        return next;
    }

//    calculate costs for node at x,y and return the node
    private Node processSuccessor(int x, int y, Node currentNode) {
        // check if x,y are outside of map boundaries
        if (x < 0 || y < 0 || x >= columns || y >= rows) {
            return null;
        }
//      prevent the algorithm from jumping across diagonal borders,
//      this also disables cutting corners
        if((x - currentNode.getX() != 0) && (y - currentNode.getY() != 0)) {
            if (blockedNodes.contains(new Node(x,currentNode.getY()))
                    || blockedNodes.contains(new Node(currentNode.getX(),y))) {
                return null;
            }
        }

//      if diagonal movements are not allowed,
//      reject nodes where both x and y is different from the currentNode
        if(!allowDiagonals) {
            if((x - currentNode.getX() != 0) && (y - currentNode.getY() != 0)) {
                return null;
            }
        }

        Node node = new Node(x, y);

        // if the node has been processed already or is in the list of blocked nodes, return null
        if (settledNodes.contains(node) || blockedNodes.contains(node) || unsettledNodes.contains(node)) {
            return null;
        }
        node.setPredecessor(currentNode);
        // calculate G cost
        int dX = Math.abs(x - currentNode.getX());
        int dY = Math.abs(y - currentNode.getY());
        double gCost = currentNode.getGCost() + Math.sqrt(square(dX) + square(dY));
        node.setGCost(gCost);

        if (algorithm == Algorithm.Dijkstra) { // Dijkstra doesn't include the heuristic element
            // F cost = G cost
            node.setFCost(gCost);
        } else { // calculate H cost
            int distanceToDestX = Math.abs(x - destination.getX());
            int distanceToDestY = Math.abs(y - destination.getY());
            double hCost = 0;
            switch(heuristic) {
                case Manhattan:
                    hCost = distanceToDestX + distanceToDestY;
                    break;
                case Euclidean:
                    hCost = Math.sqrt(square(distanceToDestX) + square(distanceToDestY));
                    break;
                case Quadratic:
                    hCost = square(distanceToDestX) + square(distanceToDestY);
                    break;
                case Diagonal:
                    hCost = Math.max(distanceToDestX, distanceToDestY);
            }
            node.setHCost(hWeight * hCost);
            // F cost = G cost + H cost
            node.setFCost(gCost + hCost);
        }
        return node;
    }

    // returns the lowest cost node from the pool of unsettled nodes
    private Node lowestCostNode() {
        if(unsettledNodes.size() != 0) {
            return Collections.min(unsettledNodes);
        }
        else return null;
    }

    // reconstruct the path back from destination
    private ArrayList<Node> fullPath() {
        ArrayList<Node> pathList = new ArrayList<>();
        Node previousNode = destination.getPredecessor();
        while (!previousNode.equals(origin)) { // repeat until we reach origin
            pathList.add(previousNode);
            previousNode = previousNode.getPredecessor();
        }
        Collections.reverse(pathList);
        return pathList;
    }

    private Node getUnsettledNode(Node that) {
        for (Node n : unsettledNodes) {
            if (n.equals(that)) {
                return n;
            }
        }
        return null;
    }

    private void setOrigin(Node n) {
        if (n != null) { // origin can't be null or equal to destination
            if(!n.equals(destination)) {
                blockedNodes.remove(n); // remove n from the list of blocked nodes
                origin = n;
                originLabel.setText("Origin: [" + n.getX() + "," + n.getY() + "]");
            }
        }
    }

    private void setDestination(Node n) {
        if (n != null) { // destination can't be null or equal to origin
            if(!n.equals(origin)) {
                blockedNodes.remove(n); // remove n from the list of blocked nodes
                destination = n;
                destinationLabel.setText("Destination: [" + n.getX() + "," + n.getY() + "]");
            }
        }
    }

    private void setBlocked(Node n) {
        if(n != null) {
            // origin and destination can't be added to blocked
            if (!n.equals(origin) && !n.equals(destination)) {
                blockedNodes.add(n);
            }
        }
    }

    private void odSwitch() {
        if(origin != null && destination != null) {
            Node tempOrigin = new Node(destination.getX(), destination.getY());
            Node tempDestination = new Node(origin.getX(), origin.getY());
            destination = null;
            origin = null;
            setDestination(tempDestination);
            setOrigin(tempOrigin);
            visualize();
            board.redraw();
        }
    }

//  handle mouse input for the board
    private void mouseClicked(MouseEvent e) {
        if (!isRunning) {
            int row = board.yToRow(e.getY());
            int column = board.xToColumn(e.getX());
            Node node = new Node(column, row);
            if (e.getButton() == MouseButton.SECONDARY) {
                blockedNodes.remove(node);
            } else if (e.getButton() == MouseButton.PRIMARY) {
                if (e.isControlDown()) {
                    setOrigin(node);
                } else if (e.isAltDown()) {
                    setDestination(node);
                } else {
                    setBlocked(node);
                }
            }
            visualize();
            board.redraw();
        }
    }
    // click+drag to draw or remove walls
    private void mouseDragged(MouseEvent e) {
        int row = board.yToRow(e.getY());
        int column = board.xToColumn(e.getX());
        Node node = new Node(column, row);
        if(e.getButton() == MouseButton.PRIMARY) { // LMB + drag
            setBlocked(node);
        } else if(e.getButton() == MouseButton.SECONDARY) { // RMB + drag
            blockedNodes.remove(node);
        }
        visualize();
        board.redraw();
    }

    private void doStart() {
        if(origin != null && destination != null) {
            origin.setGCost(0);
            current = origin;
            isRunning = true;
            algoDijkstra.setDisable(true);
            algoAstar.setDisable(true);
            diagonalsCheckBox.setDisable(true);
            pathCorrectionCheckBox.setDisable(true);
            manhattanRButton.setDisable(true);
            quadraticRButton.setDisable(true);
            euclideanRButton.setDisable(true);
            diagonalRButton.setDisable(true);
            resetButton.setDisable(true);
            saveButton.setDisable(true);
            loadButton.setDisable(true);
            startButton.setDisable(true);
            odSwitchButton.setDisable(true);
            timer.start();
        }
    }

    private void doStop() {
        timer.stop();
        resetButton.setDisable(false);
        startButton.setDisable(true);
    }

    private void doReset() {
        unsettledNodes.clear();
        settledNodes.clear();
        current = null;
        stepCount = 0;
        visualize();
        board.redraw();
        statLabel.setText("");
        pathLabel.setText("");
        algoDijkstra.setDisable(false);
        algoAstar.setDisable(false);
        manhattanRButton.setDisable(false);
        quadraticRButton.setDisable(false);
        euclideanRButton.setDisable(false);
        diagonalRButton.setDisable(false);
        diagonalsCheckBox.setDisable(false);
        pathCorrectionCheckBox.setDisable(false);
        odSwitchButton.setDisable(false);
        saveButton.setDisable(false);
        loadButton.setDisable(false);
        resetButton.setDisable(true);
        startButton.setDisable(false);
    }

    private void setAlgorithm() {
        if(algoDijkstra.isSelected()) {
            algorithm = Algorithm.Dijkstra;
        } else if (algoAstar.isSelected()) {
            algorithm = Algorithm.Astar;
        }
    }

    private void setInterval() {
        if(speedFaster.isSelected()) {
            interval = 5e6;
        } else if (speedSlower.isSelected()) {
            interval = 1e8;
        }
    }

    private void setAllowDiagonals() {
        allowDiagonals = diagonalsCheckBox.isSelected();
    }

    private void setHeuristic() {
        if(manhattanRButton.isSelected()) {
            heuristic = Heuristic.Manhattan;
        } else if (quadraticRButton.isSelected()) {
            heuristic = Heuristic.Quadratic;
        } else if (euclideanRButton.isSelected()) {
            heuristic = Heuristic.Euclidean;
        } else {
            heuristic = Heuristic.Diagonal;
        }
    }

    private void setPathCorrection() {
        pathCorrection = pathCorrectionCheckBox.isSelected();
    }

    private void visualize() {
        board.clearGrid();

        // draw settled nodes
        for (Node n : settledNodes) {
            board.setCellColor(n.getY(), n.getX(), Color.SALMON);
        }
        // draw unsettled nodes
        for (Node n : unsettledNodes) {
            board.setCellColor(n.getY(), n.getX(), Color.LIGHTSTEELBLUE);
        }
        //draw blocked nodes
        for (Node n: blockedNodes) {
            board.setCellColor(n.getY(), n.getX(), Color.BLACK);
        }
        if (current != null) {
            board.setCellColor(current.getY(), current.getX(), Color.FUCHSIA);
        }
        if (origin != null) {
            board.setCellColor(origin.getY(), origin.getX(), Color.GREEN);
        }
        if (destination != null) {
            board.setCellColor(destination.getY(), destination.getX(), Color.BLUE);
        }

    }

    private void drawPath() {
        visualize();
        ArrayList<Node> pathList = fullPath();
        // The path is actually missing the last step.
        // Here I'm estimating the cost of this step as 1 if diagonal moves are not allowed,
        // and as 1.2 if diagonal moves are allowed (the actual cost of one step
        // in this case could be either 1 or sqrt(2))
        double pathCost = 1;
        if (allowDiagonals) {
            pathCost = 1.2;
        }
        for (int i = 0; i < pathList.size(); i++) {
            int x = pathList.get(i).getX();
            int y = pathList.get(i).getY();
            board.setCellColor(y, x, Color.DARKRED);
            if (i < pathList.size() - 1) {
                pathCost += Math.abs(pathList.get(i).getGCost() - pathList.get(i+1).getGCost());
            }
        }
        pathLabel.setText("Path length: " + pathList.size() + ", cost: " + String.format("%.2f", pathCost));
    }

    // set up the UI and return the root
    public BorderPane setUI() {

        board = new SquareGrid(rows, columns,15);
        board.setDefaultColor(Color.WHITE);
        board.setGridColor(Color.LIGHTGRAY);
        board.setAlwaysDrawGrid(true);
        board.setAutomaticRedraw(false);
        board.setOnMouseClicked(e -> mouseClicked(e));
        board.setOnMouseDragged(e -> mouseDragged(e));

        Font font = Font.font("Monospace", 13);
        Font fontBold = Font.font("Monospace", FontWeight.BOLD, 13);

        originLabel = new Label("Ctrl + click to set origin");
        destinationLabel = new Label("Alt + click to set destination");
        statLabel = new Label();
        pathLabel = new Label();
        originLabel.setFont(font);
        destinationLabel.setFont(font);
        statLabel.setFont(font);
        pathLabel.setFont(fontBold);

        startButton = new Button("Start");
        resetButton = new Button("Reset");
        startButton.setFont(fontBold);
        startButton.setMaxWidth(Double.MAX_VALUE);
        resetButton.setFont(fontBold);
        resetButton.setMaxWidth(Double.MAX_VALUE);
        resetButton.setDisable(true);

        odSwitchButton = new Button("O ⇆ D");
        odSwitchButton.setFont(font);
        odSwitchButton.setMaxWidth(Double.MAX_VALUE);

        saveButton = new Button("Save map");
        loadButton = new Button("Load map");
        saveButton.setFont(font);
        loadButton.setFont(font);
        saveButton.setMaxWidth(Double.MAX_VALUE);
        loadButton.setMaxWidth(Double.MAX_VALUE);
        odSwitchButton.setOnMouseClicked(e -> odSwitch());

        saveButton.setOnMouseClicked(e -> saveMap());
        loadButton.setOnMouseClicked(e -> loadMap());
        startButton.setOnMouseClicked(e -> doStart());
        resetButton.setOnMouseClicked(e -> doReset());

        Label algoLabel = new Label("Algorithm:");
        algoLabel.setFont(fontBold);
        algoLabel.setStyle("-fx-padding: 20px 0 0 0;");
        ToggleGroup algoGroup = new ToggleGroup();
        algoDijkstra = new RadioButton("Dijkstra");
        algoDijkstra.setToggleGroup(algoGroup);
        algoDijkstra.setMaxWidth(Double.MAX_VALUE);
        algoDijkstra.setFont(font);
        algoAstar = new RadioButton("A*");
        algoAstar.setToggleGroup(algoGroup);
        algoAstar.setMaxWidth(Double.MAX_VALUE);
        algoAstar.setFont(font);
        algoAstar.setSelected(true);
        algoDijkstra.setOnAction(e -> setAlgorithm());
        algoAstar.setOnAction(e -> setAlgorithm());

        Label speedLabel = new Label("Speed:");
        speedLabel.setFont(fontBold);
        speedLabel.setStyle("-fx-padding: 20px 0 0 0;");
        ToggleGroup speedGroup = new ToggleGroup();
        speedFaster = new RadioButton("Faster");
        speedFaster.setToggleGroup(speedGroup);
        speedFaster.setMaxWidth(Double.MAX_VALUE);
        speedFaster.setFont(font);
        speedSlower = new RadioButton("Slower");
        speedSlower.setToggleGroup(speedGroup);
        speedSlower.setMaxWidth(Double.MAX_VALUE);
        speedSlower.setFont(font);
        speedSlower.setSelected(true);
        speedFaster.setOnAction(e -> setInterval());
        speedSlower.setOnAction(e -> setInterval());

        Label heuristicLabel = new Label("Heuristic:");
        heuristicLabel.setFont(fontBold);
        heuristicLabel.setStyle("-fx-padding: 20px 0 0 0;");
        ToggleGroup heuristicGroup = new ToggleGroup();
        quadraticRButton = new RadioButton("Quadratic");
        quadraticRButton.setToggleGroup(heuristicGroup);
        quadraticRButton.setFont(font);
        quadraticRButton.setMaxWidth(Double.MAX_VALUE);
        manhattanRButton = new RadioButton("Manhattan");
        manhattanRButton.setToggleGroup(heuristicGroup);
        manhattanRButton.setFont(font);
        manhattanRButton.setMaxWidth(Double.MAX_VALUE);
        euclideanRButton = new RadioButton("Euclidean");
        euclideanRButton.setToggleGroup(heuristicGroup);
        euclideanRButton.setFont(font);
        euclideanRButton.setMaxWidth(Double.MAX_VALUE);
        diagonalRButton = new RadioButton("Diagonal");
        diagonalRButton.setToggleGroup(heuristicGroup);
        diagonalRButton.setFont(font);
        diagonalRButton.setMaxWidth(Double.MAX_VALUE);
        euclideanRButton.setSelected(true);
        quadraticRButton.setOnAction(e -> setHeuristic());
        manhattanRButton.setOnAction(e -> setHeuristic());
        euclideanRButton.setOnAction(e -> setHeuristic());
        diagonalRButton.setOnAction(e -> setHeuristic());

        diagonalsCheckBox = new CheckBox("Allow\ndiagonal\nmovement");
        diagonalsCheckBox.setStyle("-fx-padding: 20px 0 0 0;");
        diagonalsCheckBox.setSelected(true);
        diagonalsCheckBox.setFont(font);
        diagonalsCheckBox.setOnAction(e -> setAllowDiagonals());

        pathCorrectionCheckBox = new CheckBox("On-the-fly\npath\ncorrection");
        pathCorrectionCheckBox.setStyle("-fx-padding: 0 0 20px 0;");
        pathCorrectionCheckBox.setSelected(false);
        pathCorrectionCheckBox.setFont(font);
        pathCorrectionCheckBox.setOnAction(e -> setPathCorrection());

        VBox rightPane = new VBox(10, startButton, resetButton, saveButton, loadButton,
                algoLabel, algoAstar, algoDijkstra,
                diagonalsCheckBox,
                pathCorrectionCheckBox,
                odSwitchButton,
                heuristicLabel, quadraticRButton, manhattanRButton, euclideanRButton, diagonalRButton,
                speedLabel, speedFaster, speedSlower);
        rightPane.setStyle("-fx-padding: 8px;");
        rightPane.setAlignment(Pos.TOP_LEFT);

        HBox bottomPane = new HBox(30, originLabel,
                destinationLabel,
                statLabel,
                pathLabel);
        bottomPane.setStyle("-fx-padding: 5px;");

        BorderPane root = new BorderPane();
        root.setCenter(board);
        root.setRight(rightPane);
        root.setBottom(bottomPane);
        return root;
    }


    private void saveMap() {

        // don't do anything if there's no origin and/or destination
        if (origin == null || destination == null) {
            return;
        }

        FileChooser fileChooser = new FileChooser();
        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Path map", "*.pathmap"));
        FileWriter fw = null;
        BufferedWriter bw = null;
        File saveFile = fileChooser.showSaveDialog(root.getScene().getWindow());
        if (saveFile != null) {
            try {
                fw = new FileWriter(saveFile);
                bw = new BufferedWriter(fw);
                bw.write("O," + origin.getX() +"," + origin.getY() + "\n");
                bw.write("D," + destination.getX() + "," + destination.getY() + "\n");
                if(!blockedNodes.isEmpty()) {
                    for (Node n : blockedNodes) {
                        bw.write("B," + n.getX() + "," + n.getY() + "\n");
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            } finally {
                try {
                    if (bw != null) {
                        bw.close();
                        fw.close();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private void loadMap() {
        FileChooser fileChooser = new FileChooser();
        fileChooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("Path map", "*.pathmap"));
        File loadFile = fileChooser.showOpenDialog(root.getScene().getWindow());
        FileReader fr = null;
        BufferedReader br = null;
        if (loadFile != null) {
            settledNodes.clear();
            unsettledNodes.clear();
            blockedNodes.clear();
            origin = null;
            destination = null;
            try {
                fr = new FileReader(loadFile);
                br = new BufferedReader(fr);
                String input;
                // reading the file line by line
                while((input = br.readLine()) != null) {
                    // split the line using comma as a separator
                    String[] splitLine = input.split(",");
                    // if there are three parts, proceed with parsing
                    if(splitLine.length == 3) {
                        int x = -1;
                        int y = -1;
                        try {
                            x = Integer.parseInt(splitLine[1]); // try to parse x
                            y = Integer.parseInt(splitLine[2]); // try to parse y
                        } catch (NumberFormatException e) {
                            e.printStackTrace();
                        }
                        if(x != -1 && y != -1) { // if parsing successful, look at the type of the node
                            switch (splitLine[0]) {
                                case "O": // origin
                                    setOrigin(new Node(x, y));
                                    break;
                                case "D": // destination
                                    setDestination(new Node(x, y));
                                    break;
                                case "B": // blocked
                                    setBlocked(new Node(x, y));
                                    break;
                            }
                        }
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            } finally {
                try {
                    if (br != null) {
                        br.close();
                        fr.close();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            visualize();
            board.redraw();
        }
    }

    public static void main(String[] args) {
        launch(args);
    }

    public static int square(int n) {
        return n*n;
    }
}