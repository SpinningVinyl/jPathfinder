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

    private SquareGrid board;


    private BorderPane root;
    private Label originLabel;
    private Label destinationLabel;
    private Label statLabel;
    private Label pathLabel;
    private Button resetButton;
    private Button startButton;
    private Button saveButton;
    private Button loadButton;

    private RadioButton algoDijkstra;
    private RadioButton algoAstar;
    private RadioButton speedFaster;
    private RadioButton speedSlower;
    private RadioButton heuristicQuadratic;
    private RadioButton heuristicLinear;
    private RadioButton heuristicPythagorean;

    private CheckBox diagonalsCheckBox;

    private final int columns = 75;
    private final int rows = 50;

    private int stepCount = 0;

    private enum Algorithm { Dijkstra,
                        Astar };

    private enum Heuristic { Linear,
                            Quadratic,
                            Pythagorean };

    private Heuristic heuristic = Heuristic.Linear;

    private Algorithm algorithm = Algorithm.Astar;
    private boolean allowDiagonals = true;

    private boolean isRunning = false;

    private AnimationTimer timer;
    private double interval = 1e8;


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
        for (int px = -1; px <= 1; px++) {
            for (int py = -1; py <= 1; py++) {
                if (px == 0 && py == 0) { // this is the current node itself
                    continue;
                }
                int nextX = previous.getX() + px;
                int nextY = previous.getY() + py;

//              calculates costs for the node at nextX, nextY and returns it
                Node tempNode = processNode(nextX, nextY, previous);
                if (tempNode != null) {
                    unsettledNodes.add(tempNode);
                }
            }
        }
        // set the current node to the lowest cost unsettled node
        Node next = lowestCostNode();

        if (next == null) {
            // no path
            isRunning = false;
            pathLabel.setText("No path found!");
        } else if(next.equals(destination)) {
            // found the path
            destination.setPrevious(next.getPrevious());
            isRunning = false;
            drawPath();
        }

//      remove the current node from unsettled
        unsettledNodes.remove(next);

//      add the current node to settled
        settledNodes.add(next);

//      return the current node
        return next;
    }

//    calculate costs for node at x,y and return the node
    private Node processNode(int x, int y, Node currentNode) {
        stepCount++;
        if (x < 0 || y < 0 || x >= columns || y >= rows) {
            return null;
        }
//      prevent the algorithm from jumping across diagonal borders,
//      this also disables cutting corners
        if((Math.abs(x - currentNode.getX()) != 0) && (Math.abs(y - currentNode.getY()) != 0)) {
            if (blockedNodes.contains(new Node(x,currentNode.getY())) || blockedNodes.contains(new Node(currentNode.getX(),y))) {
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
        node.setPrevious(currentNode);
        // calculate G cost
        int distanceX = Math.abs(x - currentNode.getX());
        int distanceY = Math.abs(y - currentNode.getY());
        double gCost = currentNode.getGCost() + Math.sqrt(square(distanceX) + square(distanceY));
        node.setGCost(gCost);

        if (algorithm == Algorithm.Dijkstra) { // Dijkstra doesn't include the heuristic element
            // F cost = G cost
            node.setFCost(gCost);
        } else { // calculate H cost
            int distanceToDestX = Math.abs(x - destination.getX());
            int distanceToDestY = Math.abs(y - destination.getY());
            double hCost = 0;
            switch(heuristic) {
                case Linear:
                    hCost = distanceToDestX + distanceToDestY;
                    break;
                case Pythagorean:
                    hCost = Math.sqrt(square(distanceToDestX) + square(distanceToDestY));
                    break;
                case Quadratic:
                    hCost = square(distanceToDestX) + square(distanceToDestY);
            }
            node.setHCost(hCost);
            // F cost = G cost + H cost
            node.setFCost(gCost + hCost);
        }
        return node;
    }

    // returns the lowest cost node from the list of unsettled nodes
    private Node lowestCostNode() {
        if(unsettledNodes.size() != 0) {
            return Collections.min(unsettledNodes);
        }
        else return null;
    }

    // reconstruct the path back from destination
    private ArrayList<Node> fullPath() {
        ArrayList<Node> pathList = new ArrayList<>();
        Node previousNode = destination.getPrevious();
        while (!previousNode.equals(origin)) {
            pathList.add(previousNode);
            previousNode = previousNode.getPrevious();
        }
        Collections.reverse(pathList);
        return pathList;
    }

    private void setOrigin(Node n) {
        if (n != null) {
            origin = n;
            originLabel.setText("Origin: ["+n.getX() + "," + n.getY() + "]");
        }
    }

    private void setDestination(Node n) {
        if (n != null) {
            destination = n;
            destinationLabel.setText("Destination: [" + n.getX() + "," + n.getY() + "]");
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
                    blockedNodes.remove(node);
                    setOrigin(node);
                } else if (e.isAltDown()) {
                    blockedNodes.remove(node);
                    setDestination(node);
                } else {
                    if (!node.equals(origin) && !node.equals(destination)) {
                        blockedNodes.add(node);
                    }
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
        if(e.getButton() == MouseButton.PRIMARY) { // LMB+drag
            if(!node.equals(origin) && ! node.equals(destination)) {
                blockedNodes.add(node);
            }
        } else if(e.getButton() == MouseButton.SECONDARY) {
            if (board.getCellColor(row, column) == Color.BLACK) {
                blockedNodes.remove(node);
            }
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
            heuristicLinear.setDisable(true);
            heuristicQuadratic.setDisable(true);
            heuristicPythagorean.setDisable(true);
            resetButton.setDisable(true);
            saveButton.setDisable(true);
            loadButton.setDisable(true);
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
        board.clearGrid();
        stepCount = 0;
        visualize();
        board.redraw();
        statLabel.setText("");
        pathLabel.setText("");
        algoDijkstra.setDisable(false);
        algoAstar.setDisable(false);
        heuristicLinear.setDisable(false);
        heuristicQuadratic.setDisable(false);
        heuristicPythagorean.setDisable(false);
        diagonalsCheckBox.setDisable(false);
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
            interval = 1e7;
        } else if (speedSlower.isSelected()) {
            interval = 1e8;
        }
    }

    private void setAllowDiagonals() {
        allowDiagonals = diagonalsCheckBox.isSelected();
    }

    private void setHeuristic() {
        if(heuristicLinear.isSelected()) {
            heuristic = Heuristic.Linear;
        } else if (heuristicQuadratic.isSelected()) {
            heuristic = Heuristic.Quadratic;
        } else if (heuristicPythagorean.isSelected()) {
            heuristic = Heuristic.Pythagorean;
        }
    }

    private void visualize() {
        // draw settled nodes
        board.clearGrid();
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
            board.setCellColor(origin.getY(), origin.getX(), Color.DARKGREEN);
        }
        if (destination != null) {
            board.setCellColor(destination.getY(), destination.getX(), Color.DARKBLUE);
        }

    }

    private void drawPath() {
        visualize();
        ArrayList<Node> pathList = fullPath();
        for (Node n : pathList) {
            board.setCellColor(n.getY(), n.getX(), Color.RED);
        }
        board.redraw();
        pathLabel.setText("Path length: " + pathList.size());
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

        saveButton = new Button("Save map");
        loadButton = new Button("Load map");
        saveButton.setFont(font);
        loadButton.setFont(font);
        saveButton.setMaxWidth(Double.MAX_VALUE);
        loadButton.setMaxWidth(Double.MAX_VALUE);

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
        heuristicQuadratic = new RadioButton("Quadratic");
        heuristicQuadratic.setToggleGroup(heuristicGroup);
        heuristicQuadratic.setFont(font);
        heuristicQuadratic.setMaxWidth(Double.MAX_VALUE);
        heuristicLinear = new RadioButton("Linear");
        heuristicLinear.setToggleGroup(heuristicGroup);
        heuristicLinear.setFont(font);
        heuristicLinear.setMaxWidth(Double.MAX_VALUE);
        heuristicPythagorean = new RadioButton("Pythagorean");
        heuristicPythagorean.setToggleGroup(heuristicGroup);
        heuristicPythagorean.setFont(font);
        heuristicPythagorean.setMaxWidth(Double.MAX_VALUE);
        heuristicLinear.setSelected(true);
        heuristicQuadratic.setOnAction(e -> setHeuristic());
        heuristicLinear.setOnAction(e -> setHeuristic());
        heuristicPythagorean.setOnAction(e -> setHeuristic());

        diagonalsCheckBox = new CheckBox("Allow\ndiagonal\nmovement");
        diagonalsCheckBox.setStyle("-fx-padding: 20px 0 0 0;");
        diagonalsCheckBox.setSelected(true);
        diagonalsCheckBox.setFont(font);
        diagonalsCheckBox.setOnAction(e -> setAllowDiagonals());


        VBox rightPane = new VBox(10, startButton, resetButton, saveButton, loadButton,
                algoLabel, algoAstar, algoDijkstra,
                diagonalsCheckBox,
                heuristicLabel, heuristicQuadratic, heuristicLinear, heuristicPythagorean,
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
                while((input = br.readLine()) != null) {
                    String[] splitLine = input.split(",");
                    if(splitLine.length != 3) {
                        break;
                    } else {
                        int x = 0;
                        int y = 0;
                        try {
                            x = Integer.parseInt(splitLine[1]);
                            y = Integer.parseInt(splitLine[2]);
                        } catch (NumberFormatException e) {
                            e.printStackTrace();
                        }
                        switch(splitLine[0]) {
                            case "O":
                                setOrigin(new Node(x, y));
                                break;
                            case "D":
                                setDestination(new Node(x, y));
                                break;
                            case "B":
                                blockedNodes.add(new Node(x, y));
                                break;
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