package com.pavelurusov.pathfinder;

import java.io.Serializable;
import java.util.Objects;

/** @author Pavel Urusov, me@pavelurusov.com
 * This class is used by my A* / Dijkstra pathfinder.
 */

public class Node implements Comparable<Node>, Serializable {
    private final int x, y; // coordinates
    private double gCost;
    private double hCost;
    private double fCost; // gCost - the cost to get from origin to this node,
    // hCost - estimated (heuristic) cost to get from this node to destination,
    // fCost = full cost of this node
    private static final long serialVersionUID = 46_467_160L;

    private Node previous;

    public Node(int x, int y) {
        this.x = Math.max(x, 0);
        this.y = Math.max(y, 0);
    }

    public void setFCost(double fCost) {
        this.fCost = Math.max(fCost, 0);
    }

    public void setGCost(double gCost) {
        this.gCost = Math.max(gCost, 0);
    }

    public void setHCost(double hCost) {
        this.hCost = Math.max(hCost, 0);
    }

    public void setPrevious(Node parent) {
        if (parent != null) {
            this.previous = parent;
        }
    }

    public int getX() {
        return this.x;
    }

    public int getY() {
        return this.y;
    }

    public double getFCost() {
        return fCost;
    }

    public double getGCost() {
        return gCost;
    }

    public double getHCost() {
        return hCost;
    }

    public Node getNode() {
        return this;
    }

    public Node getPrevious() {
        return previous;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Node node = (Node) o;
        return x == node.getX() && y == node.getY();
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }

    @Override
    public int compareTo(Node n) {
        return Double.compare(this.fCost, n.getFCost());
    }

}
