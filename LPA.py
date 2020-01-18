def main():
  initialize();
  while (true) {
    computeShortestPath();
    while (!hasCostChanges())
      sleep;
    for (edge : getChangedEdges()) {
        edge.setCost(getNewCost(edge));
        updateNode(edge.endNode);


void initialize() {
  queue = new PriorityQueue();
  for (node : getAllNodes()) {
    node.g = INFINITY;
    node.rhs = INFINITY;
  }
  start.rhs = 0;
  queue.insert(start, calculateKey(start));
}

/** Expands the nodes in the priority queue. */
void computeShortestPath() {
  while ((queue.getTopKey() < calculateKey(goal)) || (goal.rhs != goal.g)) {
    node = queue.pop();
    if (node.g > node.rhs) {
      node.g = node.rhs;
      for (successor : node.getSuccessors())
        updateNode(successor);
    } else {
      node.g = INFINITY;
      updateNode(node);
      for (successor : node.getSuccessors())
        updateNode(successor);
    }
  }
}

/** Recalculates rhs for a node and removes it from the queue.
 * If the node has become locally inconsistent, it is (re-)inserted into the queue with its new key. */
void updateNode(node) {
  if (node != start) {
    node.rhs = INFINITY;
    for (predecessor: node.getPredecessors())
      node.rhs = min(node.rhs, predecessor.g + predecessor.getCostTo(node));
    if (queue.contains(node))
      queue.remove(node);
    if (node.g != node.rhs)
      queue.insert(node, calculateKey(node));
  }
}

int[] calculateKey(node) {
  return {min(node.g, node.rhs) + node.getHeuristic(goal), min(node.g, node.rhs)};
}