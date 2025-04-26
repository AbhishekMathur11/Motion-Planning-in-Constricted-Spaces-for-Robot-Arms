# Motion Planning in Constricted Spaces for Robot Arms


In many industrial applications, robot arms are required to perform more precise tasks as opposed to trivial pick and place operations and these might occur in constricted spaces where the chances of a collision are higher. Hence, this project focuses on the implementation of key motion planning algorithms for a Franka Emika Panda robot arm inside a box environment with an obstacle and the goal is for the arm to plan a path from one side of the obstacle to the other, simulating a real world object transfer application.

Check out this page for demos of implementations:

[Click here](https://www.abhishekramanmathur.com/projects/motion-planning-in-constricted-spaces)



## 📌 Rapidly-exploring Random Tree (RRT)

**RRT** is an incremental sampling-based algorithm designed to efficiently explore high-dimensional spaces.

### 🔧 Algorithm Steps:
1. **Initialization**: Start with a tree rooted at the initial configuration.
2. **Sampling**: Uniformly sample a random configuration `q_rand` from the space.
3. **Nearest Neighbor**: Find the nearest node `q_near` in the tree to `q_rand`.
4. **Extension**: Extend from `q_near` towards `q_rand` by a fixed step size to generate `q_new`.
5. **Collision Checking**: Add `q_new` to the tree only if the edge from `q_near` to `q_new` is collision-free.
6. **Goal Check**: If `q_new` is close to the goal, attempt to connect it directly to the goal.

### ✅ Pros:
- Fast and scalable to high-dimensional spaces
- Easy to implement

### ❌ Cons:
- No guarantee of finding the optimal path
- Paths may be jagged and suboptimal

---

## 📌 Probabilistic Roadmap (PRM)

**PRM** is a multi-query planning algorithm that constructs a graph (roadmap) in the configuration space before any queries are made.

### 🔧 Algorithm Steps:
1. **Sampling**: Generate `N` random collision-free configurations in the space.
2. **Connection**: For each configuration, connect it to its `k` nearest neighbors using a local planner (e.g., straight-line planner), if the path is collision-free.
3. **Query**:
   - Add the start and goal nodes to the roadmap.
   - Connect them to nearby roadmap nodes.
   - Search the graph using Dijkstra or A* to find a path.

### ✅ Pros:
- Efficient for multiple queries after roadmap construction
- Reusable graph structure

### ❌ Cons:
- Not suitable for dynamic or single-query problems
- Connectivity depends heavily on sampling density and connection radius

---

## 📌 RRT\* (RRT Star)

**RRT\*** is an asymptotically optimal variant of RRT that rewires the tree to minimize path cost over time.

### 🔧 Algorithm Steps:
Same as RRT with two major enhancements:
1. **Choose Parent**: When adding `q_new`, choose the parent from nearby nodes that results in the lowest cost path to `q_new`.
2. **Rewire**: After adding `q_new`, check if any nearby nodes can be connected through `q_new` at a lower cost and update their parent if so.

### ✅ Pros:
- Asymptotically optimal (converges to optimal path as number of samples increases)
- Maintains advantages of RRT

### ❌ Cons:
- More computationally expensive than RRT
- Slower convergence in very cluttered environments

---

