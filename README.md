# AStarPathFinder_CPP
A* path finding implementation in C++

```
// This is a quick and dirty port of the Python code from:
// https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
//
// We aren't trying to do anything special or claim any brilliant insights
// on A* algorithm implementations. We simply need a quick and dirty
// implementation for a game so that we don't have to pull in any external
// dependencies that in turn bring in their own view of the world which
// never meshes with ours.
```

Diagonal nodes are not part of the path result because in my own game I do not
support diagonal movement. This is easy enough to uncomment if you desire.

![Sample](screenshots/sample.png)

Frank Hale &lt;frankhaledevelops@gmail.com&gt;

28 March 2021