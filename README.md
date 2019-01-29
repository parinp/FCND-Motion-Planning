## Project: 3D Motion Planning

### Objective: Path Planning

The underlying objective of the given task in **README(Udacity).md** is to calculate the shortest path, in the form of waypoints, from a __starting position__ to a **goal location** while avoiding obstacles in the city which are provided in the form of a 'csv' file.  Using the  skills taught in the lesson, I have chosen and implemented 2 approaches:

**NOTE:** The following code is written in python 

1. [**Grid Map**](#grid-map)
2. [**Probabilistic Roadmap**](probabilistic-roadmap)

### Grid Map

#### Grid Formation and Location

All of the relevant code is written in the file **motion_planning.py** and to make the programming less clustered, some of the functions are written in the file **planning_utils.py**.

First and foremost, for all of this to work, the quadrotor will have to be able to identify its location in longitude, latitude and altitude.

```python

file = 'colliders.csv'


```
  


### Probabilistic Roadmap

All of the relevant code is written in the file **motion_planning.py** and to make the programming less clustered, some of the functions are written in the file **planning_utils.py**.

