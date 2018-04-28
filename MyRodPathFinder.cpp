//
// Created by t-idkess on 09-Apr-18.
//

#include "MyRodPathFinder.h"
#include "MyQueryHandler.h"

#define STEPS 64
#define Nbbox 10
#define Nrand 900
#define K 20
#define TIMEOUT 100
#define TRANSLATION_WEIGHT 0.8

struct qPoint {
	Point_2 xy;
	double rotation;
	int index;
	double vec[3];

	void getPoint(double x, double y) {
		xy = Point_2(x,y);
		vec[0]=x;
		vec[1]=y;
	}

	  bool operator==(const qPoint& p) const
	  {
	    return (index==p.index)  ;
	  }
	  bool  operator!=(const qPoint& p) const { return ! (*this == p); }
};

struct Construct_coord_iterator {
  typedef  const double* result_type;
  const double* operator()(const qPoint& p) const
  { return static_cast<const double*>(p.vec); }
  const double* operator()(const qPoint& p, int)  const
  { return static_cast<const double*>(p.vec+3); }
};

FT globalRodLength;


double rand_between(double high, double low) {
	return low + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(high-low)));
}

qPoint newRandomQPoint(double xmin, double xmax, double ymin, double ymax) {
	double x = rand_between(xmin,xmax);
	double y = rand_between(ymin, ymax);
	double rotation = rand_between(0,2*CGAL_PI);

	qPoint p;
	p.getPoint(x,y);
	p.rotation = rotation;
	p.vec[2] = rotation;
	return p;
}


double dist_1(qPoint p1, qPoint p2) {
	FT r = globalRodLength;
	Vector_2 direction1 = {cos(p1.rotation), sin(p1.rotation)},
			direction2 = {cos(p2.rotation), sin(p2.rotation)};
	Segment_2 robot1 = Segment_2(p1.xy,p1.xy+(direction1*r)),
			robot2 = Segment_2(p2.xy,p2.xy+(direction2*r));

	Point_2 s1 = robot1.source(), t1 = robot1.target(),
			s2 = robot2.source(), t2 = robot2.target();

	FT sDist = CGAL::squared_distance(s1, s2);
	FT dDist = CGAL::squared_distance(t1, t2);

	return sDist.to_double() + dDist.to_double();
}

double dist_2(qPoint p1, qPoint p2, bool isClockwise,double rodLength) {
	double tw = TRANSLATION_WEIGHT;
	double rw = 1 - tw;
	double t_dist = CGAL::squared_distance(p1.xy, p2.xy).to_double();
	double r_dist = (p1.rotation - p2.rotation);// * (isClockwise ? -1 : 1);
	if (isClockwise) {
		r_dist = (r_dist>=0?r_dist:r_dist+2*CGAL_PI)*rodLength;
	} else {
		r_dist = (r_dist<0?-r_dist:r_dist+2*CGAL_PI)*rodLength;
	}
	return tw * t_dist + rw * r_dist;
}


double dist(qPoint p1, qPoint p2, bool isClockwise) {
	return dist_1(p1, p2);
}

double dist_min(qPoint p1, qPoint p2) {
	return min(dist(p1, p2, true), dist(p1, p2, false));
}

struct Neighbor {
	qPoint p;
	double distance;
	bool isClockwise;
};

struct setComp{

	bool operator()(const Neighbor &n1 ,const Neighbor &n2) {
		return n1.distance < n2.distance;
	}
};

// fix to range [0, 2pi)
double fixedAngle(double angle) {
	return fmod(angle, 2*CGAL_PI);
}

qPoint getPointAtStep(double i, qPoint q1, qPoint q2, bool isClockwise) {
	qPoint q;
	//double cwRotation = fixedAngle(q1.rotation + (q2.rotation-q1.rotation)*i/STEPS);
	//double ccwRotation = fixedAngle(q1.rotation+ (q2.rotation-q1.rotation)*i/STEPS);

	double ccwRotation = q2.rotation-q1.rotation>=0?q2.rotation-q1.rotation:q2.rotation-q1.rotation+2*CGAL_PI;
	ccwRotation = fixedAngle(q1.rotation+ccwRotation*i);
	double cwRotation = q2.rotation-q1.rotation>=0?-(2*CGAL_PI-(q2.rotation-q1.rotation)):q2.rotation-q1.rotation;
		cwRotation = fixedAngle(q1.rotation+cwRotation*i);

	double x1 = q1.xy.x().to_double(), y1 = q1.xy.y().to_double(),
			x2 = q2.xy.x().to_double(), y2 = q2.xy.y().to_double();
	double x = x1 + (i) * (x2-x1),
			y = y1 + (i) * (y2-y1);

	q.xy = Point_2(x,y);
	q.rotation = (isClockwise ? cwRotation : ccwRotation);

	return q;
}

int minDistance(double* dist, bool* sptSet, int V)
//for Dijkstra
{
   // Initialize min value
   double min = numeric_limits<double>::max();
   int min_index;

   for (int v = 0; v < V; v++)
     if (sptSet[v] == false && dist[v] <= min)
         min = dist[v], min_index = v;

   return min_index;
}

vector<int> dijkstra(vector<vector<double>> graph, int V, int src, int target)
{
	vector<int> path;
	double MAX = numeric_limits<double>::max();
    double* dist = (double*) calloc(V, sizeof(double));     // The output array.  dist[i] will hold the shortest
                      // distance from src to i

    bool* sptSet = (bool*) calloc(V, sizeof(bool)); // sptSet[i] will true if vertex i is included in shortest
                     // path tree or shortest distance from src to i is finalized

	int* prev = (int*) calloc(V, sizeof(int));

     // Initialize all distances as INFINITE and stpSet[] as false
     for (int i = 0; i < V; i++)
        dist[i] = MAX, sptSet[i] = false, prev[i]=-1;

     // Distance of source vertex from itself is always 0
     dist[src] = 0;

     // Find shortest path for all vertices
     for (int count = 0; count < V-1; count++)
     {
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in first iteration.
       int u = minDistance(dist, sptSet, V);

       // Mark the picked vertex as processed
       sptSet[u] = true;

       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < V; v++)

         // Update dist[v] only if is not in sptSet, there is an edge from
         // u to v, and total weight of path from src to  v through u is
         // smaller than current value of dist[v]
         if (!sptSet[v] && graph[u][v] && dist[u] != MAX
                                       && dist[u]+graph[u][v] < dist[v]) {
            dist[v] = dist[u] + graph[u][v];
			prev[v] = u;
		}
     }

	int vertex = target;
	while(vertex != -1) {
		path.insert(path.begin(), vertex);
		vertex = prev[vertex];
	}

	free(dist);
	free(sptSet);
	free(prev);

	return path;

     // print the constructed distance array
    //  printSolution(dist, V);
}

/*
 * @return:
 * 1 - clockwise route
 * -1 - counter-clockwise route
 * 2 - no route
 */

double localPlanner (qPoint q1 ,qPoint q2, MyQueryHandler handler) {

	double d[2] = {-1, -1};
	for(int countDirection = 0; countDirection < 2; countDirection++) {
		bool isClockwise = 1-countDirection;
		bool collides = false;

		int currStepSize = 2;

		while(currStepSize!=STEPS && !collides) {
			for (int i = 1; i<currStepSize; i=i+2) {
			qPoint qMid = getPointAtStep((double)i/currStepSize, q1, q2, isClockwise);
			if(!handler.isLegalConfiguration(qMid.xy, qMid.rotation)) {
					collides = true;
					break;
				}
			}

			currStepSize=currStepSize*2;
		}

		if(!collides)
			d[countDirection] = dist(q1, q2, isClockwise);
	}

	// no route
	if(d[0]<0 && d[1]<0) {
		return 2;
	}

	// clockwise is a valid route that is neccessarily shorter than cc
	if((d[0] < d[1] && (d[1]>=0 && d[0]>=0)) || (d[0]>=0 && d[1]<0)) {
		return 1;
	}
		return -1;
}

short getDirection(short direction, qPoint q1, qPoint q2, MyQueryHandler handler) {
	if(direction == 0) {
		direction = localPlanner(q1, q2, handler);
	}
	return direction;
}

struct Distance {
  typedef qPoint Query_item;
  typedef double FT;
  typedef CGAL::Dimension_tag<3> D;
  double transformed_distance(const qPoint& p1, const qPoint& p2) const {
    return dist_min(p1,p2);
  }
  double min_distance_to_rectangle(const qPoint& p,
                   const CGAL::Kd_tree_rectangle<FT,D>& b) const {
    double distance(0.0), h = p.xy.x().to_double();
    if (h < b.min_coord(0)) distance += (b.min_coord(0)-h)*(b.min_coord(0)-h);
    if (h > b.max_coord(0)) distance += (h-b.max_coord(0))*(h-b.max_coord(0));
    h=p.xy.y().to_double();
    if (h < b.min_coord(1)) distance += (b.min_coord(1)-h)*(b.min_coord(1)-h);
    if (h > b.max_coord(1)) distance += (h-b.max_coord(1))*(h-b.min_coord(1));
    h=p.rotation;
    if (h < b.min_coord(2)) distance += (b.min_coord(2)-h)*(b.min_coord(2)-h);
    if (h > b.max_coord(2)) distance += (h-b.max_coord(2))*(h-b.max_coord(2));
    return distance;
  }
  double min_distance_to_rectangle(const qPoint& p,
                   const CGAL::Kd_tree_rectangle<FT,D>& b,std::vector<double>& dists){
    double distance(0.0), h = p.xy.x().to_double();
    if (h < b.min_coord(0)){
      dists[0] = (b.min_coord(0)-h);
      distance += dists[0]*dists[0];
    }
    if (h > b.max_coord(0)){
      dists[0] = (h-b.max_coord(0));
      distance += dists[0]*dists[0];
    }
    h=p.xy.y().to_double();
    if (h < b.min_coord(1)){
      dists[1] = (b.min_coord(1)-h);
      distance += dists[1]*dists[1];
    }
    if (h > b.max_coord(1)){
      dists[1] = (h-b.max_coord(1));
      distance += dists[1]*dists[1];
    }
    h=p.rotation;
    if (h < b.min_coord(2)){
      dists[2] = (b.min_coord(2)-h);
      distance += dists[2]*dists[2];
    }
    if (h > b.max_coord(2)){
      dists[2] = (h-b.max_coord(2));
      distance += dists[2]*dists[2];
    }
    return distance;
  }
  double max_distance_to_rectangle(const qPoint& p,
                   const CGAL::Kd_tree_rectangle<FT,D>& b) const {
    double h = p.xy.x().to_double();
    double d0 = (h >= (b.min_coord(0)+b.max_coord(0))/2.0) ?
                (h-b.min_coord(0))*(h-b.min_coord(0)) : (b.max_coord(0)-h)*(b.max_coord(0)-h);
    h=p.xy.y().to_double();
    double d1 = (h >= (b.min_coord(1)+b.max_coord(1))/2.0) ?
                (h-b.min_coord(1))*(h-b.min_coord(1)) : (b.max_coord(1)-h)*(b.max_coord(1)-h);
    h=p.rotation;
    double d2 = (h >= (b.min_coord(2)+b.max_coord(2))/2.0) ?
                (h-b.min_coord(2))*(h-b.min_coord(2)) : (b.max_coord(2)-h)*(b.max_coord(2)-h);
    return d0 + d1 + d2;
  }
  double max_distance_to_rectangle(const qPoint& p,
                   const CGAL::Kd_tree_rectangle<FT,D>& b,std::vector<double>& dists){
	double h = p.xy.x().to_double();
    dists[0] = (h >= (b.min_coord(0)+b.max_coord(0))/2.0) ?
                (h-b.min_coord(0)) : (b.max_coord(0)-h);

    h=p.xy.y().to_double();
    dists[1] = (h >= (b.min_coord(1)+b.max_coord(1))/2.0) ?
                (h-b.min_coord(1)) : (b.max_coord(1)-h);
    h=p.rotation;
    dists[2] = (h >= (b.min_coord(2)+b.max_coord(2))/2.0) ?
                (h-b.min_coord(2)) : (b.max_coord(2)-h);
    return dists[0] * dists[0] + dists[1] * dists[1] + dists[2] * dists[2];
  }
  double new_distance(double& dist, double old_off, double new_off,
              int /* cutting_dimension */)  const {
    return dist + new_off*new_off - old_off*old_off;
  }
  double transformed_distance(double d) const { return d*d; }
  double inverse_of_transformed_distance(double d) { return std::sqrt(d); }
}; // end of struct Distance


typedef CGAL::Dimension_tag<3> D;
typedef CGAL::Search_traits<double, qPoint, const double*, Construct_coord_iterator, D> Traits;
typedef CGAL::Orthogonal_k_neighbor_search<Traits, Distance> K_neighbor_search;
typedef K_neighbor_search::Tree Tree;


vector<Path::PathMovement>
MyRodPathFinder::getPath(FT rodLength, Point_2 rodStartPoint, double rodStartRotation, Point_2 rodEndPoint,
                         double rodEndRotation, vector<Polygon_2> obstacles) {
	globalRodLength = rodLength;
	vector<Path::PathMovement> res;
//	int n = 10; //number of nodes to put in the Roadmap
//	int k = 5; //number of closest neighbors to examine for each configuration
//	int timeout = 1000;
	vector<qPoint> V; //Vertices;

	MyQueryHandler queryHandler(rodLength,obstacles);
	

	//TODO : find solution
//	CGAL::Bbox_2 bbox(1, -1, 1, -1);

	CGAL::Bbox_2 bbox = obstacles[0].bbox();

	for (Polygon_2 p: obstacles) {

		bbox = bbox+p.bbox();

	}

	bbox = bbox + Segment_2(rodStartPoint,rodEndPoint).bbox();

	//bbox = CGAL::Bbox_2(-2,-2,2,2);

	float bsr = 0.1;
	double xmin = bbox.xmin(), xmax = bbox.xmax(),
	ymin = bbox.ymin(), ymax = bbox.ymax();

	qPoint qstart, qend;
	qstart.xy = rodStartPoint;
	qstart.rotation = rodStartRotation;
	qstart.index = 0;
	qend.xy = rodEndPoint;
	qend.rotation = rodEndRotation;
	qend.index = 1;
	V.push_back(qstart);
	V.push_back(qend);

	int currInd = 2;


	//4*Nbbox configurations on bbox
	for (int i=0; i<Nbbox; i++) {
			int counter = 0;
			double x = (xmin-bsr)*(double)(i/Nbbox-1)+(xmax+bsr)*(1-double(i/Nbbox-1));
			double y = (ymin-bsr);
				qPoint p;
			p.getPoint(x,y);
			bool found = false;
			while (counter < TIMEOUT && !found ) {
				double rotation = rand_between(0,2*CGAL_PI);
				p.rotation = rotation;
				p.vec[2] = rotation;
					if(queryHandler.isLegalConfiguration(p.xy,p.rotation)) {
						p.index = currInd;
						V.push_back(p);
						currInd++;
						found = true;
					}
				counter++;
			}

		}

	for (int i=0; i<Nbbox; i++) {
			int counter = 0;
			double x = (xmin-bsr)*(double)(i/Nbbox-1)+(xmax+bsr)*(1-double(i/Nbbox-1));
			double y = (ymax+bsr);
				qPoint p;
			p.getPoint(x,y);
			bool found = false;
			while (counter < TIMEOUT && !found ) {
				double rotation = rand_between(0,2*CGAL_PI);
				p.rotation = rotation;
				p.vec[2] = rotation;
					if(queryHandler.isLegalConfiguration(p.xy,p.rotation)) {
						p.index = currInd;
						V.push_back(p);
						currInd++;
						found = true;
					}
				counter++;
			}

		}

	for (int i=0; i<Nbbox; i++) {
			int counter = 0;
			double x = (xmin-bsr);
			double y = (ymin-bsr)*(double)(i/Nbbox-1)+(ymax+bsr)*(1-double(i/Nbbox-1));
				qPoint p;
			p.getPoint(x,y);
			bool found = false;
			while (counter < TIMEOUT && !found ) {
				double rotation = rand_between(0,2*CGAL_PI);
				p.rotation = rotation;
				p.vec[2] = rotation;
					if(queryHandler.isLegalConfiguration(p.xy,p.rotation)) {
						p.index = currInd;
						V.push_back(p);
						currInd++;
						found = true;
					}
				counter++;
			}

		}

	for (int i=0; i<Nbbox; i++) {
			int counter = 0;
			double x = (xmax+bsr);
				double y = (ymin-bsr)*(double)(i/Nbbox-1)+(ymax+bsr)*(1-double(i/Nbbox-1));
				qPoint p;
			p.getPoint(x,y);
			bool found = false;
			while (counter < TIMEOUT && !found ) {
				double rotation = rand_between(0,2*CGAL_PI);
				p.rotation = rotation;
				p.vec[2] = rotation;
					if(queryHandler.isLegalConfiguration(p.xy,p.rotation)) {
						p.index = currInd;
						V.push_back(p);
						currInd++;
						found = true;
					}
				counter++;
			}

		}



	//N random configurations

	int currRandPoints=0;
	int counter =0;
	while (counter < TIMEOUT && currRandPoints<Nrand ) {
		qPoint temp = newRandomQPoint(xmin,xmax,ymin,ymax);
			if(queryHandler.isLegalConfiguration(temp.xy,temp.rotation)) {
				temp.index = currInd;
				V.push_back(temp);
				currInd++;
				currRandPoints++;
				counter=0;
			}
		counter++;
	}

	int N=V.size();


	vector<vector<double>> graph(N, vector<double>(N,numeric_limits<double>::max())); //matrix representation of the graph



	// 0 - default, 1 - clockwise is best(/only option), (-1) - cc, 2 - no route
	short direction[N][N];


	for (int i=0; i<N; i++) {
		for (int j=0; j<N; j++) {
			direction[i][j]=0;
		}
	}

	Tree tree;

	tree.insert(V.begin(),V.end());

	for (qPoint q: V ) {

		K_neighbor_search search(tree, q, K);

		for(K_neighbor_search::iterator it = search.begin(); it != search.end(); it++){
	    	Neighbor n;
	    	qPoint q1 = it->first;
	    	n.p = q1;
	    		short dir = getDirection(direction[q.index][q1.index],q, q1, queryHandler);
	    		if(dir != 2) {
	    			n.isClockwise = (dir == 1);
	    			n.distance = dist(q, q1, n.isClockwise);
	    	    	direction[q.index][q1.index] = dir;
	    	    	direction[q1.index][q.index] = -dir;
	    	    	graph[q.index][n.p.index] = n.distance;
	    	    	graph[n.p.index][q.index] = n.distance;
	    		} else {
	    	    	direction[q.index][q1.index] = 2;
	    	    	direction[q1.index][q.index] = 2;
	    		}


	      }
	}



	vector<int> path = dijkstra(graph, N, /*index of source*/0, /*index of target*/1);


	// TODO : check if should include last step
	for(int i=0; i<path.size(); i++) {
		Path::PathMovement movement;
		movement.location = V[path[i]].xy;
		movement.rotation = V[path[i]].rotation;
			movement.orientation =
				( direction[V[path[i-1]].index][V[path[i]].index] == 1 ?
						CGAL::CLOCKWISE :
						CGAL::COUNTERCLOCKWISE);

		res.push_back(movement);
	}

    return res;
}
