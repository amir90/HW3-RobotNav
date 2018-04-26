//
// Created by t-idkess on 09-Apr-18.
//

#include "MyRodPathFinder.h"
#include "MyQueryHandler.h"

#define STEPS 50
#define N 20
#define K 5
#define TIMEOUT 1000
#define TRANSLATION_WEIGHT 0.8

struct qPoint {
	Point_2 xy;
	double rotation;
	int index;
};

struct range {
	int start;
	int end;
};

FT globalRodLength;

bool legal_range(range r) {
	return r.start <= r.end;
}

double rand_between(double high, double low) {
	return low + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(high-low)));
}

qPoint newRandomQPoint(double xmin, double xmax, double ymin, double ymax) {
	double x = rand_between(xmin,xmax);
	double y = rand_between(ymin, ymax);
	double rotation = rand_between(0,2*CGAL_PI);

	qPoint p;
	p.xy = Point_2(x,y);
	p.rotation = rotation;

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
	return fmod(angle, CGAL_PI);
}

qPoint getPointAtStep(int i, qPoint q1, qPoint q2, bool isClockwise) {
	qPoint q;
	//double cwRotation = fixedAngle(q1.rotation + (q2.rotation-q1.rotation)*i/STEPS);
	//double ccwRotation = fixedAngle(q1.rotation+ (q2.rotation-q1.rotation)*i/STEPS);

	double ccwRotation = q2.rotation-q1.rotation>=0?q2.rotation-q1.rotation:q2.rotation-q1.rotation+2*CGAL_PI;
	ccwRotation = fixedAngle(q1.rotation+ccwRotation*i/50);
	double cwRotation = q2.rotation-q1.rotation>=0?-(2*CGAL_PI-(q2.rotation-q1.rotation)):q2.rotation-q1.rotation;
		cwRotation = fixedAngle(q1.rotation+cwRotation*i/50);

	double x1 = q1.xy.x().to_double(), y1 = q1.xy.y().to_double(),
			x2 = q2.xy.x().to_double(), y2 = q2.xy.y().to_double();
	double x = x1 + (i/50.0) * (x2-x1),
			y = y1 + (i/50.0) * (y2-y1);

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

vector<int> dijkstra(double graph[N][N], int V, int src, int target)
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
		cout << " path " << vertex << endl;
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
		bool isClockwise = countDirection;
		bool collides = false;

		queue<range> q;

		range r;
		r.start = 1;
		r.end = STEPS - 1;

		q.push(r);

		while(!q.empty()) {
			range r = q.front();
			if (q1.index==0 && q2.index==1) {
				std::cout<<"local planner: start "<<r.start<<" local planner: end "<<r.end<<endl;
			}
			q.pop();
			int mid = (r.end + r.start) / 2;
			qPoint qMid = getPointAtStep(mid, q1, q2, isClockwise);
			if(!handler.isLegalConfiguration(qMid.xy, qMid.rotation)) {
				collides = true;
				break;
			}

			// push to queue new ranges

			range left, right;
			left.start = r.start;
			left.end = mid-1;
			right.start = mid+1;
			right.end = r.end;

			if(legal_range(left))
				q.push(left);
			if(legal_range(right))
				q.push(right);
		}

		if(!collides)
			d[countDirection] = dist(q1, q2, isClockwise);
	}

	// no route
	if(d[0]<0 && d[1]<0) {
		if (q1.index==0 && q2.index==1) {
			std::cout<<"d[0] "<<d[0]<<" d[1] "<<d[1]<<endl;
		}
		return 2;
	}

	// clockwise is a valid route that is neccessarily shorter than cc
	if(d[0] < d[1] && (d[1]>=0 && d[0]>=0) || (d[0]>=0 && d[1]<0)) {
		if (q1.index==0 && q2.index==1) {
		std::cout<<"BAD PATH!"<<d[1]<<endl;
		}
		return 1;
	}
		return -1;
}

short getDirection(short direction[N][N], qPoint q1, qPoint q2, MyQueryHandler handler) {
	// route validation and direction was not found before
//	cout << "q1.index " << q1.index << endl;
//	cout << "q2.index " << q2.index << endl;
	//route was not found before
	if(direction[q1.index][q2.index] == 0) {
	//	cout << "DEBUG3" << endl;
		direction[q1.index][q2.index] = localPlanner(q1, q2, handler);
	//	cout << "DEBUG4" << endl;
		//some route was found - add symmetrical movement
		if(direction[q1.index][q2.index] != 2)
			if (q1.index==0 && q2.index==1) {
				std::cout<<"Bad!";
			}
			direction[q2.index][q1.index] = -direction[q1.index][q2.index];
	}

	return direction[q1.index][q2.index];
}


vector<Path::PathMovement>
MyRodPathFinder::getPath(FT rodLength, Point_2 rodStartPoint, double rodStartRotation, Point_2 rodEndPoint,
                         double rodEndRotation, vector<Polygon_2> obstacles) {
	globalRodLength = rodLength;
	vector<Path::PathMovement> res;
//	int n = 10; //number of nodes to put in the Roadmap
//	int k = 5; //number of closest neighbors to examine for each configuration
//	int timeout = 1000;
	qPoint V[N]; //Vertices;

	double graph[N][N]; //matrix representation of the graph

	MyQueryHandler queryHandler(rodLength,obstacles);
	

	//TODO : find solution
	CGAL::Bbox_2 bbox(10, -10, 10, -10);

	float bsr = 1.1;
	double xmin = bbox.xmin()*bsr, xmax = bbox.xmax()*bsr, 
	ymin = bbox.ymin()*bsr, ymax = bbox.ymax()*bsr;

	//wrong - the start point is not supposed to be part of the roadmap
	qPoint qstart, qend;
	qstart.xy = rodStartPoint;
	qstart.rotation = rodStartRotation;
	qstart.index = 0;
	qend.xy = rodEndPoint;
	qend.rotation = rodEndRotation;
	qend.index = 1;

	V[0] = qstart;
	V[1] = qend;

	int currInd = 2;
	int counter = 0;

	while (currInd < N && counter < TIMEOUT ) {

		qPoint temp = newRandomQPoint(xmin, xmax, ymin, ymax); 
		if(queryHandler.isLegalConfiguration(temp.xy,temp.rotation)) {
			temp.index = currInd;
			V[currInd] = temp;
			currInd++;
			counter=0;
		}

		counter++;
	}

	// 0 - default, 1 - clockwise is best(/only option), (-1) - cc, 2 - no route
	short direction[N][N] = {0};

	for (qPoint q: V ) {
	    std::set<Neighbor,setComp> neighbours{setComp{}};


	    for (qPoint q1: V) {
	    	Neighbor n;
	    	n.p = q1;
	    //	cout <<"Amir index:"<<q1.index<<endl;
    		//cout << "DEBUG1" << endl;
	    	if (neighbours.size() <  K) {
	    	//	cout << "DEBUG2" << endl;
	    		short dir = getDirection(direction,q, q1, queryHandler);

	    		// insert only if route is valid
	    		if(dir != 2) {
	    			n.isClockwise = (dir == 1);
	    			n.distance = dist(q, q1, n.isClockwise);
	    			neighbours.insert(n);
	    		}

	      }
	    	// q1 is suspected to be close enough - still need to validate route and direction
	    	else if ( dist_min(q1,q) < (std::next(neighbours.end(),-1))->distance) {
			  short dir = getDirection(direction, q, q1, queryHandler);

			  // insert only if route is valid
			  if(dir != 2) {
				  n.isClockwise = (dir == 1);
				  n.distance = dist(q, q1, n.isClockwise);
				  if(n.distance < (std::next(neighbours.end(),-1))->distance) {
					  neighbours.insert(n);
					  neighbours.erase(std::next(neighbours.end(),-1));
				  }
			  }
		  }
	    }

	    for(Neighbor n : neighbours) {
	    	graph[q.index][n.p.index] = n.distance;
	    	graph[n.p.index][q.index] = n.distance;
	    }
	}


	for (int i=0; i<N; i++) {
		for (int j=0; j<N; j++) {
			cout<<graph[i][j]<<" ";
		}
		cout<<endl;
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
