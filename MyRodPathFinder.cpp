//
// Created by t-idkess on 09-Apr-18.
//

#include "MyRodPathFinder.h"
#include "MyQueryHandler.h"

#define STEPS 50

struct qPoint {
	Point_2 xy;
	double rotation;
	int index;
};

struct range {
	int start;
	int end;
};

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

double	dist(qPoint p1, qPoint p3) {
	//TODO: implement distance function
	double wt = 0.5;
	double wf = 0.5;
	return 0;
}

struct setComp{

setComp(const qPoint &i) {this->i=i;}

bool operator()(const qPoint &a ,const qPoint &b) {
	if (dist(a,i)<dist(b,i))
	 return true;
	 else {
		 return false;
	 }
}
qPoint i;
};

// fix to range [0, 2pi)
double fixedAngle(double angle) {
	return fmod(angle, CGAL_PI) + CGAL_PI;
}

qPoint getPointAtStep(int i, qPoint q1, qPoint q2, bool isClockwise) {
	qPoint q;
	double cwRotation = fixedAngle(q1.rotation + (q1.rotation-q2.rotation)*i/STEPS);
	double ccwRotation = fixedAngle(q1.rotation+ (q2.rotation-q1.rotation)*i/STEPS);

	q.xy = q1.xy+((i/50)*q2.xy-CGAL::ORIGIN);
	q.rotation = (isClockwise ? cwRotation : ccwRotation);

	return q;
}

bool localPlanner (qPoint q1 ,qPoint q2, MyQueryHandler handler) {
	int countDirection = 0;

	while(countDirection < 2) {
		bool isClockwise = countDirection == 0;
		int collides = false;

		queue<range *> q;

		range r;
		r.start = 1;
		r.end = STEPS - 1;

		q.push(&r);

		while(!q.empty()) {
			range* r = q.pop();
			int mid = (r->end + r->start) / 2;
			qPoint qMid = getPointAtStep(mid, q1, q2, isClockwise);
			if(!handler.isLegalConfiguration(qMid.xy, qMid.rotation)) {
				collides = true;
				break;
			}

			// push to queue new ranges

			range left, right;
			left.start = r->start;
			left.end = mid-1;
			right.start = mid+1;
			right.end = r-> end;

			if(legal_range(left))
				q.push(&left);
			if(legal_range(right))
				q.push(&right);
		}

		if(!collides)
			return true;
	}

	return false;
}


vector<Path::PathMovement>
MyRodPathFinder::getPath(FT rodLength, Point_2 rodStartPoint, double rodStartRotation, Point_2 rodEndPoint,
                         double rodEndRotation, vector<Polygon_2> obstacles) {

	vector<Path::PathMovement> res;
	int n = 10; //number of nodes to put in the Roadmap
	int k = 5; //number of closest neighbors to examine for each configuration
	int timeout = 1000;
	qPoint V[n]; //Vertices;

	int graph[n][n]; //matrix representation of the graph

	MyQueryHandler queryHandler(rodLength,obstacles);
	
	CGAL::Bbox_2 bbox(queryHandler.arr.vertices_begin(),queryHandler.arr.vertices_end());

	float bsr = 1.1;
	double xmin = bbox.xmin()*bsr, xmax = bbox.xmax()*bsr, 
	ymin = bbox.ymin()*bsr, ymax = bbox.ymax()*bsr;

	int currInd = 0;
	int counter = 0;

	while (currInd<n && counter<timeout ) {

		qPoint temp = newRandomQPoint(xmin, xmax, ymin, ymax); 
		queryHandler.isLegalConfiguration(temp.xy,temp.rotation);
		V[currInd] = temp;
		currInd++;
	}

	for (qPoint q: V ) {
		std::set<qPoint,setComp> neighbours{setComp{q}};
		for ( qPoint q1: V) {
			if (neighbours.size()<k) {
			neighbours.insert(q1);
			} else if (dist(q1,q)<dist(*(--neighbours.end()),q)) {
				neighbours.insert(q1);
				neighbours.erase(--neighbours.end());
			}
		}
		for (qPoint q2: neighbours) {
				graph[q.index][q2.index]=d;
				}
	}





	//TODO:	Use Roadmap to find path (using diskstra)


	/*
    vector<Path::PathMovement> res;
    res.emplace_back(rodStartPoint, rodStartRotation, CGAL::CLOCKWISE);
    res.emplace_back(rodStartPoint, ((3 * CGAL_PI) / 2), CGAL::CLOCKWISE);
    res.emplace_back(Point_2(0, 1), (CGAL_PI / 2), CGAL::CLOCKWISE);
    res.emplace_back(Point_2(1, 1), 0, CGAL::CLOCKWISE);
    res.emplace_back(rodEndPoint, rodEndRotation, CGAL::COUNTERCLOCKWISE);
    */
    return res;
}
