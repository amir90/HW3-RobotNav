//
// Created by t-idkess on 09-Apr-18.
//

#include "MyRodPathFinder.h"
#include "MyQueryHandler.h"

struct qPoint {
	Point_2 xy;
	double rotation;
	int index;
};


double	dist(qPoint p1, qPoint p3) {
	//TODO: implement distance function
	wt = 0.5;
	wf = 0.5;
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

bool localPlanner (qPoint q1 ,qPoint q2, MyQueryHandler handler) {
	//incremental implementation
	steps = 50;
	bool breakflag = false;
	for (i=1; i<steps; i++) {

		if (!handler.isLegalConfiguration(q1.xy+((i/50)*q2.xy-CGAL::ORIGIN),(q1.rotation+(q2.rotation-q1.rotation)*i/50)) % (2*3.14)) {
			breakflag = true;
			break;
		}
	}

		if (!breakflag) return 1; //Counterclockwise rotation

		breakflag = false;

	for (i=1; i<steps; i++) {

		if (!handler.isLegalConfiguration(q1.xy+((i/50)*q2.xy-CGAL::ORIGIN),q2.rotation+(q1.rotation-q2.rotation)*i/50)) {
			break;
		}
	}
	if (!breakflag) return -1; //clockwise rotation

	return 0;

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

	int currInd = 0;
	int counter = 0;

	while (currInd<n && counter<timeout ) {

		qPoint temp;
		//TODO: get random qPoint for temp;
		queryHandler.isLegalConfiguration(Point_2(temp.x(),temp.y()),temp.z().to_double());
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
