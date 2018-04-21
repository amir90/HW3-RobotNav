//
// Created by t-idkess on 08-Apr-18.
//

#include "MyQueryHandler.h"

MyQueryHandler::MyQueryHandler(const FT &rodLength, const vector<Polygon_2> &obstacles) {
    myLength = rodLength;
    myObstacles = obstacles;
}

bool MyQueryHandler::_isLegalConfiguration(const Point_2 &point, const Vector_2 &direction, const double rotation) {
	Segment_2 robot = Segment_2(point,point+(direction*this->myLength));
	//check zone created by obstacle arrangment
	//if includes face in the interior of an obstacle - return false;
    return true;
}
