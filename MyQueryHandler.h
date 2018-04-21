//
// Created by t-idkess on 08-Apr-18.
//

#ifndef RODQUERY_MYQUERYHANDLER_H
#define RODQUERY_MYQUERYHANDLER_H


#include "IQueryHandler.h"

class MyQueryHandler : public IQueryHandler {
public:
    MyQueryHandler(const FT &rodLength, const vector<Polygon_2> &obstacles);
protected:
    bool _isLegalConfiguration(const Point_2 &point, const Vector_2 &direction,const double rotation) override;
};


#endif //RODQUERY_MYQUERYHANDLER_H
