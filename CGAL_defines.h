//
// Created by t-idkess on 08-Apr-18.
//

#ifndef RODQUERY_CGAL_DEFINES_H
#define RODQUERY_CGAL_DEFINES_H

#include <CGAL/Gmpq.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>

typedef typename CGAL::Gmpq Number_type;
typedef typename CGAL::Cartesian<Number_type> Kernel;
typedef typename Kernel::FT FT;
typedef typename Kernel::Point_2 Point_2;
typedef typename Kernel::Vector_2 Vector_2  ;
typedef typename Kernel::Segment_2 Segment_2;
typedef typename CGAL::Polygon_2<Kernel> Polygon_2;
typedef typename CGAL::Polygon_set_2<Kernel> Polygon_set_2;


#endif //RODQUERY_CGAL_DEFINES_H
