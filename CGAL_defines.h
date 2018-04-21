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
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_extended_dcel.h>

typedef typename CGAL::Gmpq Number_type;
typedef typename CGAL::Cartesian<Number_type> Kernel;
typedef typename Kernel::FT FT;
typedef typename Kernel::Point_2 Point_2;
typedef typename Kernel::Vector_2 Vector_2  ;
typedef typename Kernel::Segment_2 Segment_2;
typedef CGAL::Arr_segment_traits_2<Kernel>             Traits_2;
typedef typename CGAL::Polygon_2<Kernel> Polygon_2;
typedef typename CGAL::Polygon_set_2<Kernel> Polygon_set_2;
//typedef CGAL::Arr_face_extended_dcel<Traits_2, bool>    Dcel;
//typedef CGAL::Arrangement_2<Traits_2>            Arrangement_2;
typedef Polygon_set_2::Arrangement_2				Arrangement_2;
typedef Arrangement_2::Face_handle 					Face;


#endif //RODQUERY_CGAL_DEFINES_H
