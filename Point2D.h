#ifndef POINT2D_H
#define POINT2D_H

template <class T>
class Point2D
{
public:
  Point2D();
  ~Point2D();
  
  Point2D(T x, T y)
  : m_X(x),
    m_Y(y)
  {}
  
  T m_X; // puck position X captured by camera, in mm
  T m_Y; // puck position Y captured by camera, in mm  

  // operators
  Point2D operator + (const Point2D & rhs )
  {
    return Point2D( m_X + rhs.m_X, m_Y + rhs.m_Y );
  }

  Point2D operator - (const Point2D & rhs )
  {
    return Point2D( m_X - rhs.m_X, m_Y - rhs.m_Y );
  }

  Point2D operator * (const T rhs)
  {
    return Point2D( m_X * rhs, m_Y * rhs );
  }
  
  Point2D operator / (const T rhs)
  {
    return Point2D( m_X / rhs, m_Y / rhs );
  }
}; // Point2D


#endif
