
#ifndef TOOLS_VECTOR_2D_H_
#define TOOLS_VECTOR_2D_H_

#include <math.h>

namespace tools {

template<class T>
class Vector2D {
public:
    T x_;
    T y_;

    Vector2D<T>();
    Vector2D(T x, T y);

    virtual ~Vector2D();

    // returns the norm
    operator double();

    double dot(const Vector2D<T>& other_vector) const;
    double norm2();
    double norm();

    Vector2D<T> operator *(const double a) const;

    double scalarProduct(const Vector2D<T>& other_vector) const;


    bool operator<(const Vector2D<T>& other_vector) const;
    bool operator ==(const Vector2D<T>& other_vector) const;
    bool isNormEqual(const Vector2D<T>& other_vector, const double& threshold) const;

    Vector2D<T> operator +(const Vector2D<T>& other_vector) const;
    Vector2D<T> operator -(const Vector2D<T>& other_vector) const;
    void operator =(const Vector2D<T>& other_vector);
};

} /* namespace tools */

#include "vector_2d.hxx"
#endif /* TOOLS_VECTOR_2D_H_ */
