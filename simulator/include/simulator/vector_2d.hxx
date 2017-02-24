
#include "vector_2d.hpp"



namespace tools {

template<class T>
Vector2D<T>::Vector2D() {
    x_ = 0;
    y_ = 0;

}

template<class T>
Vector2D<T>::Vector2D(T x, T y) {
    x_ = x;
    y_ = y;

}

template<class T>
Vector2D<T>::~Vector2D() {
    // TODO Auto-generated destructor stub
}

template<class T>
Vector2D<T>::operator double() {

    return sqrt(x_ * x_ + y_ * y_);
}

template<class T>
double Vector2D<T>::dot(const Vector2D<T>& other_vector) const {

    return x_ * other_vector.x_ + y_ * other_vector.y_;

}

template<class T>
double Vector2D<T>::norm2() {
    return x_ * x_ + y_ * y_;
}

template<class T>
double Vector2D<T>::norm() {
    return sqrt(double ((double) this->x_ * (double) this->x_ + (double) this->y_ * (double) this->y_));
}

template<class T>
Vector2D<T> Vector2D<T>::operator *(const double a) const {
    Vector2D<T> vec_tmp;
    vec_tmp.x_ = a * x_;
    vec_tmp.y_ = a * y_;
    return vec_tmp;
}

template<class T>
double Vector2D<T>::scalarProduct(const Vector2D<T>& other_vector) const {
    double z =(x_ * other_vector.x_ + y_ * other_vector.y_);
    double n = sqrt((x_ * x_) + y_ * y_) * sqrt(other_vector.x_ * other_vector.x_ + other_vector.y_ * other_vector.y_);
    
    return std::acos(z / n);
}


template<class T>
bool Vector2D<T>::operator<(const Vector2D<T>& other_vector) const {
    return ((x_ * x_ + y_ * y_)
            < (other_vector.x_ * other_vector.x_
            + other_vector.y_ * other_vector.y_));
}

template<class T>
bool Vector2D<T>::isNormEqual(const Vector2D<T>& other_vector, const double& threshold) const {
    double a = sqrt(double (x_ * x_ + y_ * y_));
    double b = sqrt(double (other_vector.x_ * other_vector.x_ + other_vector.y_ * other_vector.y_));

    return fabs(a - b) < threshold;

    //  return fabs(((x_ * x_ + y_ * y_) - (other_vector.x_ * other_vector.x_ + other_vector.y_ * other_vector.y_))) < threshold;
}

template<class T>
bool Vector2D<T>::operator==(const Vector2D<T>& other_vector) const {
    return ( x_ == other_vector.x_ && y_ == other_vector.y_);
}

template<class T>
Vector2D<T> Vector2D<T>::operator +(const Vector2D<T>& other_vector) const {
    Vector2D<T> vec_tmp;
    vec_tmp.x_ = x_ + other_vector.x_;
    vec_tmp.y_ = y_ + other_vector.y_;
    return vec_tmp;
}

template<class T>
Vector2D<T> Vector2D<T>::operator -(const Vector2D<T>& other_vector) const {
    Vector2D<T> vec_tmp;
    vec_tmp.x_ = x_ - other_vector.x_;
    vec_tmp.y_ = y_ - other_vector.y_;
    return vec_tmp;
}

template<class T>
void Vector2D<T>::operator =(const Vector2D<T>& other_vector) {
    x_ = other_vector.x_;
    y_ = other_vector.y_;
}

} /* namespace tools */
