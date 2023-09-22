//
// Created by larr-desktop on 22. 7. 24.
//

#ifndef ZED_CHASING_UTILS_MATH_UTIL_H
#define ZED_CHASING_UTILS_MATH_UTIL_H

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <numeric>
#include <algorithm>    // std::sort, std::stable_sort
#include <mutex>
#include <string>
#include <iostream>


struct Velocity{
    float vx;
    float vy;
    float vz;
    Velocity(){vx=0.0,vy=0.0,vz=0.0;};
    Velocity(float vx_, float vy_, float vz_):vx(vx_),vy(vy_),vz(vz_){};
    Eigen::Vector3f toEigen() const{
        return Eigen::Vector3f (vx,vy,vz);
    }
    Eigen::Vector3d toEigend() const{
        return Eigen::Vector3d(vx,vy,vz);
    }
    Velocity operator+(const Velocity &vel) const{
        return Velocity(vel.vx+vx, vel.vy+vy,vel.vz+vz);
    }
    Velocity operator-(const Velocity &vel) const{
        return Velocity(vel.vx-vx, vel.vy-vy,vel.vz-vz);
    }
    Velocity operator*(const float &s) const{
        return Velocity(s*vx, s*vy,s*vz);
    }
    Velocity operator/(const float &s) const{
        return *this* (float)(1.0/s);
    }

    float dot(Velocity v) const{
        return vx*v.vx + vy*v.vy +vz*v.vz;
    }
    float norm() const {
        return (float)sqrt(pow(vx,2)+pow(vy,2)+pow(vz,2));
    }
    Velocity normalized() const{
        return *this *(float)(1.0/this->norm());
    }
};

struct Point{
    float x;
    float y;
    float z;

    Point(){x=0.0; y=0.0, z=0.0;};
    Point(float x_, float y_, float z_):x(x_),y(y_),z(z_){};
    Point(const Eigen::Vector3f &pnt):x(pnt(0)),y(pnt(1)),z(pnt(2)){};

    Eigen::Vector3f toEigen() const{
        return Eigen::Vector3f(x,y,z);
    }
    Eigen::Vector3d toEigend() const{
        return Eigen::Vector3d(x,y,z);
    }
    Point operator+(const Point &pnt) const{
        return Point(pnt.x+x, pnt.y+y,pnt.z+z);
    }
    Point operator-(const Point &pnt) const{
        return Point(x-pnt.x, y-pnt.y, z-pnt.z);
    }
    Point operator*(float s) const{
        return Point(s*x, s*y, s*z);
    }
    Point operator/(float s) const{
        return *this* (float)(1.0/s);
    }

    float distTo(Point p) const{
        return (float)sqrt(pow(p.x-x,2)+pow(p.y-y,2)+pow(p.z-z,2));
    }
    float dot(Point p) const{
        return x*p.x+y*p.y+z*p.z;
    }
    float norm() const {
        return (float)sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    }
    Point normalized() const{
        return *this *(float)(1.0/this->norm());
    }
};

struct Pose{
    Eigen::Transform<float,3,Eigen::Affine> poseMat;
    void inverse() {poseMat = poseMat.inverse();};

    Pose() {poseMat.setIdentity();}
    void setTranslation(const float &x, const float &y , const float &z) {poseMat.translate(Eigen::Vector3f(x,y,z));};
    void setTranslation(const Point &pnt){poseMat.translate(pnt.toEigen());};
    void setRotation(const Eigen::Quaternionf &quat){poseMat.rotate(quat);};

    Point getTranslation() const{
        return Point(poseMat.translation().x(),poseMat.translation().y(),poseMat.translation().z());
    }
    Eigen::Quaternionf getQuaternion() const{
        return Eigen::Quaternionf(poseMat.rotation());
    }
    void rotate(const Eigen::Vector3f &axis, const float &angle){
        poseMat.rotate(Eigen::AngleAxisf(angle,axis));
    }

    void applyTransform (const Pose& pose){
        poseMat = pose.poseMat * poseMat;
    }

};





#endif //ZED_CHASING_UTILS_MATH_UTIL_H
