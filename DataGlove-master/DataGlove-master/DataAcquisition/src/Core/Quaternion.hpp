#pragma once
#include "ArduinoJson.hpp"
#include <Eigen/Geometry>
#include <Eigen/Dense>

class Quaternion : public Eigen::Quaterniond
{
public:
    Quaternion()
    : Eigen::Quaterniond(1,0,0,0)
    {
    }

    Quaternion(double scalar)
    : Eigen::Quaterniond(scalar,0,0,0)
    {
    }

    Quaternion(double x, double y, double z, double w)
    : Eigen::Quaterniond(w,x,y,z)
    {
    }

    // This constructor allows you to construct Quaternion from Eigen expressions
    Quaternion(const Eigen::Quaterniond& other)
    : Eigen::Quaterniond(other)
    {
    }

    Quaternion(const Eigen::Vector3d &euler)
    : Eigen::Quaterniond(Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()))
    {
    }

    Quaternion(double ex, double ey, double ez)
    : Eigen::Quaterniond(Eigen::AngleAxisd(ex, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(ey, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(ez, Eigen::Vector3d::UnitZ()))
    {
    }

    // This method allows you to assign Eigen expressions to Quaternion
    Quaternion& operator=(const Eigen::Quaterniond& other)
    {
        this->Eigen::Quaterniond::operator=(other);
        return *this;
    }

    Quaternion& operator+=(const Quaternion &rhs)
    {
        this->x() = this->x() + rhs.x();
        this->y() = this->y() + rhs.y();
        this->z() = this->z() + rhs.z();
        this->w() = this->w() + rhs.w();
        return *this;
    }

    friend Quaternion operator+(Quaternion lhs, const Quaternion &rhs)
    {
        return Quaternion(lhs.x()+rhs.x(), lhs.y()+rhs.y(), lhs.z()+rhs.z(), lhs.w()+rhs.w());
    }

    friend Quaternion operator*(Quaternion q, const double &scalar)
    {
        return Quaternion(q.x()*scalar, q.y()*scalar, q.z()*scalar, q.w()*scalar);
    }

    friend Quaternion operator*(double scalar, const Quaternion &q)
    {
        return q * scalar;
    }

    Eigen::Vector3d eulerAngles(int a0 = 0, int a1 = 1, int a2 = 2) const
    {
        return this->toRotationMatrix().eulerAngles(a0, a1, a2);
    }

    JsonObject serialize(JsonObject obj) const
    {
        obj["x"] = x();
        obj["y"] = y();
        obj["z"] = z();
        obj["w"] = w();
        return obj;
    }

    JsonObject serialize(JsonArray parent) const
    {
        JsonObject obj = parent.createNestedObject();
        obj["x"] = x();
        obj["y"] = y();
        obj["z"] = z();
        obj["w"] = w();
        return obj;
    }

    template<unsigned int Capacity>
    JsonObject serialize(StaticJsonDocument<Capacity> parent) const
    {
        JsonObject obj = parent.createNestedObject();
        obj["x"] = x();
        obj["y"] = y();
        obj["z"] = z();
        obj["w"] = w();
        return obj;
    }
};
