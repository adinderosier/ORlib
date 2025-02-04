/**
 * @file vector2d.hpp
 * @brief Defines a 2D vector and provides vector operations.
 * @author Adin De'Rosier (adinderosier@live.com)
 * @version 0.2
 * @date 2023-09-27
 *
 * @note
 * This file defines a struct called Vector2D that represents a 2D
 * vector. It provides a set of operators for performing vector
 * operations such as addition, subtraction, multiplication, and
 * division. It also includes utility functions for vector operations
 * like dot product, vector length, and vector normalization.
 *
 * @warning
 * This file assumes that the input values are within the expected
 * range. No bounds checking is performed.
 *
 * @remark
 * This file is part of the ORlib::misc::math namespace.
 */

#ifndef _VECTOR2D_H_
#define _VECTOR2D_H_

#include <array>
#include <cmath>

namespace ORlib::misc::math
{

    /**
     * @brief Struct representing a 2D vector with overloaded operators.
     *
     * This struct provides a set of operators for performing vector
     * operations more concisely.
     */
    struct Vector2D
    {
        // The vector elements.
        double x, y;

        /**
         * @brief Construct a new Vector 2D object
         * 
         */
        Vector2D() : x(0.0), y(0.0) {}

        /**
         * @brief Construct a new Vector 2D object
         * 
         * @param x 
         * @param y 
         */
        Vector2D(double x, double y) : x(x), y(y) {}

        /**
         * @brief Copy a Vector 2D object
         * 
         * @param o 
         */
        Vector2D(const Vector2D &o) 
        {
            if (this != &o)
            {
                this->x = o.x, 
                this->y = o.y;
            }
        }

        /**
         * @brief Move a Vector 2D object
         * 
         * @param o 
         */
        Vector2D(Vector2D &&o) 
        {
            if (this != &o)
            {
                this->x = o.x, 
                this->y = o.y;
                o.x = 0.0;
                o.y = 0.0;
            }
        }

        /**
         * @brief Negates the vector.
         *
         * @return Vector2D The negated vector.
         */
        Vector2D operator-()
        {
            return {-x, -y};
        }

        /**
         * @brief Adds two vectors.
         *
         * @param o The vector to be added.
         * @return Vector2D The sum of the two vectors.
         */
        Vector2D operator+(const Vector2D &o)
        {
            return {x + o.x, y + o.y};
        }

        /**
         * @brief Subtracts a vector from another vector.
         *
         * @param o The vector to be subtracted.
         * @return Vector2D The difference between the two vectors.
         */
        Vector2D operator-(const Vector2D &o)
        {
            return {x - o.x, y - o.y};
        }

        /**
         * @brief Multiplies two vectors element-wise.
         *
         * @param o The vector to be multiplied.
         * @return Vector2D The element-wise product of the two vectors.
         */
        Vector2D operator*(const Vector2D &o)
        {
            return {x * o.x, y * o.y};
        }

        /**
         * @brief Divides two vectors element-wise.
         *
         * @param o The vector to be divided.
         * @return Vector2D The element-wise division of the two
         *         vectors.
         */
        Vector2D operator/(const Vector2D &o)
        {
            return {x / o.x, y / o.y};
        }

        /**
         * @brief Assigns a vector to another vector.
         *
         * @param o The vector to assign to.
         * @return Vector2D The reference to the updated vector.
         */
        Vector2D operator=(const Vector2D &o)
        {
            if (this != &o)
            {
                this->x = o.x,
                this->y = o.y;
            }
            return *this;
        }

        /**
         * @brief Assigns a vector to another vector.
         *
         * @param o The vector to assign to.
         * @return Vector2D The reference to the updated vector.
         */
        Vector2D operator=(Vector2D &&o)
        {
            if (this != &o)
            {
                this->x = o.x, 
                this->y = o.y;
                o.x = 0.0;
                o.y = 0.0;
            }
            return *this;
        }

        /**
         * @brief Adds another vector to the current vector and assigns
         *        the result.
         *
         * @param o The vector to be added.
         * @return Vector2D& The reference to the updated vector.
         */
        Vector2D &operator+=(Vector2D &o)
        {
            x += o.x, y += o.y;
            return *this;
        }

        /**
         * @brief Subtracts another vector from the current vector and
         *        assigns the result.
         *
         * @param o The vector to be subtracted.
         * @return Vector2D& The reference to the updated vector.
         */
        Vector2D &operator-=(Vector2D &o)
        {
            x -= o.x, y -= o.y;
            return *this;
        }

        /**
         * @brief Multiplies the current vector with another vector
         *        element-wise and assigns the result.
         *
         * @param o The vector to be multiplied.
         * @return Vector2D& The reference to the updated vector.
         */
        Vector2D &operator*=(Vector2D &o)
        {
            x *= o.x, y *= o.y;
            return *this;
        }

        /**
         * @brief Divides the current vector by another vector
         *        element-wise and assigns the result.
         *
         * @param o The vector to be divided.
         * @return Vector2D& The reference to the updated vector.
         */
        Vector2D &operator/=(Vector2D &o)
        {
            x /= o.x, y /= o.y;
            return *this;
        }

        // This is a stop gap until the codebase is made to use
        // Vector2D for bot coordinates
        /**
         * @brief Returns the vector as an array.
         *
         * @return std::array<double, 2> The vector as an array.
         */
        std::array<double, 2> std()
        {
            return {x, y};
        }

        /**
         * @brief Normalizes a vector to have unit length.
         *
         * @return Vector2D The normalized vector.
         */
        Vector2D normalize();

        /**
         * @brief Scales a vector with given scalar.
         *
         * @param scalar The value to scale the vector by.
         * @return Vector2D The scaled vector.
         */
        Vector2D scale(double scalar);

        /**
         * @brief Rotates a vector with given angle in radians.
         *
         * @param angle The angle to rotate the vector.
         * @return Vector2D The rotated vector.
         */
        Vector2D rotate(double angle);

        /**
         * @brief Returns an angle (in radians) from a given Vector2D.
         *
         * @return double The angle in radians.
         */
        double getAngle();
    
        /**
         * @brief Returns an angle from a given Vector2D.
         *
         * @return double The heading in degrees.
         */
        double getHeading();

        /**
         * @brief Returns the magnitude of a given Vector2D.
         *
         * @return double The magnitude.
         */
        double getMagnitude();
    };

    /**
     * @brief Multiplies a scalar value with a vector.
     *
     * @param s The scalar value to multiply.
     * @param v The vector to be multiplied.
     * @return Vector2D The resulting vector.
     */
    inline Vector2D operator*(double s, const Vector2D &v)
    {
        return {s * v.x, s * v.y};
    }

    /**
     * @brief Multiplies a vector with a scalar value.
     *
     * @param v The vector to be multiplied.
     * @param s The scalar value to multiply.
     * @return Vector2D The resulting vector.
     */
    inline Vector2D operator*(const Vector2D &v, double s)
    {
        return {s * v.x, s * v.y};
    }

    /**
     * @brief Divides a vector by a scalar value.
     *
     * @param v The vector to be divided.
     * @param s The scalar value to divide by.
     * @return Vector2D The resulting vector.
     */
    inline Vector2D operator/(const Vector2D &v, double s)
    {
        return {v.x / s, v.y / s};
    }

    /**
     * @brief Multiplies a vector by a scalar value and assigns the
     *        result to the vector.
     *
     * @param v The vector to be multiplied and assigned.
     * @param s The scalar value to multiply.
     * @return Vector2D& The reference to the multiplied vector.
     */
    inline Vector2D &operator*=(Vector2D &v, double s)
    {
        v.x *= s, v.y *= s;
        return v;
    }

    /**
     * @brief Divides a vector by a scalar value and assigns the result
     *        to the vector.
     *
     * @param v The vector to be divided and assigned.
     * @param s The scalar value to divide by.
     * @return Vector2D& The reference to the divided vector.
     */
    inline Vector2D &operator/=(Vector2D &v, double s)
    {
        v.x /= s, v.y /= s;
        return v;
    }

    /**
     * @brief Calculates the dot product of two vectors.
     *
     * @param a The first vector.
     * @param b The second vector.
     * @return double The dot product of the two vectors.
     */
    inline double dot(Vector2D &a, Vector2D &b)
    {
        return a.x * b.x + a.y * b.y;
    }

    /**
     * @brief Calculates the squared length of a vector.
     *
     * @param p The vector.
     * @return double The squared length of the vector.
     */
    inline double length2(Vector2D &p)
    {
        return p.x * p.x + p.y * p.y;
    }

    /**
     * @brief Calculates the squared length of a vector
     *        (rvalue version).
     *
     * @param p The vector.
     * @return double The squared length of the vector.
     */
    inline double length2(Vector2D &&p)
    {
        return p.x * p.x + p.y * p.y;
    }

    /**
     * @brief Calculates the length of a vector.
     *
     * @param p The vector.
     * @return double The length of the vector.
     */
    inline double length(Vector2D &p)
    {
        if (p.x == 0.0 && p.y == 0.0)
            return 0.0;
        else
            return std::sqrt(p.x * p.x + p.y * p.y);
    }

    /**
     * @brief Calculates the length of a vector (rvalue version).
     *
     * @param p The vector.
     * @return double The length of the vector.
     */
    inline double length(Vector2D &&p)
    {
        if (p.x == 0.0 && p.y == 0.0)
            return 0.0;
        else
            return std::sqrt(p.x * p.x + p.y * p.y);
    }

    /**
     * @brief Normalizes a vector to have unit length.
     *
     * @return Vector2D The normalized vector.
     */
    inline Vector2D Vector2D::normalize()
    {
        return *this / length(*this);
    }

    /**
     * @brief Scales a vector with given scalar.
     *
     * @param scalar The value to scale the vector by.
     * @return Vector2D The scaled vector.
     */
    inline Vector2D Vector2D::scale(double scalar)
    {
        *this = {x * scalar, y * scalar};
        return *this;
    }

    /**
     * @brief Rotates a vector with given angle (in radians).
     *
     * @param angle The angle to rotate the vector.
     * @return Vector2D The rotated vector.
     */
    inline Vector2D Vector2D::rotate(double angle)
    {
        *this = {x * cos(angle) - y * sin(angle), x * sin(angle) + y * cos(angle)};
        return *this;
    }

    /**
     * @brief Returns an angle (in radians) from a given Vector2D.
     *
     * @return double The angle in radians.
     */
    inline double Vector2D::getAngle()
    {
        return atan2(y, x);
    }

    /**
     * @brief Returns the magnitude of a given Vector2D.
     *
     * @return double The magnitude.
     */
    inline double Vector2D::getMagnitude()
    {
        return length(*this);
    }
} // namespace OITRC::misc::math

#endif //_VECTOR2D_H_
